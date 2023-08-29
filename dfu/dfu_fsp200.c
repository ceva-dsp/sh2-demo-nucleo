/*
 * Copyright 2018-21 CEVA, Inc.
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License and 
 * any applicable agreements you may have with CEVA, Inc.
 * You may obtain a copy of the License at
 *
 *   http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

/*
 * FSP200 DFU (Download Firmware Update) Implementation.
 */

#include "dfu.h"

#include <stdbool.h>
#include <stdint.h>
#include <string.h>
#include "sh2_err.h"
#include "shtp.h"
#include "sh2_hal.h"
#include "sh2_hal_init.h"
#include "firmware.h"

#define CHAN_BOOTLOADER_CONTROL    (1)


#define DFU_MAX_ATTEMPTS (5)

// DFU should complete in about 36 seconds.  Set timeout at 60.
#define DFU_TIMEOUT_US (240000000)  // Can take up to 240 sec at 9600 baud.

// ------------------------------------------------------------------------
// Type definitions

// Bootloader message ids
typedef enum {
    ID_PRODID_REQ  = 0xe1,
    ID_PRODID_RESP = 0xe2,
    ID_OPMODE_REQ  = 0xe3,
    ID_OPMODE_RESP = 0xe4,
    ID_STATUS_REQ  = 0xe5,
    ID_STATUS_RESP = 0xe6,
    ID_WRITE_REQ   = 0xe7,
    ID_WRITE_RESP  = 0xe8,
} ReportId_t;

// Bootloader operating modes
typedef enum {
    OPMODE_BOOTLOADER,
    OPMODE_UPGRADE,
    OPMODE_VALIDATE,
    OPMODE_APPLICATION,
} OpMode_t;

// Flags in status word
#define STATUS_LAUNCH_APPLICATION 0x00000001
#define STATUS_LAUNCH_BOOTLOADER  0x00000002
#define STATUS_UPGRADE_STARTED    0x00000004
#define STATUS_VALIDATE_STARTED   0x00000008
#define STATUS_APP_VALID          0x00000010
#define STATUS_APP_INVALID        0x00000020
#define STATUS_DFU_IMAGE_VALID    0x00000040
#define STATUS_DFU_IMAGE_INVALID  0x00000080
#define STATUS_ERROR              0x40000000
#define STATUS_SOURCE             0x80000000

// Error codes
typedef enum {
    DFU_NO_ERROR = 0,
    DFU_UNEXPECTED_CMD,
    DFU_INVALID_APPLICATION,
    DFU_FLASH_ERASE_ERR,
    DFU_FLASH_WRITE_ERR,
    DFU_FLASH_LOCK_ERR,
    DFU_FLASH_OVERFLOW,
    DFU_INVALID_IMAGE_TYPE,
    DFU_INVALID_IMAGE_SIZE,
    DFU_INVALID_IMAGE_VERSION,
    DFU_INCOMPATIBLE_HARDWARE,
    DFU_RESERVED_0B,
    DFU_RESERVED_0C,
    DFU_IMAGE_LEN_MISMATCH,
    DFU_INVALID_APP_SIZE_DFU_IMAGE,
    DFU_INVALID_APP_CRC_DFU_IMAGE,
    DFU_INVALID_IMAGE_CRC,
    DFU_INVALID_PAYLOAD_LENGTH,
    DFU_INVALID_DATA_OFFSET,
} DfuErr_t;

// States of DFU process
typedef enum {
    ST_INIT,
    ST_GETTING_VER,
    ST_SETTING_MODE,
    ST_WAIT_STATUS,
    ST_SENDING_DATA,
    ST_WAIT_COMPLETION,
    ST_LAUNCHING,
    ST_FINISHED,
} DfuState_t;

// DFU State machine message handler typedef
typedef DfuState_t (*DfuMsgHdlr_t)(uint8_t *payload, uint16_t len);

// DFU State machine timeout handler typedef
typedef DfuState_t (*DfuTimeoutHdlr_t)(void);

// DFU State machine state info
typedef struct {
    DfuMsgHdlr_t msgHdlr;
    DfuTimeoutHdlr_t timeoutHdlr;
    uint32_t maxTime_us;
} DfuTransition_t;

// DFU data
typedef struct {
    sh2_Hal_t *pHal;
    void *pShtp;

    int status;
    bool firmwareOpened;
    const HcBin_t *firmware;
    uint32_t appLen;

    uint16_t wordOffset;      // offset of next 32-bit word to write
    uint16_t writeLen;        // length of last write, in words

    uint32_t ignoredResponses;

    uint32_t intervalStart_us;
    DfuState_t state;
} Dfu_t;



// ------------------------------------------------------------------------
// Forward declarations

// ------------------------------------------------------------------------
// Static data
static Dfu_t dfu_;

// ------------------------------------------------------------------------
// Private functions

static void initState(void)
{
    dfu_.status = SH2_OK;
    dfu_.firmwareOpened = false;
    dfu_.appLen = 0;

    dfu_.ignoredResponses = 0;

    uint32_t now_us = dfu_.pHal->getTimeUs(dfu_.pHal);
    dfu_.intervalStart_us = now_us;
    dfu_.state = ST_INIT;
}

static void openFirmware()
{
    if (dfu_.firmware == 0) {
        // No firmware object
        dfu_.status = SH2_ERR_BAD_PARAM;
        return ;
    }

    // Open the hcbin object
    int rc = dfu_.firmware->open();
    if (rc != 0) {
        dfu_.status = SH2_ERR;
        return;
    }
    dfu_.firmwareOpened = true;
    
/*    
    // Validate firmware matches this implementation
    const char *fmt = dfu_.firmware->getMeta("FW-Format");
    const char *pn = dfu_.firmware->getMeta("SW-Part-Number");
    if ((fmt == 0) || (pn == 0)) {
        // missing format or part number
        dfu_.status = SH2_ERR_BAD_PARAM;
        return;
    }

    if ((strcmp(fmt, "EFM32_V1") == 0) && 
        (strcmp(pn, "1000-4095") == 0)) {
           // valid FSP200 firmware
    }
    else if ((strcmp(fmt, "RA2L1_V1") == 0) && 
             (strcmp(pn, "1000-4818") == 0)) {
            // valid FSP201 Firmware
    }
    else {
        // Invalid values for format and/or part number
        dfu_.status = SH2_ERR_BAD_PARAM;
        return;   
    }
*/

    // Validate firmware length
    dfu_.appLen = dfu_.firmware->getAppLen();
    if (dfu_.appLen < 1024) {
        // App data isn't real
        dfu_.status = SH2_ERR_BAD_PARAM;
        return;
    }
    if ((dfu_.appLen & 0x00000003) != 0) {
        // appLen should be a multiple of 4.
        dfu_.status = SH2_ERR_BAD_PARAM;
        return;
    }
}

static uint32_t getU32(uint8_t *payload, unsigned offset)
{
    uint32_t value = 0;

    value =
        (payload[offset]) +
        (payload[offset+1] << 8) +
        (payload[offset+2] << 16) +
        (payload[offset+3] << 24);

    return value;
}

static uint16_t getU16(uint8_t *payload, unsigned offset)
{
    uint16_t value = 0;

    value =
        (payload[offset]) +
        (payload[offset+1] << 8);

    return value;
}

static uint8_t getU8(uint8_t *payload, unsigned offset)
{
    uint16_t value = 0;

    value = payload[offset];

    return value;
}

typedef struct {
    uint8_t reportId;
    uint8_t reserved1;
} OpProdIdRequest_t;

typedef struct {
    uint8_t reportId;
    uint8_t opMode;
} OpModeRequest_t;

static void requestUpgrade(void)
{
    OpModeRequest_t req;

    req.reportId = ID_OPMODE_REQ;
    req.opMode = OPMODE_UPGRADE;
    
    shtp_send(dfu_.pShtp, CHAN_BOOTLOADER_CONTROL, (uint8_t *)&req, sizeof(req));
}

static void requestProdId(void)
{
    OpProdIdRequest_t req;

    req.reportId = ID_PRODID_REQ;
    req.reserved1 = 0;
    
    shtp_send(dfu_.pShtp, CHAN_BOOTLOADER_CONTROL, (uint8_t *)&req, sizeof(req));
}

static DfuState_t handleInitStatus(uint8_t *payload, uint16_t len)
{
    // Only process ID_STATUS_RESP
    uint8_t reportId = payload[0];
    if (reportId != ID_STATUS_RESP) return dfu_.state;
    
    DfuState_t nextState = dfu_.state;
    uint32_t status = getU32(payload, 4);
    uint32_t errCode = getU32(payload, 8);

    // Make sure status says we can proceed.
    if (status & STATUS_LAUNCH_BOOTLOADER) {
        // Issue request for product id
        requestProdId();
        nextState = ST_GETTING_VER;
    }
    else {
        // Can't start
        dfu_.status = SH2_ERR_HUB;
        nextState = ST_FINISHED;
    }
    
    return nextState;
}

static DfuState_t handleProdId(uint8_t *payload, uint16_t len)
{
    // Only process ID_STATUS_RESP
    uint8_t reportId = payload[0];
    if (reportId != ID_PRODID_RESP) return dfu_.state;
    
    DfuState_t nextState = dfu_.state;
    volatile uint32_t part_number = getU32(payload, 4);
    volatile uint8_t  ver_major = getU8(payload, 8);
    volatile uint8_t  ver_minor = getU8(payload, 9);
    volatile uint16_t ver_patch = getU16(payload, 10);
    volatile uint32_t build_num = getU32(payload, 12);
        
    // Issue request to start download
    requestUpgrade();
    nextState = ST_SETTING_MODE;
    
    return nextState;
}

typedef struct {
    uint8_t reportId;
    uint8_t length;
    uint8_t wordOffset_lsb;
    uint8_t wordOffset_msb;
    uint8_t data[16*4];
} WriteRequest_t;

static void requestWrite(void)
{
    WriteRequest_t req;

    // How many words to write next
    dfu_.writeLen = (dfu_.appLen/4) - dfu_.wordOffset;
    if (dfu_.writeLen > 16) {
        dfu_.writeLen = 16;
    }

    req.reportId = ID_WRITE_REQ;
    req.length = dfu_.writeLen;
    req.wordOffset_lsb = dfu_.wordOffset & 0xFF;
    req.wordOffset_msb = (dfu_.wordOffset >>8) & 0xFF;
    dfu_.firmware->getAppData(req.data, dfu_.wordOffset*4, dfu_.writeLen*4);
    
    int writeLen = dfu_.writeLen*4 + 4;
    shtp_send(dfu_.pShtp, CHAN_BOOTLOADER_CONTROL, (uint8_t *)&req, writeLen);
}

static DfuState_t handleModeResponse(uint8_t *payload, uint16_t len)
{
    // Only process mode reponse
    uint8_t reportId = payload[0];
    if (reportId != ID_OPMODE_RESP) return dfu_.state;
    
    DfuState_t nextState = dfu_.state;
    uint8_t opMode = payload[1];
    uint8_t opModeStatus = payload[2];

    // Make sure transition to upgrade mode succeeded
    if ((opMode == OPMODE_UPGRADE) &&
        (opModeStatus == 0)) {
        nextState = ST_WAIT_STATUS;
    }
    else {
        // Failed to start upgrade mode
        dfu_.status = SH2_ERR_HUB;
        nextState = ST_FINISHED;
    }
    
    return nextState;
}

static DfuState_t handleUpgradeStatusResponse(uint8_t *payload, uint16_t len)
{
    // Only process status reponse
    uint8_t reportId = payload[0];
    if (reportId != ID_STATUS_RESP) return dfu_.state;
    
    DfuState_t nextState = dfu_.state;
    uint8_t opMode = payload[1];
    uint32_t status = getU32(payload, 4);
    uint32_t error = getU32(payload, 8);

    // Make sure transition to upgrade mode succeeded
    if ((opMode == OPMODE_UPGRADE) &&
        ((status&0x0FFFFFFF) == STATUS_UPGRADE_STARTED) &&
        (error == 0)) {
        dfu_.wordOffset = 0;
        requestWrite();
        nextState = ST_SENDING_DATA;
    }
    else {
        // Failed to start upgrade mode
        dfu_.status = SH2_ERR_HUB;
        nextState = ST_FINISHED;
    }
    
    return nextState;
}

static DfuState_t handleWriteResponse(uint8_t *payload, uint16_t len)
{
    // Only process mode reponse
    uint8_t reportId = payload[0];
    if (reportId != ID_WRITE_RESP) return dfu_.state;
    
    DfuState_t nextState = dfu_.state;
    uint8_t writeStatus = payload[1];
    uint16_t wordOffset = getU16(payload, 2);

    if (writeStatus == 0) {
        dfu_.wordOffset += dfu_.writeLen;
        if (dfu_.wordOffset*4 == dfu_.appLen) {
            // Now we wait for final status update
            nextState = ST_WAIT_COMPLETION;
        }
        else {
            // update offset and issue new write
            requestWrite();
            nextState = ST_SENDING_DATA;
        }
    }
    else {
        // Errored out
        dfu_.status = SH2_ERR_HUB;
        nextState = ST_FINISHED;
    }

    return nextState;
}

static void requestLaunch(void)
{
    OpModeRequest_t req;

    req.reportId = ID_OPMODE_REQ;
    req.opMode = OPMODE_APPLICATION;
    
    shtp_send(dfu_.pShtp, CHAN_BOOTLOADER_CONTROL, (uint8_t *)&req, sizeof(req));
}

static DfuState_t handleFinalStatus(uint8_t *payload, uint16_t len)
{
    // Only process mode reponse
    uint8_t reportId = payload[0];
    if (reportId != ID_STATUS_RESP) return dfu_.state;
    
    DfuState_t nextState = dfu_.state;

    uint32_t status = getU32(payload, 4);
    uint32_t errCode = getU32(payload, 8);

    if ((status & STATUS_APP_VALID) &&
        ((status & STATUS_ERROR) == 0) &&
        (errCode == DFU_NO_ERROR)) {
        requestLaunch();
        dfu_.status = SH2_OK;
        nextState = ST_LAUNCHING;
    }
    else {
        dfu_.status = SH2_ERR_HUB;
        nextState = ST_FINISHED;
    }

    return nextState;
}

static DfuState_t handleLaunchResp(uint8_t *payload, uint16_t len)
{
    // Only process mode reponse
    uint8_t reportId = payload[0];
    if (reportId != ID_OPMODE_RESP) return dfu_.state;
    
    DfuState_t nextState = dfu_.state;

    uint8_t opMode = payload[1];
    uint8_t opModeStatus = payload[2];

    if ((opMode == OPMODE_APPLICATION) &&
        (opModeStatus == 0)) {
        dfu_.status = SH2_OK;
        nextState = ST_FINISHED;
    }
    else {
        dfu_.status = SH2_ERR_HUB;
        nextState = ST_FINISHED;
    }

    return nextState;
}

static DfuState_t handleLaunchTimeout()
{
    // It's OK if we don't get launch confirmation.  Since we didn't
    // see a definite failure before the timeout, assume it succeeded
    // and application was launched.
    dfu_.status = SH2_OK;

    return ST_FINISHED;
}

static DfuState_t handleFinishedResp(uint8_t *payload, uint16_t len)
{
    // Does nothing, ignores messages.
    return dfu_.state;
}

static DfuState_t handleGeneralTimeout()
{
    // Timeouts are generally failures
    dfu_.status = SH2_ERR_TIMEOUT;
    return ST_FINISHED;
}

DfuTransition_t dfuStates[] = {
    {handleInitStatus, handleGeneralTimeout, 0},                // ST_INIT
    {handleProdId, handleGeneralTimeout, 10000},                // ST_GETTING_VER
    {handleModeResponse, handleGeneralTimeout, 10000},          // ST_SETTING_MODE
    {handleUpgradeStatusResponse, handleGeneralTimeout, 10000}, // ST_WAIT_STATUS
    {handleWriteResponse, handleGeneralTimeout, 2000000},       // ST_SENDING_DATA
    {handleFinalStatus, handleGeneralTimeout, 500000},          // ST_WAIT_COMPLETION
    {handleLaunchResp, handleLaunchTimeout, 100000},            // ST_LAUNCHING
    {handleFinishedResp, handleGeneralTimeout, 0}               // ST_FINISHED
};

static void hdlr(void *cookie, uint8_t *payload, uint16_t len, uint32_t timestamp)
{
    // Find entry for the current state.
    DfuTransition_t *tr = &dfuStates[dfu_.state];
    
    // Pass this message through state-specific handler
    dfu_.state = tr->msgHdlr(payload, len);
    
    // reset timeout logic
    uint32_t now_us = dfu_.pHal->getTimeUs(dfu_.pHal);
    dfu_.intervalStart_us = now_us;
}

static void timeout()
{
    // Find entry for the current state.
    DfuTransition_t *tr = &dfuStates[dfu_.state];
    
    // Pass this message through state-specific handler
    dfu_.state = tr->timeoutHdlr();
    
    // reset timeout logic
    uint32_t now_us = dfu_.pHal->getTimeUs(dfu_.pHal);
    dfu_.intervalStart_us = now_us;
}

// ------------------------------------------------------------------------
// Public API

int dfu(void)
{
    uint32_t start_us;
    uint32_t now_us;

    // Create the HAL instance used for DFU.
    dfu_.pHal = dfu_hal_init();
    
    // Initialize state
    initState();
    
    // Open firmware and validate it
    dfu_.firmware = &firmware;
    openFirmware();
    if (dfu_.status != SH2_OK) {
        goto fin;
    }
    
    // Initialize SHTP layer
    dfu_.pShtp = shtp_open(dfu_.pHal);
    
    // Register handlers for DFU-oriented channels
    shtp_listenChan(dfu_.pShtp, CHAN_BOOTLOADER_CONTROL, hdlr, &dfu_);

    // service SHTP until DFU process completes
    now_us = dfu_.pHal->getTimeUs(dfu_.pHal);
    start_us = now_us;
    while (((now_us - start_us) < DFU_TIMEOUT_US) &&
           (dfu_.state != ST_FINISHED))
    {
        DfuState_t state = dfu_.state;
        uint32_t maxTime_us = dfuStates[state].maxTime_us;
        
        if ((maxTime_us > 0) &&
            (maxTime_us <= (now_us - dfu_.intervalStart_us))) {
            // A timeout event has occurred.
            timeout();
        }
        
        shtp_service(dfu_.pShtp);
        now_us = dfu_.pHal->getTimeUs(dfu_.pHal);
    }
    
    if (dfu_.state != ST_FINISHED) {
        dfu_.status = SH2_ERR_TIMEOUT;
    }

    // close SHTP
    shtp_close(dfu_.pShtp);
    dfu_.pShtp = 0;

fin:
    if (dfu_.firmwareOpened) {
        dfu_.firmware->close();
        dfu_.firmwareOpened = false;
    }
    
    return dfu_.status;
}

