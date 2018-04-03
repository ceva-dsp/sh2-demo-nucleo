/*
 * Copyright 2018 Hillcrest Laboratories, Inc.
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License and 
 * any applicable agreements you may have with Hillcrest Laboratories, Inc.
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
#include "firmware-fsp200.h"

#define MAX_PACKET_LEN (64)
#define DFU_MAX_ATTEMPTS (5)

// DFU should complete in less than 30 seconds.
#define DFU_TIMEOUT_US (30000000)

// ------------------------------------------------------------------------
// Type definitions

#define GUID_BOOTLOADER (10)

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
    ST_SETTING_MODE,
    ST_SENDING_DATA,
    ST_WAIT_COMPLETION,
    ST_LAUNCHING,
    ST_FINISHED,
} DfuState_t;

// DFU State machine Action typedef
typedef DfuState_t (*DfuAction_t)(uint8_t *payload, uint16_t len);

// DFU State machine transition typedef
typedef struct {
    DfuState_t state;
    uint8_t reportId;
    DfuAction_t action;
} DfuTransition_t;

// DFU data
typedef struct {
    sh2_Hal_t *pHal;
    void *pShtp;

    int status;
    bool firmwareOpened;
    const HcBin_t *firmware;
    uint32_t appLen;
    uint8_t packetLen;

    uint16_t wordOffset;      // offset of next 32-bit word to write
    uint16_t writeLen;        // length of last write, in words

    uint32_t ignoredResponses;

    uint8_t controlChan;  // Channel number for DFU over SHTP
    
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
    dfu_.packetLen = 0;

    dfu_.ignoredResponses = 0;

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

    // Validate firmware matches this implementation
    const char *s = dfu_.firmware->getMeta("FW-Format");
    if ((s == 0) || (strcmp(s, "EFM32_V1") != 0)) {
        // No format info or Incorrect format
        dfu_.status = SH2_ERR_BAD_PARAM;
        return;
    }

    // Validate firmware is for the right part number
    s = dfu_.firmware->getMeta("SW-Part-Number");
    if (s == 0) {
        // No part number info
        dfu_.status = SH2_ERR_BAD_PARAM;
        return;
    }
    if (strcmp(s, "1000-4095") != 0) {
        // Incorrect part number
        dfu_.status = SH2_ERR_BAD_PARAM;
        return;
    }

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

    // Determine packet length to use
    dfu_.packetLen = dfu_.firmware->getPacketLen();
    if ((dfu_.packetLen == 0) || (dfu_.packetLen > MAX_PACKET_LEN)) {
        dfu_.packetLen = MAX_PACKET_LEN;
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

typedef struct {
    uint8_t reportId;
    uint8_t opMode;
} OpModeRequest_t;

static void requestUpgrade(void)
{
    uint8_t chan = dfu_.controlChan;
    OpModeRequest_t req;

    req.reportId = ID_OPMODE_REQ;
    req.opMode = OPMODE_UPGRADE;
    
    shtp_send(dfu_.pShtp, chan, (uint8_t *)&req, sizeof(req));
}

static DfuState_t handleInitStatus(uint8_t *payload, uint16_t len)
{
    DfuState_t nextState = dfu_.state;
    uint32_t status = getU32(payload, 4);
    uint32_t errCode = getU32(payload, 8);

    // At this point, advertisements should be done, we can determine
    // the channel number to use for DFU
    dfu_.controlChan = shtp_chanNo(dfu_.pShtp, "Bootloader", "control");
    if (dfu_.controlChan == 0xFF) {
        // Couldn't resolve channel no!
        dfu_.status = SH2_ERR_IO;
        nextState = ST_FINISHED;
    }
    else {
        // Make sure status says we can proceed.
        if (status & STATUS_LAUNCH_BOOTLOADER) {
            // Issue request to start download
            requestUpgrade();
            nextState = ST_SETTING_MODE;
        }
        else {
            // Can't start
            dfu_.status = SH2_ERR_HUB;
            nextState = ST_FINISHED;
        }
    }
    
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
    uint8_t chan = dfu_.controlChan;
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
    
    shtp_send(dfu_.pShtp, chan, (uint8_t *)&req, sizeof(req));
}

static DfuState_t handleModeResponse(uint8_t *payload, uint16_t len)
{
    DfuState_t nextState = dfu_.state;
    uint8_t opMode = payload[1];
    uint8_t opModeStatus = payload[2];

    // Make sure transition to upgrade mode succeeded
    if ((opMode == OPMODE_UPGRADE) &&
        (opModeStatus == 0)) {
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
    uint8_t chan = dfu_.controlChan;
    OpModeRequest_t req;

    req.reportId = ID_OPMODE_REQ;
    req.opMode = OPMODE_APPLICATION;
    
    shtp_send(dfu_.pShtp, chan, (uint8_t *)&req, sizeof(req));
}

static DfuState_t handleFinalStatus(uint8_t *payload, uint16_t len)
{
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

DfuTransition_t dfuStateTransition[] = {
    {ST_INIT, ID_STATUS_RESP, handleInitStatus},
    {ST_SETTING_MODE, ID_OPMODE_RESP, handleModeResponse},
    {ST_SENDING_DATA, ID_WRITE_RESP, handleWriteResponse},
    {ST_WAIT_COMPLETION, ID_STATUS_RESP, handleFinalStatus},
    {ST_LAUNCHING, ID_OPMODE_RESP, handleLaunchResp},
};

static DfuTransition_t *findTransition(DfuState_t state, uint8_t reportId)
{
    for (int n = 0; n < sizeof(dfuStateTransition)/sizeof(dfuStateTransition[0]); n++) {
        if ((dfuStateTransition[n].state == state) &&
            (dfuStateTransition[n].reportId == reportId)) {
            // Found the entry for this state, reportId
            return &dfuStateTransition[n];
        }
    }

    // Didn't find a match
    return 0;
}

static void hdlr(void *cookie, uint8_t *payload, uint16_t len, uint32_t timestamp)
{
    uint8_t reportId = payload[0];

    // Find a state machine table entry matching current state and report id.
    DfuTransition_t *pEntry = findTransition(dfu_.state, reportId);

    if (pEntry) {
        // Take the prescribed action for this transition and assign new state
        dfu_.state = pEntry->action(payload, len);
    }
    else {
        // Unexpected event/state combination.  Ignore.
        dfu_.ignoredResponses++;
    }
}

// ------------------------------------------------------------------------
// Public API

int dfu(void)
{
    uint32_t start_us;
    uint32_t now_us;

    // Initialize state
    initState();

    // Open firmware and validate it
    dfu_.firmware = &firmware;
    openFirmware();
    if (dfu_.status != SH2_OK) {
        goto fin;
    }

    // Create the HAL instance used for DFU.
    dfu_.pHal = fsp200_dfu_hal_init();
    
    // Initialize SHTP layer
    dfu_.pShtp = shtp_open(dfu_.pHal);
    
    // Register handlers for DFU-oriented channels
    shtp_listenChan(dfu_.pShtp, GUID_BOOTLOADER, "control", hdlr, &dfu_);

    // service SHTP until DFU process completes
    now_us = dfu_.pHal->getTimeUs(dfu_.pHal);
    start_us = now_us;
    while (((now_us - start_us) < DFU_TIMEOUT_US) &&
           (dfu_.state != ST_FINISHED))
    {
        shtp_service(dfu_.pShtp);
        now_us = dfu_.pHal->getTimeUs(dfu_.pHal);
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

