/*
 * Copyright 2021 CEVA, Inc.
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
 * SPI-based HALs for SH2 and DFU.
 */

#include "sh2_hal_init.h"

#include "sh2_hal.h"
#include "sh2_err.h"

#include <stdint.h>
#include <stdbool.h>
#include <string.h>

#include "stm32f4xx_hal.h"
#include "stm32f4xx_hal_tim.h"
#include "stm32f4xx_hal_spi.h"

#define CLKSEL0_PORT GPIOA
#define CLKSEL0_PIN  GPIO_PIN_8

#define RSTN_PORT GPIOB
#define RSTN_PIN  GPIO_PIN_4

#define BOOTN_PORT GPIOB
#define BOOTN_PIN  GPIO_PIN_5

#define PS0_WAKEN_PORT GPIOB
#define PS0_WAKEN_PIN  GPIO_PIN_10

#define PS1_PORT GPIOB
#define PS1_PIN  GPIO_PIN_0

#define INTN_PORT GPIOA
#define INTN_PIN GPIO_PIN_10

// CSN pin: B6
#define CSN_PORT GPIOB
#define CSN_PIN  GPIO_PIN_6

// Keep reset asserted this long.
// (Some targets have a long RC decay on reset.)
#define RESET_DELAY_US (10000)

// Wait up to this long to see first interrupt from SH
#define START_DELAY_US (2000000)

// Wait this long before assuming bootloader is ready
#define DFU_BOOT_DELAY_US (50000)

#define DFU_CS_TIMING_US (20)
#define DFU_BYTE_TIMING_US (28)
#define DFU_CS_DEASSERT_DELAY_RX_US (0)
#define DFU_CS_DEASSERT_DELAY_TX_US (5000)

// How many bytes to read when reading the length field
#define READ_LEN (4)

// ------------------------------------------------------------------------
// Private types

typedef enum SpiState_e
{
    SPI_INIT,
    SPI_DUMMY,
    SPI_DFU,
    SPI_IDLE,
    SPI_RD_HDR,
    SPI_RD_BODY,
    SPI_WRITE
} SpiState_t;

// ------------------------------------------------------------------------
// Private data

// Dummy transmit data for SPI reads
static const uint8_t txZeros[SH2_HAL_MAX_TRANSFER_IN] = {0};

// Timer handle
static TIM_HandleTypeDef tim2;

// SPI Peripheral, SPI1
static SPI_HandleTypeDef spi;

// SPI Bus access state machine state
static SpiState_t spiState = SPI_INIT;

// Timestamp
static volatile uint32_t rxTimestamp_us;

// true from time SH is put in reset until first INTN indication
static volatile bool inReset;

// set true when INTN is observed, until RX operation starts
static volatile bool rxReady;

// Receive support
static uint8_t rxBuf[SH2_HAL_MAX_TRANSFER_IN];
static volatile uint32_t rxBufLen;
static volatile bool rxDataReady;

// Transmit support
static uint8_t txBuf[SH2_HAL_MAX_TRANSFER_OUT];
static uint32_t txBufLen;

// Instances of the SPI HAL for SH2 and DFU
static sh2_Hal_t sh2Hal;
static sh2_Hal_t dfuHal;

static bool isOpen = false;

// ------------------------------------------------------------------------
// Private methods

static void bootn(bool state)
{
    HAL_GPIO_WritePin(BOOTN_PORT, BOOTN_PIN, 
                      state ? GPIO_PIN_SET : GPIO_PIN_RESET);
}

static void rstn(bool state)
{
    HAL_GPIO_WritePin(RSTN_PORT, RSTN_PIN, 
                      state ? GPIO_PIN_SET : GPIO_PIN_RESET);
}

static void ps0_waken(bool state)
{
    HAL_GPIO_WritePin(PS0_WAKEN_PORT, PS0_WAKEN_PIN, 
                      state ? GPIO_PIN_SET : GPIO_PIN_RESET);
}

static void ps1(bool state)
{
    HAL_GPIO_WritePin(PS1_PORT, PS1_PIN, 
                      state ? GPIO_PIN_SET : GPIO_PIN_RESET);
}

static void csn(bool state)
{
    HAL_GPIO_WritePin(CSN_PORT, CSN_PIN, 
                      state ? GPIO_PIN_SET : GPIO_PIN_RESET);
}

static uint32_t timeNowUs(void)
{
    return __HAL_TIM_GET_COUNTER(&tim2);
}

static void hal_init_timer(void)
{
    __HAL_RCC_TIM2_CLK_ENABLE();
    
    // Prescale to get 1 count per uS
    uint32_t prescaler = (uint32_t)((HAL_RCC_GetPCLK2Freq() / 1000000) - 1);

    tim2.Instance = TIM2;
    tim2.Init.Period = 0xFFFFFFFF;
    tim2.Init.Prescaler = prescaler;
    tim2.Init.ClockDivision = 0;
    tim2.Init.CounterMode = TIM_COUNTERMODE_UP;

    HAL_TIM_Base_Init(&tim2);
    HAL_TIM_Base_Start(&tim2);
}

static void hal_init_gpio(void)
{
    GPIO_InitTypeDef GPIO_InitStruct;

    __HAL_RCC_GPIOA_CLK_ENABLE();
    __HAL_RCC_GPIOB_CLK_ENABLE();
    
    /* Configure PS0_WAKEN */
    HAL_GPIO_WritePin(PS0_WAKEN_PORT, PS0_WAKEN_PIN, GPIO_PIN_RESET);
    GPIO_InitStruct.Pin = PS0_WAKEN_PIN;
    GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
    HAL_GPIO_Init(PS0_WAKEN_PORT, &GPIO_InitStruct);

    /* Configure PS1 */
    HAL_GPIO_WritePin(PS1_PORT, PS1_PIN, GPIO_PIN_SET);
    GPIO_InitStruct.Pin = PS1_PIN;
    GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
    HAL_GPIO_Init(PS1_PORT, &GPIO_InitStruct);

    /* Configure RSTN*/
    HAL_GPIO_WritePin(RSTN_PORT, RSTN_PIN, GPIO_PIN_RESET);
    GPIO_InitStruct.Pin = RSTN_PIN;
    GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
    HAL_GPIO_Init(RSTN_PORT, &GPIO_InitStruct);

    /* Configure BOOTN */
    HAL_GPIO_WritePin(BOOTN_PORT, BOOTN_PIN, GPIO_PIN_RESET);
    GPIO_InitStruct.Pin = BOOTN_PIN;
    GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
    HAL_GPIO_Init(BOOTN_PORT, &GPIO_InitStruct);

    /*Configure GPIO pin : INTN */
    GPIO_InitStruct.Pin = INTN_PIN;
    GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
    GPIO_InitStruct.Pull = GPIO_PULLUP;
    HAL_GPIO_Init(INTN_PORT, &GPIO_InitStruct);

    /*Configure GPIO pin : CLKSEL0_PIN */
    /* Set CLKSEL0 to 0 : FSP200 should use crystal for timing. */
    HAL_GPIO_WritePin(CLKSEL0_PORT, CLKSEL0_PIN, GPIO_PIN_RESET);
    GPIO_InitStruct.Pin = CLKSEL0_PIN;
    GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
    HAL_GPIO_Init(CLKSEL0_PORT, &GPIO_InitStruct);

    /* EXTI interrupt init*/
    HAL_NVIC_SetPriority(EXTI15_10_IRQn, 5, 0);
}

static void spiDummyOp(void)
{
    // We need to establish SCLK in proper initial state.
    // Do one SPI operation with reset asserted and no CS asserted to get clock sorted.
    uint8_t dummyTx[1];
    uint8_t dummyRx[1];
    
    memset(dummyTx, 0xAA, sizeof(dummyTx));

    HAL_SPI_TransmitReceive(&spi, dummyTx, dummyRx, sizeof(dummyTx), 2);
}

static void hal_init_spi(bool dfu)
{
    GPIO_InitTypeDef GPIO_InitStruct;

    // SPI1 GPIO Configuration    
    // PA5 : SPI1_SCK
    // PA6 : SPI1_MISO
    // PA7 : SPI1_MOSI 
    GPIO_InitStruct.Pin = GPIO_PIN_5|GPIO_PIN_6|GPIO_PIN_7;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
    GPIO_InitStruct.Alternate = GPIO_AF5_SPI1;
    HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

    /* Configure CSN */
    HAL_GPIO_WritePin(CSN_PORT, CSN_PIN, GPIO_PIN_RESET);
    GPIO_InitStruct.Pin = CSN_PIN;
    GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
    HAL_GPIO_Init(CSN_PORT, &GPIO_InitStruct);

    // Peripheral clock enable
    __HAL_RCC_SPI1_CLK_ENABLE();

    // Init SPI Peripheral
    spi.Instance = SPI1;
    spi.Init.Mode = SPI_MODE_MASTER;
    spi.Init.Direction = SPI_DIRECTION_2LINES;
    spi.Init.DataSize = SPI_DATASIZE_8BIT;
    if (dfu)
    {
        // Differences for DFU mode.
        spi.Init.CLKPolarity = SPI_POLARITY_LOW;
        spi.Init.CLKPhase = SPI_PHASE_1EDGE;
        spi.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_128; // 650 kHz
    }
    else
    {
        spi.Init.CLKPolarity = SPI_POLARITY_HIGH;
        spi.Init.CLKPhase = SPI_PHASE_2EDGE;
        spi.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_64; // 1.3MHz
    }
    spi.Init.NSS = SPI_NSS_SOFT;
    spi.Init.FirstBit = SPI_FIRSTBIT_MSB;
    spi.Init.TIMode = SPI_TIMODE_DISABLE;
    spi.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
    spi.Init.CRCPolynomial = 10;

    HAL_SPI_Init(&spi);
    
    // Set priority for IRQ and enable
    HAL_NVIC_SetPriority(SPI1_IRQn, 5, 0);
}

static void hal_init_hw(bool dfu)
{
    hal_init_timer();
    hal_init_gpio();
    hal_init_spi(dfu);
}

static void enableInts(void)
{
    HAL_NVIC_EnableIRQ(EXTI15_10_IRQn);
    HAL_NVIC_EnableIRQ(SPI1_IRQn);
}

static void disableInts()
{
    HAL_NVIC_DisableIRQ(SPI1_IRQn);
    HAL_NVIC_DisableIRQ(EXTI15_10_IRQn);
}

// Attempt to start a SPI operation.
// This can be done from interrupt context or with interrupts disabled.
// If SPI periph is not in use and there is data to send or receive,
// this will start a SPI operation.
static void spiActivate(void)
{
    if ((spiState == SPI_IDLE) && (rxBufLen == 0))
    {
        if (rxReady)
        {
            // reset flag that was set with INTN
            rxReady = false;
            
            // assert CSN
            csn(false);

            if (txBufLen > 0)
            {
                spiState = SPI_WRITE;
                
                // Start operation to write (and, incidentally, read)
                HAL_SPI_TransmitReceive_IT(&spi, txBuf, rxBuf, txBufLen);

                // Deassert Wake
                ps0_waken(true);
            }
            else
            {
                spiState = SPI_RD_HDR;
                
                // Start SPI operation to read header (writing zeros)
                HAL_SPI_TransmitReceive_IT(&spi, (uint8_t *)txZeros, rxBuf, READ_LEN);
            }
        }
    }
}

// Handle the end of a SPI operation.
// This can be done from interrupt context or with interrupts disabled.
// Depending on spiState, it may start a follow-up operation or transition
// to idle.  In the latter case, it will call spiActivate
static void spiCompleted(void)
{
    // Get length of payload available
    uint16_t rxLen = (rxBuf[0] + (rxBuf[1] << 8)) & ~0x8000;
        
    // Truncate that to max len we can read
    if (rxLen > sizeof(rxBuf))
    {
        rxLen = sizeof(rxBuf);
    }

    if (spiState == SPI_DUMMY)
    {
        // SPI Dummy operation completed, transition now to idle
        spiState = SPI_IDLE;
    }
    else if (spiState == SPI_RD_HDR)
    {
        // We read a header

        if (rxLen > READ_LEN) {
            // There is more to read

            // Transition to RD_BODY state
            spiState = SPI_RD_BODY;
        
            // Start a read operation for the remaining length.  (We already read the first READ_LEN bytes.)
            HAL_SPI_TransmitReceive_IT(&spi, (uint8_t *)txZeros, rxBuf+READ_LEN, rxLen-READ_LEN);
        }
        else
        {
            // No SHTP payload was received, this operation is done
            csn(true);            // deassert CSN
            rxBufLen = 0;         // no rx data available
            spiState = SPI_IDLE;  // back to idle state
            spiActivate();        // activate next operation, if any.
        }
    }
    else if (spiState == SPI_RD_BODY)
    {
        // We completed the read or write of a payload
        // deassert CSN.
        csn(true);

        // Check len of data read and set rxBufLen
        rxBufLen = rxLen;

        // transition back to idle state
        spiState = SPI_IDLE;

        // Activate the next operation, if any.
        spiActivate();
    }
    else if (spiState == SPI_WRITE)
    {
        // We completed the read or write of a payload
        // deassert CSN.
        csn(true);

        // Since operation was a write, transaction was for txBufLen bytes.  So received
        // data len is, at a maximum, txBufLen.
        rxBufLen = (txBufLen < rxLen) ? txBufLen : rxLen;

        // Tx buffer is empty now.
        txBufLen = 0;
        
        // transition back to idle state
        spiState = SPI_IDLE;

        // Activate the next operation, if any.
        spiActivate();
    }
}


// Interrupt handlers and SPI operation callbacks

void HAL_GPIO_EXTI_Callback(uint16_t n)
{
    rxTimestamp_us = timeNowUs();
    
    inReset = false;
    rxReady = true;

    // Start read, if possible
    spiActivate();
}

// Handle INTN Interrupt through STM32 HAL
// (It, in turn, calls HAL_GPIO_EXTI_Callback, above)
void EXTI15_10_IRQHandler(void)
{
    HAL_GPIO_EXTI_IRQHandler(GPIO_PIN_10);
}

void HAL_SPI_TxRxCpltCallback(SPI_HandleTypeDef * hspi)
{
    if (isOpen)
    {
        spiCompleted();
    }
}

void HAL_SPI_ErrorCallback(SPI_HandleTypeDef * hspi)
{
    // Shouldn't happen
    while (1);
}

// Handle SPI1 Global Interrupt
void SPI1_IRQHandler(void)
{
    HAL_SPI_IRQHandler(&spi);
}

void delayUs(uint32_t delay)
{
    volatile uint32_t now = timeNowUs();
    uint32_t start = now;
    while ((now - start) < delay) {
        now = timeNowUs();
    }
}

void resetDelayUs(uint32_t delay)
{
    volatile uint32_t now = timeNowUs();
    uint32_t start = now;
    while (((now - start) < delay) && (inReset))
    {
        now = timeNowUs();
    }
}

// ------------------------------------------------------------------------
// SH2 SPI Hal Methods

static int sh2_spi_hal_open(sh2_Hal_t *self)
{
    int retval = SH2_OK;

    if (isOpen)
    {
        // Can't open if another instance is already open
        return SH2_ERR;
    }

    isOpen = true;

    // Init hardware (false -> non-DFU config)
    hal_init_hw(false);

    // Hold in reset
    rstn(false);

    // deassert CSN
    csn(true);

    // Clear rx, tx buffers
    rxBufLen = 0;
    txBufLen = 0;
    rxDataReady = false;
    rxReady = false;
    
    inReset = true;  // will change back to false when INTN serviced

    // Do dummy SPI operation
    // (First SPI op after reconfig has bad initial state of signals
    // so this is a throwaway operation.  Afterward, all is well.)
    spiState = SPI_DUMMY;
    spiDummyOp();
    spiState = SPI_IDLE;
    
    // Delay for RESET_DELAY_US to ensure reset takes effect
    delayUs(RESET_DELAY_US);
    
    // To boot in SHTP-SPI mode, must have PS1=1, PS0=1.
    // PS1 is set via jumper.
    // PS0 will be 1 PS1 jumper is 1 AND PS0_WAKEN sig is 1.
    // So we set PS0_WAKEN signal to 1
    ps0_waken(true);
    ps1(true);

    // Deassert reset, boot in non-DFU mode
    bootn(true);
    rstn(true);

    // enable interrupts
    enableInts();

    // Wait for INTN to be asserted
    resetDelayUs(START_DELAY_US);

    return retval;
}

static void sh2_spi_hal_close(sh2_Hal_t *self)
{
    // Disable interrupts
    disableInts();
    
    // Set state machine to INIT state
    spiState = SPI_INIT;
    
    // Hold sensor hub in reset
    rstn(false);
    
    // deassert CSN
    csn(true);

    // Deinit SPI peripheral
    HAL_SPI_DeInit(&spi);
    
    // Deinit timer
    __HAL_TIM_DISABLE(&tim2);

    // No longer open
    isOpen = false;
}

static int sh2_spi_hal_read(sh2_Hal_t *self, uint8_t *pBuffer, unsigned len, uint32_t *t)
{
    int retval = 0;

    // If there is received data available...
    if (rxBufLen > 0)
    {
        // And if the data will fit in this buffer...
        if (len >= rxBufLen)
        {
            // Copy data to the client buffer
            memcpy(pBuffer, rxBuf, rxBufLen);
            retval = rxBufLen;

            // Set timestamp of that data
            *t = rxTimestamp_us;

            // Clear rxBuf so we can receive again
            rxBufLen = 0;
        }
        else
        {
            // Discard what was read and return error because buffer was too small.
            retval = SH2_ERR_BAD_PARAM;
            rxBufLen = 0;
        }
        
        // Now that rxBuf is empty, activate SPI processing to send any
        // potential write that was blocked.
        disableInts();
        spiActivate();
        enableInts();
    }

    return retval;
}

static int sh2_spi_hal_write(sh2_Hal_t *self, uint8_t *pBuffer, unsigned len)
{
    int retval = SH2_OK;

    // Validate parameters
    if ((self == 0) || (len > sizeof(txBuf)) ||
        ((len > 0) && (pBuffer == 0)))
    {
        return SH2_ERR_BAD_PARAM;
    }

    // If tx buffer is not empty, return 0
    if (txBufLen != 0)
    {
        return 0;
    }
    
    // Copy data to tx buffer
    memcpy(txBuf, pBuffer, len);
    txBufLen = len;
    retval = len;

    // disable SH2 interrupts for a moment
    disableInts();

    // Assert Wake
    ps0_waken(false);

    // re-enable SH2 interrupts.
    enableInts();

    return retval;
}

static uint32_t sh2_spi_hal_getTimeUs(sh2_Hal_t *self)
{
    return timeNowUs();
}

// ------------------------------------------------------------------------
// DFU SPI Hal Methods

static int dfu_spi_hal_open(sh2_Hal_t *self)
{
    int retval = SH2_OK;

    if (isOpen)
    {
        // Can't open if another instance is already open
        return SH2_ERR;
    }

    isOpen = true;

    // Init hardware (true -> DFU config)
    hal_init_hw(true);

    // Hold in reset, for DFU
    rstn(false);

    // deassert CSN
    csn(true);

    // Clear rx, tx buffers
    rxBufLen = 0;
    txBufLen = 0;
    rxDataReady = false;
    rxReady = false;

    inReset = true;  // will change back to false when INTN serviced

    // Do dummy SPI operation
    // (First SPI op after reconfig has bad initial state of signals
    // so this is a throwaway operation.  Afterward, all is well.)
    spiState = SPI_DUMMY;
    spiDummyOp();
    spiState = SPI_DFU;
    
    // Delay for RESET_DELAY_US to ensure reset takes effect
    delayUs(RESET_DELAY_US);
    
    // To boot in SHTP-SPI mode, must have PS1=1, PS0=1.
    // PS1 is set via jumper.
    // PS0 will be 1 PS1 jumper is 1 AND PS0_WAKEN sig is 1.
    // So we set PS0_WAKEN signal to 1
    ps0_waken(true);
    ps1(true);

    // Deassert reset, boot in DFU mode
    bootn(false);
    rstn(true);

    // enable interrupts
    enableInts();

    // Wait for bootloader to be ready
    delayUs(DFU_BOOT_DELAY_US);

    return retval;
}

static void dfu_spi_hal_close(sh2_Hal_t *self)
{
    // Disable interrupts
    disableInts();
    
    // Set state machine to INIT state
    spiState = SPI_INIT;
    
    // Hold sensor hub in reset
    rstn(false);
    
    // deassert CSN
    csn(true);

    // Deinit SPI peripheral
    HAL_SPI_DeInit(&spi);
    
    // Deinit timer
    __HAL_TIM_DISABLE(&tim2);
    
    // No longer open
    isOpen = false;
}

static int dfu_spi_hal_read(sh2_Hal_t *self, uint8_t *pBuffer, unsigned len, uint32_t *t)
{
    int retval = 0;
    int rc = 0;

    // DFU HAL always returns 0 in timestamp param
    *t = 0;

    // assert CSN, delay to meet bootloader timing requirements
    csn(false);
    delayUs(DFU_CS_TIMING_US);
    
    // perform transfer, one byte at a time
    for (int n = 0; n < len; n++)
    {
        rc = HAL_SPI_Receive(&spi, pBuffer+n, 1, 2);
        delayUs(DFU_BYTE_TIMING_US);
        if (rc != 0)
        {
            break;
        }
    }

    // Set return status
    if (rc == 0)
    {
        retval = len;
    }
    else
    {
        retval = SH2_ERR_IO;
    }
    
    // deassert CSN, delay to meet bootloader timing requirements
    csn(true);
    delayUs(DFU_CS_DEASSERT_DELAY_RX_US);

    return retval;
}

static int dfu_spi_hal_write(sh2_Hal_t *self, uint8_t *pBuffer, unsigned len)
{
    int retval = SH2_OK;
    int rc;
    
    rc = 0;

    // assert CSN, delay
    csn(false);
    delayUs(DFU_CS_TIMING_US);
    
    // perform bytewise writes
    for (int n = 0; n < len; n++)
    {
        rc = HAL_SPI_Transmit(&spi, pBuffer+n, 1, 2);
        delayUs(DFU_BYTE_TIMING_US);
        if (rc != 0)
        {
            break;
        }
    }
    
    // set up return status
    if (rc != 0)
    {
        retval = SH2_ERR_IO;
    }
    else
    {
        retval = len;
    }
    
    // deassert CSN, delay
    csn(true);
    delayUs(DFU_CS_DEASSERT_DELAY_TX_US);

    return retval;
}

static uint32_t dfu_spi_hal_getTimeUs(sh2_Hal_t *self)
{
    return timeNowUs();
}

// ------------------------------------------------------------------------
// Public methods

sh2_Hal_t *sh2_hal_init(void)
{
    // Set up the HAL reference object for the client
    sh2Hal.open = sh2_spi_hal_open;
    sh2Hal.close = sh2_spi_hal_close;
    sh2Hal.read = sh2_spi_hal_read;
    sh2Hal.write = sh2_spi_hal_write;
    sh2Hal.getTimeUs = sh2_spi_hal_getTimeUs;

    return &sh2Hal;
}

sh2_Hal_t *dfu_hal_init(void)
{
    // Set up the HAL reference object for the client
    dfuHal.open = dfu_spi_hal_open;
    dfuHal.close = dfu_spi_hal_close;
    dfuHal.read = dfu_spi_hal_read;
    dfuHal.write = dfu_spi_hal_write;
    dfuHal.getTimeUs = dfu_spi_hal_getTimeUs;

    return &dfuHal;
}



    
