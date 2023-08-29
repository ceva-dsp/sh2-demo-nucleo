/*
 * Copyright 2017-2021 CEVA, Inc.
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
 * SHTP UART-based HAL for SH2.
 */

#include "uart_hal.h"

#include "sh2_hal_init.h"
#include "sh2_hal.h"
#include "sh2_err.h"

#include <stdint.h>
#include <stdbool.h>
#include <string.h>

#include "stm32f4xx_hal.h"
#include "stm32f4xx_hal_tim.h"
#include "usart.h"
#include "dbg.h"

// For FSP200 UART-SHTP Autobaud, uncomment and set desired baud rate in SH2_BPS
// #define USE_FSP200_SHTP_AUTOBAUD

#ifdef USE_FSP200_SHTP_AUTOBAUD
#define SH2_BPS (115200)             // 9600 - 2.3M for FSP200 SHTP AUTOBAUD
#else
#define SH2_BPS (3000000)            // always 3Mbps with no autobaud
#endif

#define BNO_DFU_BPS (115200)         // 115200 bps for UART-DFU

// When defined, PS0_WAKEN is always asserted.  Otherwise PS0_WAKEN
// is deasserted when host is not talking to sensor hub.
// #define ALWAYS_WAKE

#define TX_PORT GPIOA
#define TX_PIN  GPIO_PIN_9
#define RX_PORT GPIOB
#define RX_PIN  GPIO_PIN_3

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

#define H_MOSI_ABN_PORT GPIOA
#define H_MOSI_ABN_PIN GPIO_PIN_7

#define RFC1662_FLAG (0x7e)
#define RFC1662_ESCAPE (0x7d)

#define PROTOCOL_CONTROL (0)
#define PROTOCOL_SHTP (1)

// Time between transmitted characters
// With autobaud, adjust TX interval to include 10-bit char transmit time.
// Explanation: 1000000.0 : 1 million microseconds per second
//              10.0      : 10 bit times per character
#define TX_CHAR_US ((1000000.0 * 10.0/SH2_BPS))  // time for one char to clock out
#define TX_INTERVAL_US ((TX_CHAR_US >= 100.0) ? (unsigned)(TX_CHAR_US*1.2) : 100)

// Keep reset asserted this long.
// (Some targets have a long RC decay on reset.)
#define RESET_DELAY_US (10000)

// Wait up to this long to see first interrupt from SH
#define START_DELAY_US (4000000)

// Wait this long before assuming bootloader is ready
#define DFU_BOOT_DELAY_US (3000000)

// ------------------------------------------------------------------------
// Private types

typedef enum {
    OUTSIDE_FRAME,   // Waiting for start of frame
    INSIDE_FRAME,    // Inside frame until end of frame
    ESCAPED,         // Inside frame, after escape char
} RxState_t;

typedef enum {
    TX_IDLE,
    TX_SENDING_BSQ,
    TX_SENDING_FRAME,
    TX_SENDING_DFU,
} TxState_t;

// ------------------------------------------------------------------------
// Private data

static bool isOpen = false;

// DMA stream for USART1 Rx.
DMA_HandleTypeDef hdma_usart1_rx;

// USART1 handle
UART_HandleTypeDef huart1;

// Timer handle
TIM_HandleTypeDef tim2;

// receive support
#define UART_HAL_DMA_SIZE (64)            // must be a power of 2!
static uint8_t rxBuffer[UART_HAL_DMA_SIZE]; // receives UART data via DMA (must be a power of 2)
static uint32_t rxIndex = 0;               // next index to read
static uint32_t rxTimestamp_uS;            // timestamp of INTN event

// RFC 1622 frame decode area
static uint8_t rxFrame[SH2_HAL_MAX_TRANSFER_IN];
static uint32_t rxFrameLen;
static bool rxFrameReady;
static RxState_t rxState;

// Transmit support
static uint32_t lastTxTime = 0;        // uS timestamp of last tx char (for 100uS intervals)
static uint16_t lastBsn = 0;           // value of last valid BSN
static volatile TxState_t txState = TX_IDLE;    // transmit state: IDLE, SENDING_BSQ, SENDING_FRAME.
static uint8_t txFrame[2*SH2_HAL_MAX_TRANSFER_OUT+2]; // frame to be sent. (RFC encode on insert)
static uint32_t txFrameLen;            // len of frame to be sent (after RFC encode).
// buffer status query message (RFC encoded)
static const uint8_t bsq[3] = { RFC1662_FLAG, PROTOCOL_CONTROL, RFC1662_FLAG };
static uint32_t txIndex = 0;           // index of next byte to be sent (of txFrame or bsq)

// True between asserting reset and seeing first INTN assertion
static volatile bool inReset;

#ifdef USE_FSP200_SHTP_AUTOBAUD
// FSP200 autobaud sequence
#define FSP200_AUTOBAUD_CHAR (0x55)
static const uint8_t fsp200_autobaud[] = { FSP200_AUTOBAUD_CHAR };
#endif

// ------------------------------------------------------------------------
// Private methods

static void disableInts(void)
{
    HAL_NVIC_DisableIRQ(EXTI15_10_IRQn);
    HAL_NVIC_DisableIRQ(DMA2_Stream2_IRQn);
    HAL_NVIC_DisableIRQ(USART1_IRQn);
}

static void enableInts(void)
{
    HAL_NVIC_EnableIRQ(USART1_IRQn);
    HAL_NVIC_EnableIRQ(DMA2_Stream2_IRQn);
    HAL_NVIC_EnableIRQ(EXTI15_10_IRQn);
}

static void hal_init_gpio(void)
{
    GPIO_InitTypeDef GPIO_InitStruct;

    __HAL_RCC_GPIOA_CLK_ENABLE();
    __HAL_RCC_GPIOB_CLK_ENABLE();
    
    /* Configure PS0_WAKEN */
    GPIO_InitStruct.Pin = PS0_WAKEN_PIN;
    GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
    HAL_GPIO_Init(PS0_WAKEN_PORT, &GPIO_InitStruct);
    HAL_GPIO_WritePin(PS0_WAKEN_PORT, PS0_WAKEN_PIN, GPIO_PIN_RESET);

    /* Configure PS1 */
    GPIO_InitStruct.Pin = PS1_PIN;
    GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
    HAL_GPIO_Init(PS1_PORT, &GPIO_InitStruct);
    HAL_GPIO_WritePin(PS1_PORT, PS1_PIN, GPIO_PIN_SET);

    /* Configure RSTN*/
    GPIO_InitStruct.Pin = RSTN_PIN;
    GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
    HAL_GPIO_Init(RSTN_PORT, &GPIO_InitStruct);
    HAL_GPIO_WritePin(RSTN_PORT, RSTN_PIN, GPIO_PIN_RESET);

    /* Configure BOOTN */
    GPIO_InitStruct.Pin = BOOTN_PIN;
    GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
    HAL_GPIO_Init(BOOTN_PORT, &GPIO_InitStruct);
    HAL_GPIO_WritePin(BOOTN_PORT, BOOTN_PIN, GPIO_PIN_RESET);

    /*Configure GPIO pin : INTN */
    GPIO_InitStruct.Pin = INTN_PIN;
    GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
    GPIO_InitStruct.Pull = GPIO_PULLUP;
    HAL_GPIO_Init(INTN_PORT, &GPIO_InitStruct);

    /*Configure GPIO pin : CLKSEL0_PIN */
    /* Set CLKSEL0 to 0 : FSP200 should use crystal for timing. */
    GPIO_InitStruct.Pin = CLKSEL0_PIN;
    GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
    HAL_GPIO_Init(CLKSEL0_PORT, &GPIO_InitStruct);
    HAL_GPIO_WritePin(CLKSEL0_PORT, CLKSEL0_PIN, GPIO_PIN_RESET);

    /* Configure H_MOSI_ABN pin */
#ifdef USE_FSP200_SHTP_AUTOBAUD
    GPIO_InitStruct.Pin = H_MOSI_ABN_PIN;
    GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
    HAL_GPIO_Init(H_MOSI_ABN_PORT, &GPIO_InitStruct);
    HAL_GPIO_WritePin(H_MOSI_ABN_PORT, H_MOSI_ABN_PIN, GPIO_PIN_RESET);
#else
    GPIO_InitStruct.Pin = H_MOSI_ABN_PIN;
    GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
    GPIO_InitStruct.Pull = GPIO_PULLUP;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
    HAL_GPIO_Init(H_MOSI_ABN_PORT, &GPIO_InitStruct);
#endif

    /* EXTI interrupt init*/
    HAL_NVIC_SetPriority(EXTI15_10_IRQn, 5, 0);
}

static void hal_init_dma(void)
{
    /* DMA controller clock enable */
    __HAL_RCC_DMA2_CLK_ENABLE();

    /* DMA interrupt init */
    /* DMA2_Stream2_IRQn interrupt configuration */
    HAL_NVIC_SetPriority(DMA2_Stream2_IRQn, 5, 0);
}

static void hal_init_usart(void)
{
    GPIO_InitTypeDef GPIO_InitStruct;
    
    // Enable USART1
    __HAL_RCC_USART1_CLK_ENABLE();

    // Configure GPIO Pins for use with USART1
    GPIO_InitStruct.Pin = TX_PIN;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull = GPIO_PULLUP;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_MEDIUM;
    GPIO_InitStruct.Alternate = GPIO_AF7_USART1;
    HAL_GPIO_Init(TX_PORT, &GPIO_InitStruct);
        
    GPIO_InitStruct.Pin = RX_PIN;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull = GPIO_PULLUP;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_MEDIUM;
    GPIO_InitStruct.Alternate = GPIO_AF7_USART1;
    HAL_GPIO_Init(RX_PORT, &GPIO_InitStruct);

    // Set Priority for USART1_IRQ
    HAL_NVIC_SetPriority(USART1_IRQn, 5, 0);
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

static void hal_init_hw(void)
{
    hal_init_timer();
    hal_init_gpio();
    hal_init_dma();
    hal_init_usart();
}

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

static void onTxCpltDfu(UART_HandleTypeDef *huart)
{
    if (txState == TX_SENDING_DFU)
    {
        txState = TX_IDLE;
    }
}

static int setupUsart(uint32_t baudrate)
{
    hdma_usart1_rx.Instance = DMA2_Stream2;
    hdma_usart1_rx.Init.Channel = DMA_CHANNEL_4;
    hdma_usart1_rx.Init.Direction = DMA_PERIPH_TO_MEMORY;
    hdma_usart1_rx.Init.PeriphInc = DMA_PINC_DISABLE;
    hdma_usart1_rx.Init.MemInc = DMA_MINC_ENABLE;
    hdma_usart1_rx.Init.PeriphDataAlignment = DMA_PDATAALIGN_BYTE;
    hdma_usart1_rx.Init.MemDataAlignment = DMA_MDATAALIGN_BYTE;
    hdma_usart1_rx.Init.Mode = DMA_CIRCULAR;
    hdma_usart1_rx.Init.Priority = DMA_PRIORITY_LOW;
    hdma_usart1_rx.Init.FIFOMode = DMA_FIFOMODE_DISABLE;
    if (HAL_DMA_Init(&hdma_usart1_rx) != HAL_OK)
    {
        return SH2_ERR_IO;
    }

    // Resync rxIndex with DMA process
    rxIndex = 0;

    __HAL_LINKDMA(&huart1,hdmarx,hdma_usart1_rx);

    huart1.Instance = USART1;
    huart1.Init.BaudRate = baudrate;
    huart1.Init.WordLength = UART_WORDLENGTH_8B;
    huart1.Init.StopBits = UART_STOPBITS_1;
    huart1.Init.Parity = UART_PARITY_NONE;
    huart1.Init.Mode = UART_MODE_TX_RX;
    huart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
    huart1.Init.OverSampling = UART_OVERSAMPLING_16;
    if (HAL_UART_Init(&huart1) != HAL_OK)
    {
        return SH2_ERR_IO;
    }
  
    return SH2_OK;
}

// reset RFC 1662 decode state machine
static void rfc1662_reset(void)
{
    rxFrameLen = 0;
    rxFrameReady = false;
    rxState = OUTSIDE_FRAME;
}

static void rxResetFrame(void)
{
    rxFrameLen = 0;
    rxFrameReady = false;
}

static void rxAddToFrame(uint8_t c)
{
    // Add the character to the frame in progress
    if (rxFrameLen < sizeof(rxFrame)) {
        rxFrame[rxFrameLen] = c;
        rxFrameLen++;
    }
    else {
        // overflowed the buffer!  Don't store data
        rxFrameLen++;
    }
}

// Process a received byte through RFC 1662 framing
static void rfc1662_rx(uint8_t c)
{
    // Use state machine to build up chars into frames for delivery.
    switch (rxState) {
        case OUTSIDE_FRAME:
            // Look for start of frame
            if (c == RFC1662_FLAG) {
                // Init frame in progress
                rxResetFrame();
                rxState = INSIDE_FRAME;
            }
            break;
        case INSIDE_FRAME:
            // Look for end of frame
            if (c == RFC1662_FLAG) {
                if (rxFrameLen > 0) {
                    // Frame is done
                    rxFrameReady = true;
                    rxState = OUTSIDE_FRAME;
                }
                else {
                    // Treat second consec flag as another start flag.
                    rxState = INSIDE_FRAME;
                }
            }
            else if (c == RFC1662_ESCAPE) {
                // Go to escaped state so next char can be a flag or escape
                rxState = ESCAPED;
            }
            else {
                rxAddToFrame(c);
            }
            break;
        case ESCAPED:
            rxAddToFrame(c ^ 0x20);
            rxState = INSIDE_FRAME;
            break;
        default:
            // Bad state.  Recover by resetting to outside frame state
            rxState = OUTSIDE_FRAME;
            break;
    }
}

static uint32_t timeNowUs(void)
{
    return __HAL_TIM_GET_COUNTER(&tim2);
}

static void delay_us(uint32_t t)
{
    uint32_t start, now;

    now = timeNowUs();
    start = now;
    while ((now - start) < t) {
        now = timeNowUs();
    }
}

static void reset_delay_us(uint32_t t)
{
    uint32_t start, now;

    now = timeNowUs();
    start = now;
    while (((now - start) < t) && inReset) {
        now = timeNowUs();
    }
}

// Called from write and from read to move transmit processing forward.
static void txStep(void)
{
    // Inactive, return quickly
    if (txState == TX_IDLE) {
        return;
    }

    // Not enough time elapsed since last transmit, skip this
    uint32_t now = timeNowUs();
    if ((now - lastTxTime) < TX_INTERVAL_US) {
        return;
    }

    // We have stuff to do and it's time to do it.
    if (txState == TX_SENDING_BSQ) {
        // assert wake
        ps0_waken(false);
        
        if ((txIndex == 0) && (lastBsn >= txFrameLen)) {
            // Switch to sending frame
            txState = TX_SENDING_FRAME;
        }
        else {
            // Send one byte of BSQ
            HAL_UART_Transmit_IT(&huart1, (uint8_t *)&bsq[txIndex], 1);
            txIndex = (txIndex + 1) % sizeof(bsq);
            lastTxTime = now;
        }
    }
    if (txState == TX_SENDING_FRAME) {
        if (txIndex >= txFrameLen) {
            // Frame transmit is done
            txIndex = 0;
            txState = TX_IDLE;
            lastBsn = 0;
             
#ifdef ALWAYS_WAKE
            ps0_waken(false);
#else
            // deassert wake
            ps0_waken(true);
#endif
        }
        else {
            // Send one byte
            HAL_UART_Transmit_IT(&huart1, &txFrame[txIndex], 1);
            txIndex += 1;
            lastTxTime = now;
        }
    }
}

static void txStore(uint8_t c)
{
    if (txFrameLen < sizeof(txFrame)) {
        txFrame[txFrameLen] = c;
    }
    txFrameLen++;
}

// Form transmit buffer with flags, PROTOCOL_SHTP id, RFC 1662 encoding.
// Result ends up in txFrame, txFrameLen.
// If the process overflows, txFrameLen will be > sizeof(txFrame)
// but the extra bytes will not be written beyond txFrame[]
static void txEncode(uint8_t *pSrc, uint32_t len)
{
    uint32_t i = 0;
    txFrameLen = 0;

    // Start of frame
    txStore(RFC1662_FLAG);

    // Protocol ID
    txStore(PROTOCOL_SHTP);
    
    // Frame contents
    for (i = 0; i < len; i++) {
        if ((pSrc[i] == RFC1662_FLAG) ||
            (pSrc[i] == RFC1662_ESCAPE)) {
            // Store escaped character
            txStore(RFC1662_ESCAPE);
            txStore(pSrc[i] ^ 0x20);
        }
        else {
            // store the character normally
            txStore(pSrc[i]);
        }
    }
    
    // End of frame
    txStore(RFC1662_FLAG);
}

// Called by USART driver when an error is detected.
// Read function will reset receive process and restart comms.
volatile bool usartErrorEncountered = false;
void usartError(UART_HandleTypeDef *huart)
{
    usartErrorEncountered = true;
}

// ------------------------------------------------------------------------
// SHTP UART HAL Methods

static int shtp_uart_hal_open(sh2_Hal_t *self_)
{
    uart_hal_t *self = (uart_hal_t *)self_;
    
    if (isOpen)
    {
        return SH2_ERR;
    }

    isOpen = true;
    
    // Assert reset
    rstn(false);
    inReset = true;

    disableInts();
    
    // Init hardware peripherals
    hal_init_hw();

    // To boot in SHTP-UART mode, must have PS1=1, PS0=0.
    // PS1 is set via jumper.
    // PS0 will be 0 if PS0 jumper is 0 OR (PS1 jumper is 1 AND PS0_WAKEN sig is 0)
    // So we set PS0_WAKEN signal to 0 just in case PS1 jumper is in 1 position.
    ps0_waken(false);
    ps1(true);

    // Use DFU flag to decide whether to go into DFU mode
    bootn(!self->dfu);

    // Delay for RESET_DELAY_US to ensure reset takes effect
    delay_us(RESET_DELAY_US);
    
    // Reset RFC 1662 decoder
    rfc1662_reset();

    // Reset BSQ/BSN negotiation
    lastBsn = 0;

    // register for rx, tx callbacks (no handlers used, actually)
    usartRegisterHandlers(&huart1, 0, 0, usartError);

    // Initialize USART peripheral
    setupUsart(SH2_BPS);

    enableInts();

    // Start data flowing
    HAL_UART_Receive_DMA(&huart1, rxBuffer, sizeof(rxBuffer));

    // Deassert reset
    rstn(true);

    // Wait for INTN to be asserted
    reset_delay_us(START_DELAY_US);

#ifdef USE_FSP200_SHTP_AUTOBAUD
    // For FSP200 SHTP-AUTOBAUD, send one autobaud sense char now.
    HAL_UART_Transmit_IT(&huart1,
                         (uint8_t *)fsp200_autobaud,
                         sizeof(fsp200_autobaud));
#endif

    return SH2_OK;
}

static void shtp_uart_hal_close(sh2_Hal_t *self)
{
    disableInts();
    
    // Hold sensor hub in reset
    rstn(false);
    inReset = true;
    
    // Disable UART
    __HAL_UART_DISABLE(&huart1);
    
    // Disable DMA
    __HAL_DMA_DISABLE(&hdma_usart1_rx);

    // Deinit timer
    __HAL_TIM_DISABLE(&tim2);

    isOpen = false;
}

static int shtp_uart_hal_read(sh2_Hal_t *self, uint8_t *pBuffer, unsigned len, uint32_t *t)
{
    uint32_t stopPoint = sizeof(rxBuffer)-__HAL_DMA_GET_COUNTER(&hdma_usart1_rx);
    int retval = 0;

    // This function needs to:
    //  * Process data from DMA buffer, through frame assembly.
    //  * Store data from BSN frames for Tx side.
    //  * Keep tx data flowing (at 1 char per 100uS.)
    //  * Deliver a whole frame to caller, if one is ready

    // If UART encountered an error, get data flowing again.
    if (usartErrorEncountered) {
        usartErrorEncountered = false;
        // reset receiver state
        rxIndex = 0;
        rfc1662_reset();
        // restart DMA process
        HAL_UART_Receive_DMA(&huart1, rxBuffer, sizeof(rxBuffer));
    }

    while ((rxIndex != stopPoint) && !rxFrameReady) {
        rfc1662_rx(rxBuffer[rxIndex]);
        rxIndex = (rxIndex+1) & sizeof(rxBuffer)-1;

        if (rxFrameReady && (rxFrameLen > 0) && (rxFrame[0] == PROTOCOL_CONTROL)) {
            // Process control protocol (BSN received)
            lastBsn = (rxFrame[2]<<8) + rxFrame[1];

            // That frame was consumed
            rxFrameReady = false;
            
            bootn(true);  // If bootn was asserted, we can deassert it now.
        }
    }

    // Try to move the tx process forward by sending a character, if possible.
    txStep();
    
    // If a frame was assembled, return it
    if (rxFrameReady) {
        if (rxFrameLen <= sizeof(rxFrame)) {
            // Set timestamp when returning a frame
            *t = rxTimestamp_uS;
        
            // Copy into pBuffer
            memcpy(pBuffer, &rxFrame[1], rxFrameLen-1);  // Copy all but first char, protocol id
        
            // signal that we consumed the frame
            retval = rxFrameLen-1;
        }
        else {
            // Frame overflowed rxFrame buffer and was discarded
        }
        rxFrameReady = false;
    }
    
    return retval;
}

static int shtp_uart_hal_write(sh2_Hal_t *self, uint8_t *pBuffer, unsigned len)
{
    // Validate parameters
    if ((pBuffer == 0) || (len == 0)) {
        return SH2_ERR_BAD_PARAM;
    }

    // If write mechanisms are busy, return 0.  Try again later.
    if (txState != TX_IDLE) {
        return 0;
    }

    // RFC encode the buffer and store in txFrame
    txEncode(pBuffer, len);
    if (txFrameLen > sizeof(txFrame)) {
        // frame overflowed the buffer after encode
        return SH2_ERR_BAD_PARAM;
    }

    // Reset txIndex 
    txIndex = 0;

    // Tx process will start with buffer status query.
    txState = TX_SENDING_BSQ;

    // Try to move the tx process forward by sending a character, if possible.
    txStep();
    
    return len;
}

static uint32_t shtp_uart_hal_getTimeUs(sh2_Hal_t *self)
{
    return timeNowUs();
}

// ------------------------------------------------------------------------
// DFU UART HAL Methods

static int bno_dfu_uart_hal_open(sh2_Hal_t *self)
{
    if (isOpen)
    {
        return SH2_ERR;
    }
    
    isOpen = true;

    disableInts();
    
    // Init hardware peripherals
    hal_init_hw();

    // Hold in reset, for DFU
    rstn(false);
    inReset = true;

    // Delay for RESET_DELAY_US to ensure reset takes effect
    delay_us(RESET_DELAY_US);
    
    // register for rx, tx callbacks
    usartRegisterHandlers(&huart1, 0, onTxCpltDfu, 0);

    // Initialize USART peripheral
    setupUsart(BNO_DFU_BPS);

    enableInts();

    // Start data flowing
    HAL_UART_Receive_DMA(&huart1, rxBuffer, sizeof(rxBuffer));

    // To boot in SHTP-UART mode, must have PS1=1, PS0=0.
    // PS1 is set via jumper.
    // PS0 will be 0 if PS0 jumper is 0 OR (PS1 jumper is 1 AND PS0_WAKEN sig is 0)
    // So we set PS0_WAKEN signal to 0 just in case PS1 jumper is in 1 position.
    ps0_waken(false);
    ps1(true);

    // Come out of reset in DFU mode
    bootn(false);

    // deassert reset
    rstn(true);
    
    // Wait until we know bootloader is up
    delay_us(DFU_BOOT_DELAY_US);

    return SH2_OK;
}

static void bno_dfu_uart_hal_close(sh2_Hal_t *self)
{
    disableInts();
    
    // Hold sensor hub in reset
    rstn(false);
    inReset = true;
    
    // Disable UART
    __HAL_UART_DISABLE(&huart1);
    
    // Disable DMA
    __HAL_DMA_DISABLE(&hdma_usart1_rx);

    // Deinit timer
    __HAL_TIM_DISABLE(&tim2);

    isOpen = false;
}

static int bno_dfu_uart_hal_read(sh2_Hal_t *self, uint8_t *pBuffer, unsigned len, uint32_t *t)
{
    uint32_t stopPoint = sizeof(rxBuffer)-__HAL_DMA_GET_COUNTER(&hdma_usart1_rx);

    // Timestamps are zero in DFU mode
    *t = 0;
        
    // Process data from DMA buffer, moving to rxFrame.
    while ((rxIndex != stopPoint) && (rxFrameLen < len)) {
        rxFrame[rxFrameLen] = rxBuffer[rxIndex];
        rxFrameLen += 1;
        rxIndex = (rxIndex+1) & sizeof(rxBuffer)-1;
    }

    // Deliver len bytes to caller, if they are ready.
    if (rxFrameLen >= len) {
        
        // Copy into pBuffer
        memcpy(pBuffer, &rxFrame, len);  // Copy rxFrame to pBuffer
        
        // We consumed that frame, reset rxFrame buffer
        rxFrameLen = 0;

        // return len to tell caller data is valid
        return len;
    }
    
    return 0;
}

static int bno_dfu_uart_hal_write(sh2_Hal_t *self, uint8_t *pBuffer, unsigned len)
{
    // Validate parameters
    if ((pBuffer == 0) || (len == 0)) {
        return SH2_ERR_BAD_PARAM;
    }

    // If write mechanisms are busy, return 0.  Try again later.
    if (txState != TX_IDLE) {
        return 0;
    }

    // Copy the data to transmit
    memcpy(txFrame, pBuffer, len);
    txFrameLen = len;

    // Send the data
    txState = TX_SENDING_DFU;
    HAL_UART_Transmit_IT(&huart1, pBuffer, len);

    return len;
}

static uint32_t bno_dfu_uart_hal_getTimeUs(sh2_Hal_t *self)
{
    return timeNowUs();
}

// ----------------------------------------------------------------------------------
// Callbacks for ISR, UART Operations
// ----------------------------------------------------------------------------------

void HAL_GPIO_EXTI_Callback(uint16_t n)
{
    inReset = false;
    rxTimestamp_uS = timeNowUs();
}

/**
 * @brief This function handles USART1 global interrupt.
 */
void USART1_IRQHandler(void)
{
    HAL_UART_IRQHandler(&huart1);
}

/**
 * @brief This function handles DMA2 stream2 global interrupt.
 */
void DMA2_Stream2_IRQHandler(void)
{
    HAL_DMA_IRQHandler(&hdma_usart1_rx);
}

void HAL_UART_RxHalfCpltCallback(UART_HandleTypeDef *huart)
{
    // Ignore
}

// Handle INTN Interrupt through STM32 HAL
// (It, in turn, calls HAL_GPIO_EXTI_Callback, above)
void EXTI15_10_IRQHandler(void)
{
    HAL_GPIO_EXTI_IRQHandler(GPIO_PIN_10);
}

// ------------------------------------------------------------------------
// Public methods

sh2_Hal_t *shtp_uart_hal_init(uart_hal_t *pHal, bool dfu)
{
    pHal->dfu = dfu;

    // Set up the HAL reference object for the client
    pHal->sh2_hal.open = shtp_uart_hal_open;
    pHal->sh2_hal.close = shtp_uart_hal_close;
    pHal->sh2_hal.read = shtp_uart_hal_read;
    pHal->sh2_hal.write = shtp_uart_hal_write;
    pHal->sh2_hal.getTimeUs = shtp_uart_hal_getTimeUs;
    
    return &pHal->sh2_hal;
}

sh2_Hal_t *bno_dfu_uart_hal_init(uart_hal_t *pHal, bool dfu)
{
    pHal->dfu = dfu;
    
    // Set up the HAL reference object for the client
    pHal->sh2_hal.open = bno_dfu_uart_hal_open;
    pHal->sh2_hal.close = bno_dfu_uart_hal_close;
    pHal->sh2_hal.read = bno_dfu_uart_hal_read;
    pHal->sh2_hal.write = bno_dfu_uart_hal_write;
    pHal->sh2_hal.getTimeUs = bno_dfu_uart_hal_getTimeUs;

    return &pHal->sh2_hal;
}



