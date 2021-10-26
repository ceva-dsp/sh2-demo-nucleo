/*
 * Copyright 2020-21 CEVA, Inc.
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

#include "rvc_hal.h"

#include <stdbool.h>
#include <string.h>
#include "rvc.h"

#include "stm32f4xx_hal.h"

#include "usart.h"

// -------------------------------------------------------------------------
// Constants

#define RVC_BPS (115200)         // 115200 bps for RVC mode

// UART pins
#define TX_PORT GPIOA
#define TX_PIN  GPIO_PIN_9
#define RX_PORT GPIOB
#define RX_PIN  GPIO_PIN_3

// GPIO interface pins
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

// Keep reset asserted this long.
// (Some targets have a long RC decay on reset.)
#define RESET_DELAY_US (10000)

// ------------------------------------------------------------------------
// Private data

static bool isOpen = false;

// True between asserting reset and seeing first data from sensor hub
static volatile bool inReset;

// Timer handle
TIM_HandleTypeDef tim2;

// USART1 handle
UART_HandleTypeDef huart1;

// DMA stream for USART1 Rx.
DMA_HandleTypeDef hdma_usart1_rx;

// receive support
#define DMA_SIZE (256)  // Must be a power of 2
static uint8_t rxBuffer[DMA_SIZE]; // receives UART data via DMA (must be a power of 2)
static uint32_t rxIndex = 0;               // next index to read

// RVC frame
#define RVC_HEADER    ( 0)
#define RVC_INDEX     ( 2)
#define RVC_YAW_LSB   ( 3)
#define RVC_YAW_MSB   ( 4)
#define RVC_PITCH_LSB ( 5)
#define RVC_PITCH_MSB ( 6)
#define RVC_ROLL_LSB  ( 7)
#define RVC_ROLL_MSB  ( 8)
#define RVC_ACC_X_LSB ( 9)
#define RVC_ACC_X_MSB (10)
#define RVC_ACC_Y_LSB (11)
#define RVC_ACC_Y_MSB (12)
#define RVC_ACC_Z_LSB (13)
#define RVC_ACC_Z_MSB (14)
#define RVC_MI        (15)
#define RVC_MR        (16)
#define RVC_RESERVED  (17)
#define RVC_CSUM      (18)

#define RVC_FRAME_LEN (19)
static uint8_t rxFrame[RVC_FRAME_LEN];  // receives frame from sensor hub
static uint32_t rxFrameLen = 0;         // length of frame so far
static bool rxFrameReady = false;       // set true when a full, valid frame received.


// ------------------------------------------------------------------------
// Private methods

static void disableInts(void)
{
    HAL_NVIC_DisableIRQ(DMA2_Stream2_IRQn);
    HAL_NVIC_DisableIRQ(USART1_IRQn);
}

static void enableInts(void)
{
    HAL_NVIC_EnableIRQ(USART1_IRQn);
    HAL_NVIC_EnableIRQ(DMA2_Stream2_IRQn);
}

static void init_timer(void)
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

static void init_gpio(void)
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
}

static void init_dma(void)
{
    /* DMA controller clock enable */
    __HAL_RCC_DMA2_CLK_ENABLE();

    /* DMA interrupt init */
    /* DMA2_Stream2_IRQn interrupt configuration */
    HAL_NVIC_SetPriority(DMA2_Stream2_IRQn, 5, 0);
}

static void init_usart(void)
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

static void init_hw(void)
{
    init_timer();
    init_gpio();
    init_dma();
    init_usart();
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

static void bootn(bool state)
{
    HAL_GPIO_WritePin(BOOTN_PORT, BOOTN_PIN, 
                      state ? GPIO_PIN_SET : GPIO_PIN_RESET);
}

static uint32_t timeNowUs(void)
{
    return __HAL_TIM_GET_COUNTER(&tim2);
}

static uint64_t timeNowUs_64(void)
{
    static uint32_t last = 0;
    static uint32_t rollovers = 0;
    
    uint32_t now = timeNowUs();
    if (now < last) {
        rollovers++;
    }
    
    last = now;

    return ((uint64_t)rollovers << 32) | now;
}

// delay for t microseconds
static void delay_us(uint32_t t)
{
    uint32_t start, now;

    now = timeNowUs();
    start = now;
    while ((now - start) < t) {
        now = timeNowUs();
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
        return RVC_ERR_IO;
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
        return RVC_ERR_IO;
    }
  
    return RVC_OK;
}

static bool checksumOk()
{
    uint8_t check = 0;

    for (int n = 2; n < RVC_CSUM; n++) {
        check += rxFrame[n];
    }

    return (check == rxFrame[RVC_CSUM]);
}

// Process one character of an RVC frame
void rx(uint8_t c)
{
    if (rxFrameLen == RVC_FRAME_LEN) {
        // rxFrame has filled but we still don't have a valid frame.
        // (a checksum error?  Or bad frame sync?)  Anyways, shift data
        // into the rxFrame buffer until we have a good, whole frame
        for (int n = 0; n < RVC_FRAME_LEN-1; n++) {
            rxFrame[n] = rxFrame[n+1];
        }
        rxFrame[RVC_FRAME_LEN-1] = c;
    }
    else {
        // Just stuff the latest char into the buffer at the end
        rxFrame[rxFrameLen++] = c;
    }

    // If rx buffer is full, see if we have a valid frame
    if ((rxFrameLen == RVC_FRAME_LEN) &&
        (rxFrame[0] == 0xAA) &&
        (rxFrame[1] == 0xAA) &&
        checksumOk()) {
        // It's a good frame
        rxFrameReady = true;
    }
}

// ----------------------------------------------------------------------------------
// Callbacks for ISR, UART Operations
// ----------------------------------------------------------------------------------

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

// -----------------------------------------------------------------------
// Public methods

int rvc_hal_open(void)
{
    if (isOpen) {
        // It's an error to open an already-open RVC device
        return RVC_ERR;
    }
    
    // disable interrupts
    disableInts();

    // init hardware peripherals
    init_hw();
    
    // reset part
    rstn(false);
    inReset = true;
    
    // init USART
    usartRegisterHandlers(&huart1, 0, 0, 0);
    setupUsart(RVC_BPS);
    
    // delay to ensure reset hold time
    delay_us(RESET_DELAY_US);
    
    // re-enable interrupts
    enableInts();
    
    // start data flowing through UART
    for (int n = 0; n < sizeof(rxBuffer); n++) {
        rxBuffer[n] = 0xAA;
    }
    HAL_UART_Receive_DMA(&huart1, rxBuffer, sizeof(rxBuffer));
    
    // set PS0, PS1, BOOTN to boot into RVC mode
    // To boot in RVC mode, must have PS1=0, PS0=1.
    // PS1 is set via jumper.
    // PS0 will be 0 if PS0 jumper is 0 OR (PS1 jumper is 1 AND PS0_WAKEN sig is 0)
    // So we set PS0_WAKEN signal to 0 just in case PS1 jumper is in 1 position.
    ps0_waken(true);   
    ps1(false);
    bootn(true);

    // deassert reset
    rstn(true);
    inReset = false;
    
    isOpen = true;

    return RVC_OK;
}

void rvc_hal_close(void)
{
    // disable interrupts
    disableInts();
    
    // hold device in reset
    rstn(false);
    inReset = true;
    
    // disable USART, DMA
    __HAL_UART_DISABLE(&huart1);
    __HAL_DMA_DISABLE(&hdma_usart1_rx);

    // disable timer
    __HAL_TIM_DISABLE(&tim2);
    
    isOpen = false;
}

int rvc_hal_read(rvc_SensorEvent_t *pEvent)
{
    int retval = 0;
    
    // DMA has received characters up to this point in rxBuffer
    uint32_t stopPoint = sizeof(rxBuffer)-__HAL_DMA_GET_COUNTER(&hdma_usart1_rx);

    // Process data until we've seen everything or we have a fully formed
    // RVC frame in rxFrame.
    while ((rxIndex != stopPoint) && !rxFrameReady) {
        // process one input character
        rx(rxBuffer[rxIndex]);
        rxIndex = (rxIndex + 1) & (sizeof(rxBuffer)-1);
    }

    if (rxFrameReady) {
        // copy data into pBuffer
        pEvent->timestamp_uS = timeNowUs_64();
        pEvent->index = rxFrame[RVC_INDEX];
        pEvent->yaw =   (int16_t)((rxFrame[RVC_YAW_MSB] << 8) + rxFrame[RVC_YAW_LSB]);
        pEvent->pitch = (int16_t)((rxFrame[RVC_PITCH_MSB] << 8) + rxFrame[RVC_PITCH_LSB]);
        pEvent->roll =  (int16_t)((rxFrame[RVC_ROLL_MSB] << 8) + rxFrame[RVC_ROLL_LSB]);
        pEvent->acc_x = (int16_t)((rxFrame[RVC_ACC_X_MSB] << 8) + rxFrame[RVC_ACC_X_LSB]);
        pEvent->acc_y = (int16_t)((rxFrame[RVC_ACC_Y_MSB] << 8) + rxFrame[RVC_ACC_Y_LSB]);
        pEvent->acc_z = (int16_t)((rxFrame[RVC_ACC_Z_MSB] << 8) + rxFrame[RVC_ACC_Z_LSB]);
        pEvent->mi =    (uint8_t)(rxFrame[RVC_MI]);
        pEvent->mr =    (uint8_t)(rxFrame[RVC_MI]);

        retval = 1;

        // reset the rx frame
        rxFrameReady = false;
        rxFrameLen = 0;
    }
    
    return retval;
}

