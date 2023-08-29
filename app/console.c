/*
 * Copyright 2015-21 CEVA, Inc.
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
 * usart2 console
 * Supports standard i/o over VCOM USB interface on Nucleo F401/F411 boards.
 */

#include "console.h"

#include <stdbool.h>
#include <stm32f4xx_hal.h>
#include <string.h>

#include "usart.h"

#define CONSOLE_BUFLEN (4*4096)
#define CONSOLE_BAUD_RATE 115200
// #define CONSOLE_BAUD_RATE 2000000  // Use this when logging DSF records

// ------------------------------------------------------------------------
// Private types

typedef struct Fifo_s {
    uint8_t *buffer;
    uint32_t buflen;
    volatile unsigned nextIn;
    volatile unsigned nextOut;
} Fifo_t;

// ------------------------------------------------------------------------
// Private state variables

// The UART used by the console
static UART_HandleTypeDef consoleUart; 

// Transmit support
static uint8_t txFifoBuffer[CONSOLE_BUFLEN];
static Fifo_t txFifo;
static volatile bool txActive;  // true if a call to HAL_UART_Transmit_IT is in flight
static uint8_t txBuffer[CONSOLE_BUFLEN];

// Receive support
static uint8_t rxFifoBuffer[CONSOLE_BUFLEN];
static Fifo_t rxFifo;
static volatile bool rxActive;  // true if a call to HAL_UART_Recieve_IT is in flight
static uint8_t rxChar;
static uint32_t rxDrops;

// ------------------------------------------------------------------------
// Forward declarations

static void startTx(void);
static void consoleRxCplt(UART_HandleTypeDef *huart);
static void consoleTxCplt(UART_HandleTypeDef *huart);

// FIFO Operations
static void fifo_init(Fifo_t *fifo, uint8_t *pBuf, uint32_t bufLen);
static bool fifo_isEmpty(Fifo_t *fifo);
static unsigned fifo_insert(Fifo_t *fifo, uint8_t *pData, unsigned len);
static unsigned fifo_insert1(Fifo_t *fifo, uint8_t c);
static unsigned fifo_remove(Fifo_t *fifo, uint8_t *pData, unsigned len);

// ------------------------------------------------------------------------
// Public API

void console_init(void)
{
    GPIO_InitTypeDef GPIO_InitStruct;
    
    __HAL_RCC_USART2_CLK_ENABLE();
    __HAL_RCC_GPIOA_CLK_ENABLE();
    
    // Configure GPIO for Console: PA2=USART2_TX, PA3=USART2_RX
    GPIO_InitStruct.Pin = GPIO_PIN_2|GPIO_PIN_3;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull = GPIO_PULLUP;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
    GPIO_InitStruct.Alternate = GPIO_AF7_USART2;
    HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

    /* Peripheral interrupt init */
    HAL_NVIC_SetPriority(USART2_IRQn, 7, 0);

    // Init UART itself
    consoleUart.Instance = USART2;
    consoleUart.Init.BaudRate = CONSOLE_BAUD_RATE;;
    consoleUart.Init.WordLength = UART_WORDLENGTH_8B;
    consoleUart.Init.StopBits = UART_STOPBITS_1;
    consoleUart.Init.Parity = UART_PARITY_NONE;
    consoleUart.Init.Mode = UART_MODE_TX_RX;
    consoleUart.Init.HwFlowCtl = UART_HWCONTROL_NONE;
    consoleUart.Init.OverSampling = UART_OVERSAMPLING_16;
    HAL_UART_Init(&consoleUart);

    // Register for rxCplt and txCplt callbacks on console uart
    usartRegisterHandlers(&consoleUart, consoleRxCplt, consoleTxCplt, 0);

    // Init FIFOs
    fifo_init(&txFifo, txFifoBuffer, sizeof(txFifoBuffer));
    fifo_init(&rxFifo, rxFifoBuffer, sizeof(rxFifoBuffer));

    // Transmit inactive initially
    txActive = false;
    
    // Start receiving characters
    rxActive = true;
    HAL_UART_Receive_IT(&consoleUart, &rxChar, 1);
    
    // Enable interrupts now that we're ready.
    HAL_NVIC_EnableIRQ(USART2_IRQn);
}

size_t __read(int Handle, unsigned char * Buf, size_t BufSize)
{
    size_t copied = 0;

    // This function only works for stdin
    if (Handle != 0) {
        return -1;
    }

    copied = fifo_remove(&rxFifo, Buf, BufSize);

    return copied;
}

int getchar(void)
{
    uint8_t c = -1;
        
    if (!rxActive) {
        // Start receiving.
        rxActive = true;
        HAL_UART_Receive_IT(&consoleUart, &rxChar, 1);
    }

    while (fifo_isEmpty(&rxFifo)) {
        // Wait for data
    }

    fifo_remove(&rxFifo, &c, 1);
    
    // translate CR to LF
    if (c == '\r') {
        c = '\n';
    }
        
    // echo
    putchar(c);

    return (int)c;
}

size_t __write(int Handle, const unsigned char * Buf, size_t Bufsize)
{
    size_t n;
    int rc;

    // This function only works for stdout, stderr
    if (!((Handle == 1) || (Handle == 2))) {
        return -1;
    }

    for (n = 0; n < Bufsize; n++) {
        rc = putchar(Buf[n]);
        if (rc < 0) break;
    }

    return n;
}

int putchar(int c)
{
    // insert CF before each LF
    if (c == '\n') {
        fifo_insert1(&txFifo, '\r');
    }

    // insert this character
    fifo_insert1(&txFifo, (uint8_t)c);

    // Activate transmission if not already active
    startTx();

    return c;
}

// ------------------------------------------------------------------------
// Private utility functions

// Process USART2 IRQ through STM32 HAL
void USART2_IRQHandler(void)
{
    HAL_UART_IRQHandler(&consoleUart);
}

// If transmit inactive, start it.
static void startTx(void)
{
    unsigned len;
    
    // If tx is inactive, start it.
    if (!txActive) {
        // Load txBuffer from tx Fifo
        len = fifo_remove(&txFifo, txBuffer, sizeof(txBuffer));

        if (len > 0) {
            txActive = true;
            HAL_UART_Transmit_IT(&consoleUart, txBuffer, len);
        }
    }
}

// When transmit completes
static void consoleTxCplt(UART_HandleTypeDef *huart)
{
    unsigned len;
    
    // One transmission is complete.

    // If there is more data to transmit now, immediately start it.
    len = fifo_remove(&txFifo, txBuffer, 1);

    if (len > 0)
    {
        HAL_UART_Transmit_IT(&consoleUart, txBuffer, len);
    }
    else
    {
        txActive = false;
    }
}

// When a receive operation completes.  (We do receive one char at a time.)
static void consoleRxCplt(UART_HandleTypeDef *huart)
{
    // rxChar now has the latest input.
    // Insert the new char in input fifo.
    unsigned len = fifo_insert(&rxFifo, &rxChar, 1);
    if (len == 0) {
        // If fifo insert failed, count an overflow
        rxDrops++;
    }

    // Start reception of next character
    HAL_UART_Receive_IT(&consoleUart, &rxChar, 1);
}

// Init a fifo structure
static void fifo_init(Fifo_t *fifo, uint8_t *pBuf, uint32_t bufLen)
{
    fifo->buffer = pBuf;
    fifo->buflen = bufLen;
    
    // Stuff the buffer with dummy data
    memset(fifo->buffer, 'Z', bufLen);

    // Set in, out indices to start of buffer.
    fifo->nextIn = 0;
    fifo->nextOut = 0;
}

// return true if fifo is empty
static bool fifo_isEmpty(Fifo_t *fifo)
{
    unsigned in = fifo->nextIn;
    unsigned out = fifo->nextOut;
    return (in == out);
}

// Insert up to len bytes in fifo.  Returns length actually inserted
static unsigned fifo_insert(Fifo_t *fifo, uint8_t *pData, unsigned len)
{
    unsigned in;        // local copy of nextIn (we will only update the real nextIn once.)
    unsigned beyondIn;  // one index position beyond in, with wrap
    unsigned n;         // index into pData

    n = 0;
    in = fifo->nextIn;
    beyondIn = in + 1;
    if (beyondIn >= fifo->buflen)
    {
        beyondIn = 0;
    }
    
    // While there is data to insert and room to insert it...
    while ((n < len) && (beyondIn != fifo->nextOut)) {
        fifo->buffer[in] = pData[n];
        n++;
        in = beyondIn;
        beyondIn++;
        if (beyondIn > fifo->buflen)
        {
            beyondIn = 0;
        }
    }

    // Update input pointer once
    fifo->nextIn = in;

    // Copied this many bytes
    return n;
}

// Insert one byte in fifo.  Returns 1 if successful, otherwise 0
static unsigned fifo_insert1(Fifo_t *fifo, uint8_t c)
{
    return fifo_insert(fifo, &c, 1);
}

// Remove up to len bytes from fifo.  Returns length actually removed
static unsigned fifo_remove(Fifo_t *fifo, uint8_t *pData, unsigned len)
{
    unsigned out;        // local copy of nextOut
    unsigned n;          // index into pData

    n = 0;
    out = fifo->nextOut;

    // While there is space to read to and data to read
    while ((n < len) && (out != fifo->nextIn)) {
        pData[n] = fifo->buffer[out];
        n++;
        out++;
        if (out >= fifo->buflen)
        {
            out = 0;
        }
    }

    // Update input pointer once
    fifo->nextOut = out;

    // Copied this many bytes
    return n;
}
