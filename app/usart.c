/**
 * Copyright 2017-21 CEVA, Inc.
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
 * @file usart.h
 * @author David Wheeler
 * @date 24 May 2017
 * @brief Callback registration mechanism for USART peripheral on STM32F411re Nucleo board
 */

/*
 * The STM32 HAL Libraries support callback functions named HAL_UART_RxCpltCallback and
 * HAL_UART_TxCpltCallback (among others.)  But our application needs to provide callbacks
 * for different USART peripherals from different code modules.  So this file implements
 * a simple registration mechanism.  Clients register the callbacks to be used for each
 * USART instance.  This file implements the main callback function and then invokes the 
 * client-specific ones.
 */

#include "usart.h"
#include <stdbool.h>
#include <string.h>
#include <stm32f4xx_hal.h>

// ------------------------------------------------------------------------
// Private Type definitions

// A USART handler record stores callbacks associated with particular USART.
typedef struct {
    USART_TypeDef *Instance;
    UART_HandleTypeDef *huart;
    CpltCallback_t *rxCplt;
    CpltCallback_t *txCplt;
    CpltCallback_t *err;
} UsartHandler_t;

#define NUM_USART (4)

// ------------------------------------------------------------------------
// Forward declarations

static void usartInit(void);

// ------------------------------------------------------------------------
// Private data
// USART handler records
static UsartHandler_t handler[NUM_USART];

// Flag to ensure initialization happens once, when needed.
static bool usartInitialized = false;

// ------------------------------------------------------------------------
// Public API

// Register a client for a particular USART instance.
void usartRegisterHandlers(UART_HandleTypeDef *huart,
                           CpltCallback_t *rxCplt,
                           CpltCallback_t *txCplt,
                           CpltCallback_t *err)
{
    if (!usartInitialized) {
        usartInit();
    }

    // Clear any previous entry for this huart
    for (int n = 0; n < NUM_USART; n++) {
        if (handler[n].Instance == huart->Instance) {
            handler[n].Instance = 0;
            handler[n].huart = 0;
            handler[n].rxCplt = 0;
            handler[n].txCplt = 0;
            handler[n].err = 0;
        }
    }

    // Find an open entry (or where this uart already registered)
    for (int n = 0; n < NUM_USART; n++) {
        if (handler[n].Instance == 0) {
            handler[n].Instance = huart->Instance;
            handler[n].huart = huart;
            handler[n].rxCplt = rxCplt;
            handler[n].txCplt = txCplt;
            handler[n].err = err;
            break;
        }
    }
}

// Unregister a client.
void usartUnregisterHandlers(UART_HandleTypeDef *huart)
{
    if (!usartInitialized) {
        usartInit();
    }

    // Clear any previous entry for this huart
    for (int n = 0; n < NUM_USART; n++) {
        if (handler[n].Instance == huart->Instance) {
            handler[n].Instance = 0;
            handler[n].huart = 0;
            handler[n].rxCplt = 0;
            handler[n].txCplt = 0;
            handler[n].err = 0;
        }
    }
}

// This is the actual TX callback invoked by the STM32 HAL library
void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart)
{
    for (int n = 0; n < NUM_USART; n++) {
        if (handler[n].huart == huart) {
            if (handler[n].txCplt != 0) {
                handler[n].txCplt(huart);
            }
        }
    }
}

// This is the actual RX callback invoked by the STM32 HAL library
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
    for (int n = 0; n < NUM_USART; n++) {
        if (handler[n].huart == huart) {
            if (handler[n].rxCplt != 0) {
                handler[n].rxCplt(huart);
            }
        }
    }
}

void HAL_UART_ErrorCallback(UART_HandleTypeDef *huart)
{
    for (int n = 0; n < NUM_USART; n++) {
        if (handler[n].huart == huart) {
            if (handler[n].err != 0) {
                handler[n].err(huart);
            }
        }
    }
}

// ------------------------------------------------------------------------
// Private functions

static void usartInit(void)
{
    // Initialize the array of registrations
    memset(handler, 0, sizeof(handler));

    usartInitialized = true;
}
