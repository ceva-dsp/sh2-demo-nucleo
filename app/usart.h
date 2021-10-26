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

#ifndef USART_H
#define USART_H

#include <stm32f4xx_hal.h>

typedef void (CpltCallback_t)(UART_HandleTypeDef *huart);

// Register a client for a particular USART instance.
void usartRegisterHandlers(UART_HandleTypeDef *huart,
                           CpltCallback_t *rxCplt, CpltCallback_t *txCplt,
                           CpltCallback_t *err);

// Unregister a client
void usartUnregisterHandlers(UART_HandleTypeDef *huart);


#endif
