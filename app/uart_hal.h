/*
 * Copyright 2022 CEVA, Inc.
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
 * UART HAL support
 */

#ifndef UART_HAL_H
#define UART_HAL_H

#include "sh2_hal.h"
#include <stdbool.h>

typedef struct uart_hal_s {
    sh2_Hal_t sh2_hal;       // Must be first so we can cast (sh2_hal_t *) to (uart_hal_t *)
    bool dfu;
} uart_hal_t;

sh2_Hal_t *shtp_uart_hal_init(uart_hal_t *pHal, bool dfu);

sh2_Hal_t *bno_dfu_uart_hal_init(uart_hal_t *pHal, bool dfu);

#endif
