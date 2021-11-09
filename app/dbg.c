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
 * GPIO control for debug support
 */


#include "dbg.h"

#include "stm32f4xx_hal.h"

// Use pin C4 for debug output
#define DEBUG_GPIO_PORT GPIOC
#define DEBUG_GPIO_PIN  GPIO_PIN_4

// Use pin B13 for debug output 2
#define DEBUG2_GPIO_PORT GPIOB
#define DEBUG2_GPIO_PIN  GPIO_PIN_13

// Initialize debug support (set debug pin as output)
void dbg_init()
{
    GPIO_InitTypeDef GPIO_InitStruct;
  
    __HAL_RCC_GPIOC_CLK_ENABLE();
    __HAL_RCC_GPIOB_CLK_ENABLE();
  
    /*Configure GPIO pins : Debug */
    GPIO_InitStruct.Pin = DEBUG_GPIO_PIN;
    GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_HIGH;
    HAL_GPIO_Init(DEBUG_GPIO_PORT, &GPIO_InitStruct);

    /*Configure GPIO pins : Debug2 */
    GPIO_InitStruct.Pin = DEBUG2_GPIO_PIN;
    GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_HIGH;
    HAL_GPIO_Init(DEBUG2_GPIO_PORT, &GPIO_InitStruct);
}

// Pulse the debug pin high and low <count> times
void dbg_pulse(unsigned count)
{
    for (unsigned n = 0; n < count; n++) {
        HAL_GPIO_WritePin(DEBUG_GPIO_PORT, DEBUG_GPIO_PIN, GPIO_PIN_SET);
        HAL_GPIO_WritePin(DEBUG_GPIO_PORT, DEBUG_GPIO_PIN, GPIO_PIN_RESET);
    }
}

// Set the debug pin high
void dbg_set()
{
    HAL_GPIO_WritePin(DEBUG_GPIO_PORT, DEBUG_GPIO_PIN, GPIO_PIN_SET);
}

// Set the debug pin low
void dbg_clear()
{
    HAL_GPIO_WritePin(DEBUG_GPIO_PORT, DEBUG_GPIO_PIN, GPIO_PIN_RESET);
}

// send a high-low pair for every one bit, low-high pair for zero.
// This is decodable with a Saleae Logic Analyzer with the following settings:
// Mode: Manchester
// Bit Rate: 2000000
// negative edge is binary one
// 8 bits per transfer
// Most significant bit sent first
// Preamble bits to ignore: 1
// Tolerance: 25% of period (default)
void dbg_manchester(unsigned byte)
{
    // Construct the outgoing sequence in output.
    // For each bit of byte, we push two bits into output --
    // A one in byte will go out as 10, a zero will go as 01.
    uint32_t output = 0;

    // Push a "1" pattern first, this will be the stop bit
    // output = (output << 2) | 0x01;

    // Note that we visit bits of byte in reverse order that they
    // will go out on the line -- they are pushed then popped so to
    // speak.
    for (int n = 0; n < 8; n++) {
        if (byte & 0x01) {
            // Send a one
            output = (output << 2) | 0x01;
        }
        else {
            output = (output << 2) | 0x02;
        }
        byte >>= 1;
    }
    
    // Push a "1" pattern last, this will be the start bit
    output = (output << 2) | 0x01;

    // Send 18 half-bits for start, 8 data, stop.
    for (int n = 0; n < 18; n++) {
        if (output & 0x01) {
            // High
            HAL_GPIO_WritePin(DEBUG_GPIO_PORT, DEBUG_GPIO_PIN, GPIO_PIN_SET);
        }
        else {
            // Low
            HAL_GPIO_WritePin(DEBUG_GPIO_PORT, DEBUG_GPIO_PIN, GPIO_PIN_RESET);
        }
        output >>= 1;
    }

    // Always reset the line to low state at the end.
    HAL_GPIO_WritePin(DEBUG_GPIO_PORT, DEBUG_GPIO_PIN, GPIO_PIN_RESET);
}

// Pulse the debug2 pin high and low <count> times
void dbg2_pulse(unsigned count)
{
    for (unsigned n = 0; n < count; n++) {
        HAL_GPIO_WritePin(DEBUG2_GPIO_PORT, DEBUG2_GPIO_PIN, GPIO_PIN_SET);
        HAL_GPIO_WritePin(DEBUG2_GPIO_PORT, DEBUG2_GPIO_PIN, GPIO_PIN_RESET);
    }
}

// Set the debug pin high
void dbg2_set()
{
    HAL_GPIO_WritePin(DEBUG2_GPIO_PORT, DEBUG2_GPIO_PIN, GPIO_PIN_SET);
}

// Set the debug pin low
void dbg2_clear()
{
    HAL_GPIO_WritePin(DEBUG2_GPIO_PORT, DEBUG2_GPIO_PIN, GPIO_PIN_RESET);
}
