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
 * GPIO control for debug.
 */

#ifndef DBG_H
#define DBG_H

// Initialize debug pin
void dbg_init(void);

// Pulse the debug pin <count> times, as fast as possible.
void dbg_pulse(unsigned count);

// Set the debug pin
void dbg_set(void);

// Clear the debug pin
void dbg_clear(void);

// Output a byte with manchester encoding
void dbg_manchester(unsigned byte);

// Pulse the debug2 pin <count> times, as fast as possible.
void dbg2_pulse(unsigned count);

// Set the debug2 pin
void dbg2_set(void);

// Clear the debug2 pin
void dbg2_clear(void);

#endif
