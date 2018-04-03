/*
 * Copyright 2017-2018 Hillcrest Laboratories, Inc.
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
 * Declare init methods for sh2 HAL and DFU HAL.
 */

#ifndef SH2_HAL_INIT_H
#define SH2_HAL_INIT_H

#include "sh2_hal.h"

// Initialize the SHTP HAL and return a reference to it.
sh2_Hal_t *sh2_hal_init(void);
sh2_Hal_t *dfu_hal_init(void);
sh2_Hal_t *fsp200_dfu_hal_init(void);

#endif
