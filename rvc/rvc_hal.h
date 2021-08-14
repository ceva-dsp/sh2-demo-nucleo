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

// Hardware-dependent portions of RVC interface.

#ifndef RVC_HAL_H
#define RVC_HAL_H

#include <stdint.h>

#include "rvc.h"

int rvc_hal_open();
void rvc_hal_close();
int rvc_hal_read(rvc_SensorEvent_t *event);

#endif
