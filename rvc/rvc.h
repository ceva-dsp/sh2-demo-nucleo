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


#ifndef RVC_H
#define RVC_H

#include <stdint.h>

// return status values
#define RVC_OK                 (0)  /**< Success */
#define RVC_ERR                (-1) /**< General Error */
#define RVC_ERR_BAD_PARAM      (-2) /**< Bad parameter to an API call */
#define RVC_ERR_OP_IN_PROGRESS (-3) /**< Operation in progress */
#define RVC_ERR_IO             (-4) /**< Error communicating with hub */
#define RVC_ERR_HUB            (-5) /**< Error reported by hub */
#define RVC_ERR_TIMEOUT        (-6) /**< Operation timed out */

// mi field, Motion Intent, values:
#define MI_UNKNOWN                       (0)
#define MI_STATIONARY_WITHOUT_VIBRATION  (1)
#define MI_STATIONARY_WITH_VIBRATION     (2)
#define MI_IN_MOTION                     (3)

#define MR_NO_CONSTRAINT                 (0)
#define MR_STAY_STATIONARY_REQUIRED      (1)
#define MR_STAY_STATIONARY_OPTIONAL      (2) // Deprecated, ignore
#define MR_NON_URGENT_STAY_STATIONARY    (3)
#define MR_URGENT_STATIONARY             (4)
#define MR_TIMER_STATIONARY              (5)

// Use integers as references to RVC HAL instances.
// (Maybe change to a structure later?)
typedef int RvcHal_t;

typedef struct rvc_SensorEvent_s
{
    uint8_t index;
    int16_t yaw;
    int16_t pitch;
    int16_t roll;
    int16_t acc_x;
    int16_t acc_y;
    int16_t acc_z;
    uint8_t mi;      // Motion intent
    uint8_t mr;      // Motion request
    uint64_t timestamp_uS;
} rvc_SensorEvent_t;

typedef struct rvc_SensorValue_s
{
    uint8_t index;
    float yaw_deg;
    float pitch_deg;
    float roll_deg;
    float acc_x_g;
    float acc_y_g;
    float acc_z_g;
    uint8_t mi;      // Motion intent
    uint8_t mr;      // Motion request
    uint64_t timestamp_uS;
} rvc_SensorValue_t;

typedef void rvc_Callback_t(void *cookie, rvc_SensorEvent_t *pEvent);

// Public API calls used by demo_rvc.c

// Initialize the RVC sensor hub module
int rvc_init();

// Application registers callback function to receive sensor events
int rvc_setCallback(rvc_Callback_t *callback, void * cookie);

// Open starts the flow of sensor events from the sensor hub.
int rvc_open();

// Close stops the flow of sensor events from the sensor hub.
void rvc_close();

// Service performs periodic servicing of the sensor hub interface.
// This reads the UART data and should be called frequently to keep
// communications flowing.
void rvc_service();

// Convert data from integers, fixed point to floating point, natural units.
void rvc_decode(rvc_SensorValue_t *value, const rvc_SensorEvent_t *event);

#endif
