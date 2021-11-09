/*
 * Copyright 2017-2021 CEVA, Inc.
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
 * Demo App for SH2 devices (BNO08x and FSP200)
 */

// ------------------------------------------------------------------------

// Sensor Application
#include <stdio.h>
#include <string.h>
#include <stdint.h>

#include "demo_app.h"

#include "rvc.h"

// Uncomment this to produce DSF format data output
// #define DSF_OUTPUT

// --- Private methods --------------------------------------------------

// Print headers for DSF format output
void printDsfHeaders(void)
{
    printf("+0 TIME[x]{us},SAMPLE_ID[x]{samples},EULER[xyz]{deg},LIN_ACC_GRAVITY[xyz]{g},MOTION_INTENT[x]{state},MOTION_REQUEST[x]{state}\n");
    printf("!0 name=\"Format-H\"\n");
}

// Print a sensor event as a DSF record
void printDsf(const rvc_SensorEvent_t * event)
{
    float t;
    static uint32_t lastSequence;  // last sequence number
    rvc_SensorValue_t value;

    // Convert event to value
    rvc_decode(&value, event);
    
    // Compute 32-bit sample_id
    uint8_t deltaSeq = value.index - (lastSequence & 0xFF);
    lastSequence += deltaSeq;

    // Get time as float
    t = value.timestamp_uS / 1000000.0;

    printf(".%d %0.6f,%d,%0.2f,%0.2f,%0.2f,%0.3f,%0.3f,%0.3f,%d,%d\n",
           0,                                 // using sensor id 0
           t,                                 // time
           lastSequence,                      // sequence
           value.pitch_deg,                   // euler angles [deg]
           value.roll_deg,
           value.yaw_deg,
           value.acc_x_g,                     // accel [g]
           value.acc_y_g,                     // accel [g]
           value.acc_z_g,                     // accel [g]
           value.mi,                          // motion_intent
           value.mr                           // motion_request
        );
}

// Print a sensor event to the console
void printEvent(const rvc_SensorEvent_t * event)
{
    rvc_SensorValue_t value;
    
    // decode rvc sensor event.
    rvc_decode(&value, event);

    printf("%3d : yaw:%0.2f pitch:%0.2f roll:%0.2f ax:%0.3f ay:%0.3f az:%0.3f mi:%d mr:%d\n",
           value.index,
           value.yaw_deg,
           value.pitch_deg,
           value.roll_deg,
           value.acc_x_g,
           value.acc_y_g,
           value.acc_z_g,
           value.mi,
           value.mr
        );
}

// Handle sensor events.
static void sensorHandler(void * cookie, rvc_SensorEvent_t *pEvent)
{
#ifdef DSF_OUTPUT
    printDsf(pEvent);
#else
    printEvent(pEvent);
#endif
}

// --- Public methods -------------------------------------------------

// Initialize demo. 
void demo_init(void)
{
    int status;
    
    printf("\n\n");
    printf("CEVA RVC Demo.\n");

    status = rvc_init();
    if (status != RVC_OK) {
        printf("Error, %d, from rvc_init.\n", status);
    }

    status = rvc_setCallback(sensorHandler, NULL);
    if (status != RVC_OK) {
        printf("Error, %d, from rvc_setCallback.\n", status);
    }
    
    status = rvc_open();
    if (status != RVC_OK) {
        printf("Error, %d, from rvc_open.\n", status);
    }

#ifdef DSF_OUTPUT
    // Print DSF file headers
    printDsfHeaders();
#endif
}

// This must be called periodically.  (The demo main calls it continuously in a loop.)
// It calls sh2_service to keep data flowing between host and sensor hub.
void demo_service(void)
{
    // Check on uart driver.  Data will be delivered via callback.
    rvc_service();
}



