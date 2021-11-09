/*
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
 * Simple Calibration App for CEVA FSP200.
 */


// Sensor Application
#include <stdio.h>
#include <string.h>

#include "demo_app.h"
#include "sh2.h"
#include "sh2_err.h"
#include "sh2_hal_init.h"

// --- Forward declarations -------------------------------------------

static void eventHandler(void * cookie, sh2_AsyncEvent_t *pEvent);
static void reportProdIds(void);
static void startCal(void);
static void serviceCal(void);

// --- Private data ---------------------------------------------------

static sh2_Hal_t *pSh2Hal = 0;

static bool resetOccurred = false;

static sh2_CalStatus_t calStatus;

static const char * calStatusMsg[] = {
    "Success",
    "No ZRO",
    "No stationary detection",
    "Rotation outside of specification",
    "ZRO outside of specification",
    "ZGO outside of specifidation",
    "Gyro gain outside of specification",
    "Gyro period outside of specification",
    "Gyro sample drops outside of specification",
};

typedef enum
{
    CAL_WAIT_START,
    CAL_WAIT_FINISH,
    CAL_DONE,
} CalState_t;

static CalState_t calState;

// --- Public methods -------------------------------------------------

// Called once during system initialization
void demo_init(void)
{
    int status;
    
    printf("\n\n");
    printf("CEVA FSP200 Calibration Demo.\n");
    
    // Create HAL instance
    pSh2Hal = sh2_hal_init();

    // Open SH2 interface (also registers non-sensor event handler.)
    status = sh2_open(pSh2Hal, eventHandler, NULL);
    if (status != SH2_OK)
    {
        printf("Error, %d, from sh2_open.\n", status);
    }

    // resetOccurred would have been set earlier.
    // We can reset it since we are starting the sensor reports now.
    resetOccurred = false;

    // Read and display sensor hub product ids
    reportProdIds();

    // Init calibration process
    startCal();
}

// Called repeatedly during system operation
void demo_service(void)
{
    if (resetOccurred) {
        // reinit calibration process
        startCal();
    }

    // Service the sensor hub.
    sh2_service();

    // Service calibration state machine
    serviceCal();

    if (calState == CAL_DONE)
    {
        // Restart calibration each time it finishes.
        startCal();
    }
}

// --- Private methods ----------------------------------------------

static void eventHandler(void * cookie, sh2_AsyncEvent_t *pEvent)
{
    if (pEvent->eventId == SH2_RESET) {
        // Set flag indicating we saw a reset.
        resetOccurred = true;
    }
}

static void reportProdIds(void)
{
    sh2_ProductIds_t prodIds;
    int status;
     
    memset(&prodIds, 0, sizeof(prodIds));
    status = sh2_getProdIds(&prodIds);
    
    if (status < 0) {
        printf("Error from sh2_getProdIds.\n");
        return;
    }

    // Report the results
    for (int n = 0; n < prodIds.numEntries; n++) {
        printf("Part %d : Version %d.%d.%d Build %d\n",
               prodIds.entry[n].swPartNumber,
               prodIds.entry[n].swVersionMajor, prodIds.entry[n].swVersionMinor, 
               prodIds.entry[n].swVersionPatch, prodIds.entry[n].swBuildNumber);
    }
}

// Start calibration process
static void startCal(void)
{
    // Prompt for start
    printf("Put module in start orientation, press ENTER.\n");
    printf("> ");

    // Set state
    calStatus = SH2_CAL_SUCCESS;
    calState = CAL_WAIT_START;
}

static bool pressedEnter(void)
{
    char c = getchar();
    if (c == '\n')
    {
        return true;
    }
    else
    {
        return false;
    }
}

static void serviceCal(void)
{
    switch (calState)
    {
        case CAL_WAIT_START:
            // Waiting for user to press ENTER
            if (pressedEnter())
            {
                uint32_t interval_us = 10000; // calibrate for 100Hz operation
                int status = sh2_startCal(interval_us);
                if (status != SH2_OK)
                {
                    // End calibration process with error
                    printf("Error from sh2_startCal: %d\n", status);
                    calState = CAL_DONE;
                }
                else
                {
                    // Proceed to next step: final orientation
                    printf("Rotate module to final orientation, press ENTER.\n");
                    printf("> ");
                    calState = CAL_WAIT_FINISH;
                }
            }
            break;
        case CAL_WAIT_FINISH:
            // Waiting for user to press ENTER at final orientation
            if (pressedEnter())
            {
                int status = sh2_finishCal(&calStatus);
                if (status != SH2_OK)
                {
                    // End calibration process with error
                    printf("Error from sh2_finishCal: %d\n", status);
                }
                else if (calStatus != 0)
                {
                    // Calibration process ended
                    printf("Calibration completed with status: %d\n", calStatus);
                    printf("%s\n", calStatusMsg[calStatus]);
                }
                else
                {
                    printf("Calibration completed successfully.\n");
                }
                calState = CAL_DONE;
            }
            break;
        case CAL_DONE:
            break;
    }
}
