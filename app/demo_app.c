/*
 * Copyright 2017-2023 CEVA, Inc.
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
 * Demo App for SH-2 devices (BNO08x and FSP20x)
 */

// ------------------------------------------------------------------------
// Configure Compile time options for the demo app

// Running this demo.
// The demo app included here, simply establishes communications with the
// sensor hub, enables one or more sensors, then prints the sensor reports
// to the console.
//
// To select which sensors are enabled, you can edit the enabledSensors[]
// array, found below.  And the requested rate is set in
// config.reportInterval_us.
//
// Normally, sensor values are printed in a format that's easy to read.  But
// the user may want to produce records in DSF format for further analysis.
// To do so, enable the DSF_OUTPUT option, below.
//
// If the shake detector is enabled, you may also want to configure that
// detector for a different threshold or timing.  To do so, uncomment the
// CONFIG_SHAKE_DETECTOR flag here, then change the shake configuration
// parameters by the function configShakeDetector().

// Define this to produce DSF data for logging
// #define DSF_OUTPUT

// Define this to perform fimware update at startup.
// #define PERFORM_DFU

// Define this to configure the shake detector
// #define CONFIG_SHAKE_DETECTOR

// ------------------------------------------------------------------------

// Sensor Application
#include <stdio.h>
#include <string.h>

#include "demo_app.h"
#include "button.h"

#include "sh2.h"
#include "sh2_util.h"
#include "euler.h"
#include "sh2_err.h"
#include "sh2_SensorValue.h"
#include "sh2_hal_init.h"

#ifdef PERFORM_DFU
#include "dfu.h"
#endif

#define FIX_Q(n, x) ((int32_t)(x * (float)(1 << n)))

// --- Private data ---------------------------------------------------

sh2_ProductIds_t prodIds;

sh2_Hal_t *pSh2Hal = 0;

bool resetOccurred = false;

// --- Private methods ----------------------------------------------

#ifdef CONFIG_SHAKE_DETECTOR

// Configuration of shake detector
#define MIN_SHAKE_TIME_US (50000)      // 50ms min shake time
#define MAX_SHAKE_TIME_US (400000)     // 400ms max shake time
#define SHAKE_THRESHOLD FIX_Q(26, 0.5) // m/s^2 threshold
#define SHAKE_COUNT (3)                // 2 direction changes constitute a shake
#define ENABLE_FLAGS (0x00000007)      // X, Y and Z axes enabled.

static void configShakeDetector(void)
{
    uint32_t frsData[5];

    frsData[0] = MIN_SHAKE_TIME_US;
    frsData[1] = MAX_SHAKE_TIME_US;
    frsData[2] = SHAKE_THRESHOLD;
    frsData[3] = SHAKE_COUNT;
    frsData[4] = ENABLE_FLAGS;

    int status = sh2_setFrs(SHAKE_DETECT_CONFIG, frsData, 5);
    if (status < 0) {
        printf("Configure shake detector failed: %d\n", status);
    }
}

#endif

static void delayUs(uint32_t t)
{
    uint32_t now_us = pSh2Hal->getTimeUs(pSh2Hal);
    uint32_t start_us = now_us;

    while (t > (now_us - start_us))
    {
        now_us = pSh2Hal->getTimeUs(pSh2Hal);
    }
}

// Configure one sensor to produce periodic reports
static void startReports()
{
    int status;

    // Each entry of sensorConfig[] represents one sensor to be configured in the loop below
    static const struct {
        int sensorId;
        sh2_SensorConfig_t config;
    } sensorConfig[] =
    {
        // Game Rotation Vector, 100Hz
        {SH2_GAME_ROTATION_VECTOR, {.reportInterval_us = 10000}},

        // Stability Detector, 100 Hz, changeSensitivityEnabled
        // {SH2_STABILITY_DETECTOR, {.reportInterval_us = 10000, .changeSensitivityEnabled = true}},

        // Raw accel, 100 Hz
        // {SH2_RAW_ACCELEROMETER, {.reportInterval_us = 10000}},

        // Raw gyroscope, 100 Hz
        // {SH2_RAW_GYROSCOPE, {.reportInterval_us = 10000}},

        // Rotation Vector, 100 Hz
        // {SH2_ROTATION_VECTOR, {.reportInterval_us = 10000}},

        // Gyro Integrated Rotation Vector, 100 Hz
        // {SH2_GYRO_INTEGRATED_RV, {.reportInterval_us = 10000}},

        // Motion requests for Interactive Zero Reference Offset cal
        // {SH2_IZRO_MOTION_REQUEST, {.reportInterval_us = 10000}},

        // Shake detector
        // {SH2_SHAKE_DETECTOR, {.reportInterval_us = 10000}},
    };

    for (int n = 0; n < ARRAY_LEN(sensorConfig); n++)
    {
        int sensorId = sensorConfig[n].sensorId;

        status = sh2_setSensorConfig(sensorId, &sensorConfig[n].config);
        if (status != 0) {
            printf("Error while enabling sensor %d\n", sensorId);
        }
    }
}

// Handle non-sensor events from the sensor hub
static void eventHandler(void * cookie, sh2_AsyncEvent_t *pEvent)
{
    // If we see a reset, set a flag so that sensors will be reconfigured.
    if (pEvent->eventId == SH2_RESET) {
        resetOccurred = true;
    }
    else if (pEvent->eventId == SH2_SHTP_EVENT) {
        printf("EventHandler  id:SHTP, %d\n", pEvent->shtpEvent);
    }
    else if (pEvent->eventId == SH2_GET_FEATURE_RESP) {
        // printf("EventHandler Sensor Config, %d\n", pEvent->sh2SensorConfigResp.sensorId);
    }
    else {
        printf("EventHandler, unknown event Id: %d\n", pEvent->eventId);
    }
}

#ifdef DSF_OUTPUT
// Print headers for DSF format output
static void printDsfHeaders(void)
{
    printf("+%d TIME[x]{s}, SAMPLE_ID[x]{samples}, ANG_POS_GLOBAL[rijk]{quaternion}, ANG_POS_ACCURACY[x]{rad}\n",
           SH2_ROTATION_VECTOR);
    printf("+%d TIME[x]{s}, SAMPLE_ID[x]{samples}, GAME_ROTATION_VECTOR[rijk]{quaternion}\n",
           SH2_GAME_ROTATION_VECTOR);
    printf("+%d TIME[x]{s}, SAMPLE_ID[x]{samples}, RAW_ACCELEROMETER[xyz]{adc units}\n",
           SH2_RAW_ACCELEROMETER);
    printf("+%d TIME[x]{s}, SAMPLE_ID[x]{samples}, RAW_MAGNETOMETER[xyz]{adc units}\n",
           SH2_RAW_MAGNETOMETER);
    printf("+%d TIME[x]{s}, SAMPLE_ID[x]{samples}, RAW_GYROSCOPE[xyz]{adc units}\n",
           SH2_RAW_GYROSCOPE);
    printf("+%d TIME[x]{s}, SAMPLE_ID[x]{samples}, ACCELEROMETER[xyz]{m/s^2}\n",
           SH2_ACCELEROMETER);
    printf("+%d TIME[x]{s}, SAMPLE_ID[x]{samples}, MAG_FIELD[xyz]{uTesla}, STATUS[x]{enum}\n",
           SH2_MAGNETIC_FIELD_CALIBRATED);
    printf("+%d TIME[x]{s}, ANG_VEL_GYRO_RV[xyz]{rad/s}, ANG_POS_GYRO_RV[wxyz]{quaternion}\n",
           SH2_GYRO_INTEGRATED_RV);
}
#endif


#ifdef DSF_OUTPUT
// Print a sensor event as a DSF record
static void printDsf(const sh2_SensorEvent_t * event)
{
    float t, r, i, j, k, acc_rad;
    float angVelX, angVelY, angVelZ;
    static uint32_t lastSequence[SH2_MAX_SENSOR_ID+1];  // last sequence number for each sensor
    sh2_SensorValue_t value;

    // Convert event to value
    sh2_decodeSensorEvent(&value, event);

    // Compute new sample_id
    uint8_t deltaSeq = value.sequence - (lastSequence[value.sensorId] & 0xFF);
    lastSequence[value.sensorId] += deltaSeq;

    // Get time as float
    t = value.timestamp / 1000000.0;

    switch (value.sensorId) {
        case SH2_RAW_ACCELEROMETER:
            printf(".%d %0.6f, %d, %d, %d, %d\n",
                   SH2_RAW_ACCELEROMETER,
                   t,
                   lastSequence[value.sensorId],
                   value.un.rawAccelerometer.x,
                   value.un.rawAccelerometer.y,
                   value.un.rawAccelerometer.z);
            break;
        
        case SH2_RAW_MAGNETOMETER:
            printf(".%d %0.6f, %d, %d, %d, %d\n",
                   SH2_RAW_MAGNETOMETER,
                   t,
                   lastSequence[value.sensorId],
                   value.un.rawMagnetometer.x,
                   value.un.rawMagnetometer.y,
                   value.un.rawMagnetometer.z);
            break;
        
        case SH2_RAW_GYROSCOPE:
            printf(".%d %0.6f, %d, %d, %d, %d\n",
                   SH2_RAW_GYROSCOPE,
                   t,
                   lastSequence[value.sensorId],
                   value.un.rawGyroscope.x,
                   value.un.rawGyroscope.y,
                   value.un.rawGyroscope.z);
            break;

        case SH2_MAGNETIC_FIELD_CALIBRATED:
            printf(".%d %0.6f, %d, %0.6f, %0.6f, %0.6f, %u\n",
                   SH2_MAGNETIC_FIELD_CALIBRATED,
                   t,
                   lastSequence[value.sensorId],
                   value.un.magneticField.x,
                   value.un.magneticField.y,
                   value.un.magneticField.z,
                   value.status & 0x3
                );
            break;
        
        case SH2_ACCELEROMETER:
            printf(".%d %0.6f, %d, %0.6f, %0.6f, %0.6f\n",
                   SH2_ACCELEROMETER,
                   t,
                   lastSequence[value.sensorId],
                   value.un.accelerometer.x,
                   value.un.accelerometer.y,
                   value.un.accelerometer.z);
            break;
        
        case SH2_ROTATION_VECTOR:
            r = value.un.rotationVector.real;
            i = value.un.rotationVector.i;
            j = value.un.rotationVector.j;
            k = value.un.rotationVector.k;
            acc_rad = value.un.rotationVector.accuracy;
            printf(".%d %0.6f, %d, %0.6f, %0.6f, %0.6f, %0.6f, %0.6f\n",
                   SH2_ROTATION_VECTOR,
                   t,
                   lastSequence[value.sensorId],
                   r, i, j, k,
                   acc_rad);
            break;
        
        case SH2_GAME_ROTATION_VECTOR:
            r = value.un.gameRotationVector.real;
            i = value.un.gameRotationVector.i;
            j = value.un.gameRotationVector.j;
            k = value.un.gameRotationVector.k;
            printf(".%d %0.6f, %d, %0.6f, %0.6f, %0.6f, %0.6f\n",
                   SH2_GAME_ROTATION_VECTOR,
                   t,
                   lastSequence[value.sensorId],
                   r, i, j, k);
            break;
            
        case SH2_GYRO_INTEGRATED_RV:
            angVelX = value.un.gyroIntegratedRV.angVelX;
            angVelY = value.un.gyroIntegratedRV.angVelY;
            angVelZ = value.un.gyroIntegratedRV.angVelZ;
            r = value.un.gyroIntegratedRV.real;
            i = value.un.gyroIntegratedRV.i;
            j = value.un.gyroIntegratedRV.j;
            k = value.un.gyroIntegratedRV.k;
            printf(".%d %0.6f, %0.6f, %0.6f, %0.6f, %0.6f, %0.6f, %0.6f, %0.6f\n",
                   SH2_GYRO_INTEGRATED_RV,
                   t,
                   angVelX, angVelY, angVelZ,
                   r, i, j, k);
            break;
        default:
            printf("Unknown sensor: %d\n", value.sensorId);
            break;
    }
}
#endif


#ifndef DSF_OUTPUT
// Read product ids with version info from sensor hub and print them
static void reportProdIds(void)
{
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

        // Wait a bit so we don't overflow the console output.
        delayUs(10000);
    }
}
#endif

#ifndef DSF_OUTPUT
// Print a sensor event to the console
static void printEvent(const sh2_SensorEvent_t * event)
{
    int rc;
    sh2_SensorValue_t value;
    float scaleRadToDeg = 180.0 / 3.14159265358;
    float r, i, j, k, acc_deg, x, y, z;
    float t;
    static int skip = 0;

    rc = sh2_decodeSensorEvent(&value, event);
    if (rc != SH2_OK) {
        printf("Error decoding sensor event: %d\n", rc);
        return;
    }

    t = value.timestamp / 1000000.0;  // time in seconds.
    switch (value.sensorId) {
        case SH2_RAW_ACCELEROMETER:
            printf("%8.4f Raw acc: %d %d %d time_us:%d\n",
                   (double)t,
                   value.un.rawAccelerometer.x,
                   value.un.rawAccelerometer.y,
                   value.un.rawAccelerometer.z,
                   value.un.rawAccelerometer.timestamp);
            break;

        case SH2_ACCELEROMETER:
            printf("%8.4f Acc: %f %f %f\n",
                   (double)t,
                   (double)value.un.accelerometer.x,
                   (double)value.un.accelerometer.y,
                   (double)value.un.accelerometer.z);
            break;
            
        case SH2_RAW_GYROSCOPE:
            printf("%8.4f Raw gyro: x:%d y:%d z:%d temp:%d time_us:%d\n",
                   (double)t,
                   value.un.rawGyroscope.x,
                   value.un.rawGyroscope.y,
                   value.un.rawGyroscope.z,
                   value.un.rawGyroscope.temperature,
                   value.un.rawGyroscope.timestamp);
            break;
            
        case SH2_ROTATION_VECTOR:
            r = value.un.rotationVector.real;
            i = value.un.rotationVector.i;
            j = value.un.rotationVector.j;
            k = value.un.rotationVector.k;
            acc_deg = scaleRadToDeg * 
                value.un.rotationVector.accuracy;
            printf("%8.4f Rotation Vector: "
                   "r:%0.6f i:%0.6f j:%0.6f k:%0.6f (acc: %0.6f deg)\n",
                   (double)t,
                   (double)r, (double)i, (double)j, (double)k, (double)acc_deg);
            break;
        case SH2_GAME_ROTATION_VECTOR:
            r = value.un.gameRotationVector.real;
            i = value.un.gameRotationVector.i;
            j = value.un.gameRotationVector.j;
            k = value.un.gameRotationVector.k;
            printf("%8.4f GRV: "
                   "r:%0.6f i:%0.6f j:%0.6f k:%0.6f\n",
                   (double)t,
                   (double)r, (double)i, (double)j, (double)k);
            break;
        case SH2_GYROSCOPE_CALIBRATED:
            x = value.un.gyroscope.x;
            y = value.un.gyroscope.y;
            z = value.un.gyroscope.z;
            printf("%8.4f GYRO: "
                   "x:%0.6f y:%0.6f z:%0.6f\n",
                   (double)t,
                   (double)x, (double)y, (double)z);
            break;
        case SH2_GYROSCOPE_UNCALIBRATED:
            x = value.un.gyroscopeUncal.x;
            y = value.un.gyroscopeUncal.y;
            z = value.un.gyroscopeUncal.z;
            printf("%8.4f GYRO_UNCAL: "
                   "x:%0.6f y:%0.6f z:%0.6f\n",
                   (double)t,
                   (double)x, (double)y, (double)z);
            break;
        case SH2_GYRO_INTEGRATED_RV:
            // These come at 1kHz, too fast to print all of them.
            // So only print every 10th one
            skip++;
            if (skip == 10) {
                skip = 0;
                r = value.un.gyroIntegratedRV.real;
                i = value.un.gyroIntegratedRV.i;
                j = value.un.gyroIntegratedRV.j;
                k = value.un.gyroIntegratedRV.k;
                x = value.un.gyroIntegratedRV.angVelX;
                y = value.un.gyroIntegratedRV.angVelY;
                z = value.un.gyroIntegratedRV.angVelZ;
                printf("%8.4f Gyro Integrated RV: "
                       "r:%0.6f i:%0.6f j:%0.6f k:%0.6f x:%0.6f y:%0.6f z:%0.6f\n",
                       (double)t,
                       (double)r, (double)i, (double)j, (double)k,
                       (double)x, (double)y, (double)z);
            }
            break;
        case SH2_IZRO_MOTION_REQUEST:
            printf("IZRO Request: intent:%d, request:%d\n",
                   value.un.izroRequest.intent,
                   value.un.izroRequest.request);
            break;
        case SH2_SHAKE_DETECTOR:
            printf("Shake Axis: %c%c%c\n", 
                   (value.un.shakeDetector.shake & SHAKE_X) ? 'X' : '.',
                   (value.un.shakeDetector.shake & SHAKE_Y) ? 'Y' : '.',
                   (value.un.shakeDetector.shake & SHAKE_Z) ? 'Z' : '.');

            break;
        case SH2_STABILITY_CLASSIFIER:
            printf("Stability Classification: %d\n",
                   value.un.stabilityClassifier.classification);
            break;
        case SH2_STABILITY_DETECTOR:
            printf("Stability Detector: %d\n",
                   value.un.stabilityDetector.stability);
            break;
        default:
            printf("Unknown sensor: %d\n", value.sensorId);
            break;
    }
}
#endif

// Handle sensor events.
static void sensorHandler(void * cookie, sh2_SensorEvent_t *pEvent)
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

    printf("\n\nCEVA SH2 Sensor Hub Demo.\n\n");

#ifdef PERFORM_DFU
    printf("DFU completes in 10-25 seconds in most configurations.\n");
    printf("It can take up to 240 seconds with 9600 baud UART.\n");
    printf("DFU Process started.\n");
    status = dfu();
    if (status == SH2_OK) {
        printf("DFU completed successfully.\n");
    }
    else {
        printf("DFU failed.  Error=%d.\n", status);
        if (status == SH2_ERR_BAD_PARAM) {
            printf("Is the firmware image valid?\n");
        }
    }
#endif

    // Create HAL instance
    pSh2Hal = sh2_hal_init();

    // Open SH2 interface (also registers non-sensor event handler.)
    status = sh2_open(pSh2Hal, eventHandler, NULL);
    if (status != SH2_OK) {
        printf("Error, %d, from sh2_open.\n", status);
    }

    // Register sensor listener
    sh2_setSensorCallback(sensorHandler, NULL);

#ifdef DSF_OUTPUT
    // Print DSF file headers
    printDsfHeaders();
#else
    // Read and display device product ids
    reportProdIds();
#endif

    // resetOccurred would have been set earlier.
    // We can reset it since we are starting the sensor reports now.
    resetOccurred = false;

#ifdef CONFIG_SHAKE_DETECTOR
    // Configure shake detector
    // (The configuration will be permantently stored in flash but
    // doesn't take effect until the system is restarted.)
    configShakeDetector();
#endif

    // Start the flow of sensor reports
    startReports();
}

// This must be called periodically.  (The demo main calls it continuously in a loop.)
// It calls sh2_service to keep data flowing between host and sensor hub.
void demo_service(void)
{
    uint32_t now = pSh2Hal->getTimeUs(pSh2Hal);

    if (resetOccurred) {
        // Restart the flow of sensor reports
        resetOccurred = false;
        startReports();
    }

    // Service the sensor hub.
    // Sensor reports and event processing handled by callbacks.
    sh2_service();

    // Handle button presses.
    button_poll(now);
}



