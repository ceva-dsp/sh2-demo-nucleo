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

/**
 * @file HcBin.h
 * @author David Wheeler
 * @date 10 February 2016
 * @brief API Definition for HcBin objects (Hillcrest Binary Files).
 *
 * This interface definition is intended to represent such files in a
 * way that supports compression and/or streaming data via a serial
 * interface.
 *
 *
 */

#ifndef HCBIN_H
#define HCBIN_H

#include <stdint.h>

/**
 * Abstract Data Type for Hillcrest Binary Files.  
 * 
 * Function pointers provide open, close and other operations.
 * Note restrictions on calling order among these operations.  Open
 * must be called first, followed by calls to getMeta, getAppLen 
 * and getPacketLen.  Only then should getAppData be called,
 * starting from offset 0 and increasing continuously.  close may
 * be called at any time to terminate the session.
 */

typedef struct HcBin_s {
    /** 
     * Open a session with the HcBin object.
     * @return 0 on success, non-zero on error.
     */
    int (*open)(void);

    /**
     * Close a session with the HcBin object.
     * @return 0 on success, non-zero on error.
     */
    int (*close)(void);

    /**
     * Get metadata value for a given key.
     * @param key The key (string) to look up.
     * @return The value (string) associated with the key or NULL.
     */
    const char * (*getMeta)(const char * key);

    /**
     * Get length of application code.
     * @return application length in bytes.
     */
    uint32_t (*getAppLen)(void);

    /**
     * Get preferred packet length for getAppData calls.
     * During DFU protocol, data is read in chunks, via the getAppData()
     * function.  If the HcBin implementation has a preferred packet
     * len (e.g. due to serial line protocol or decompression method)
     * it should return that value through this function.
     * If the HcBin implementation places no restriction on len in
     * the getAppData() method, it should return 0
     * @return preferred packet len or 0, if no preferred value.
     */
    uint32_t (*getPacketLen)(void);

    /**
     * Get App Data.
     * Copies len bytes of application data, starting from offset, into 
     * buffer pointed to by packet.  After first call to getAppData, the
     * user should not access other functions except close.  (This is 
     * to facilitate streaming HcBin data via a serial protocol.)
     * @return 0 on success, Non-zero on error.
     */
    int (*getAppData)(uint8_t *packet, uint32_t offset, uint32_t len);
} HcBin_t;

#endif
