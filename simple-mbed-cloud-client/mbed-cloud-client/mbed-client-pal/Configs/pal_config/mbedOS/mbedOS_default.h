/*******************************************************************************
 * Copyright 2016-2018 ARM Ltd.
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 *******************************************************************************/

#ifndef PAL_MBEDOS_CONFIGURATION_H_

#include "cmsis_os.h"
#include "mbed_version.h"

#if (defined(MBED_DEBUG) && !defined(DEBUG))
    #define DEBUG
#endif

#ifndef PAL_RTOS_WAIT_FOREVER
    #define PAL_RTOS_WAIT_FOREVER osWaitForever
#endif

#ifndef PAL_NUMBER_OF_PARTITIONS
    #define PAL_NUMBER_OF_PARTITIONS 1
#endif


#ifndef PAL_FS_MOUNT_POINT_PRIMARY
    #if (MBED_VERSION >= 51000)  // 5.10 or above
        #define PAL_FS_MOUNT_POINT_PRIMARY    "/default"           //!< User should change this for the his working folder
    #else
        #define PAL_FS_MOUNT_POINT_PRIMARY    "/sd"
    #endif
#endif

#ifndef PAL_FS_MOUNT_POINT_SECONDARY
    #if (PAL_NUMBER_OF_PARTITIONS == 2)
        #if (MBED_VERSION >= 51000)  // 5.10 or above
            #define PAL_FS_MOUNT_POINT_SECONDARY    "/default2"
        #else
            #define PAL_FS_MOUNT_POINT_SECONDARY    "/sd2"
        #endif
    #else
        #define PAL_FS_MOUNT_POINT_SECONDARY PAL_FS_MOUNT_POINT_PRIMARY      //!< User should change this for the his working folder
    #endif
#endif

#ifndef PAL_NUM_OF_THREAD_INSTANCES
    #define PAL_NUM_OF_THREAD_INSTANCES 1
#endif

#ifndef PAL_MAX_SEMAPHORE_COUNT
    #define PAL_MAX_SEMAPHORE_COUNT 1024
#endif

#ifndef PAL_USE_INTERNAL_FLASH
    #define PAL_USE_INTERNAL_FLASH  1
#endif

#ifndef PAL_INT_FLASH_NUM_SECTIONS
    #define PAL_INT_FLASH_NUM_SECTIONS 2
#endif

#ifndef PAL_USE_HW_ROT
    #define PAL_USE_HW_ROT 0
#endif

#ifndef PAL_US_HW_RTC
    #define PAL_USE_HW_RTC 0
#endif

// DEVICE_TRNG is a define that mbed-os defines for every board that is configured to have a TRNG
// mbedtls uses this define when gathering entropy 
#if defined(DEVICE_TRNG) && !defined(PAL_USE_HW_TRNG)
    #define PAL_USE_HW_TRNG 1
#elif !defined(DEVICE_TRNG) && !defined(PAL_USE_HW_TRNG)
    #define PAL_USE_HW_TRNG 0
#endif

#endif /* PAL_MBEDOS_CONFIGURATION_H_ */
