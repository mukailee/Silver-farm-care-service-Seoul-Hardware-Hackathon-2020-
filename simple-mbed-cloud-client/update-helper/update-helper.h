// ----------------------------------------------------------------------------
// Copyright 2016-2018 ARM Ltd.
//
// SPDX-License-Identifier: Apache-2.0
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.
// ----------------------------------------------------------------------------

#ifndef UPDATE_UI_EXAMPLE_H
#define UPDATE_UI_EXAMPLE_H

#include "mbed-cloud-client/MbedCloudClient.h"

#ifdef MBED_CLOUD_CLIENT_SUPPORT_UPDATE

#ifdef ARM_UPDATE_CLIENT_VERSION_VALUE
#if ARM_UPDATE_CLIENT_VERSION_VALUE > 101000
/**
 * @brief Set the cloud client instance for the update UI to use
 * @param[in] client pointer to the cloud client instance
 */
void update_helper_set_cloud_client(MbedCloudClient* client);

/**
 * @brief Function for authorizing firmware downloads and reboots.
 * @param request The request under consideration.
 */
void update_authorize(int32_t request);
#endif
#endif

/**
 * @brief Callback function for reporting the firmware download progress.
 * @param progress Received bytes.
 * @param total Total amount of bytes to be received.
 */
void update_progress(uint32_t progress, uint32_t total);


#endif // MBED_CLOUD_CLIENT_SUPPORT_UPDATE

#endif // UPDATE_UI_EXAMPLE_H
