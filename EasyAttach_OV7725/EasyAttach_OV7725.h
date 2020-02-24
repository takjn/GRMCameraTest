/* Copyright (c) 2017 dkato
 * SPDX-License-Identifier: Apache-2.0
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
 */
#ifndef EASY_ATTACH_OV7725_H
#define EASY_ATTACH_OV7725_H

#include "DisplayBace.h"
#include "OV7725_config.h"

//  #define MBED_CONF_APP_CAMERA_TYPE    CAMERA_OV7725
//  #define CAMERA_MODULE             MODULE_VDC

extern DisplayBase::graphics_error_t EasyAttach_OV7725_Init(
    DisplayBase& Display,
    uint16_t cap_width = 0,
    uint16_t cap_height = 0
);

extern DisplayBase::graphics_error_t EasyAttach_OV7725_CameraStart(
    DisplayBase& Display,
    DisplayBase::video_input_channel_t channel = DisplayBase::VIDEO_INPUT_CHANNEL_0
);

// extern void EasyAttach_SetTypicalBacklightVol(float typ_vol);

// extern void EasyAttach_LcdBacklight(bool type = true);

// extern void EasyAttach_LcdBacklight(float value);

#endif
