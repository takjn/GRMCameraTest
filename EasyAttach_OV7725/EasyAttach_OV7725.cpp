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
#include "mbed.h"
#include "rtos.h"
#include "EasyAttach_OV7725.h"

static DisplayBase::graphics_error_t camera_init(DisplayBase& Display, uint16_t cap_width, uint16_t cap_height) {
  DisplayBase::graphics_error_t error;
  DisplayBase::video_input_sel_t video_input_sel;

  // defined(TARGET_GR_MANGO)
  PinName cmos_camera_pin[11] = {
      /* data pin */
      P7_4, P7_5, P9_7, P9_6, P9_5, P9_4, P9_3, P9_2,
      /* control pin */
      P7_0,       /* VIO_CLK   */
      P7_1,       /* VIO_VD */
      P7_3        /* VIO_HD */
  };
  DigitalOut pwdn(P1_3);
  DigitalOut rstb(P1_4);

  pwdn = 0;
  rstb = 0;
  ThisThread::sleep_for(10 + 1);
  rstb = 1;
  ThisThread::sleep_for(1 + 1);

  video_input_sel = DisplayBase::INPUT_SEL_EXT;
  error = Display.Graphics_Dvinput_Port_Init(cmos_camera_pin, 11);

  if( error != DisplayBase::GRAPHICS_OK) {
      printf("Line %d, error %d\n", __LINE__, error);
      return error;
  }

  OV7725_config camera_cfg;

  DisplayBase::video_ext_in_config_t ext_in_config;

  camera_cfg.Initialise();
  camera_cfg.SetExtInConfig(&ext_in_config);
  if (cap_width != 0) {
      ext_in_config.cap_width  = cap_width;                             /* Capture width */
  }
  if (cap_height != 0) {
      ext_in_config.cap_height = cap_height;                            /* Capture heigh */
  }

  error = Display.Graphics_Video_init(video_input_sel, &ext_in_config);
  if( error != DisplayBase::GRAPHICS_OK ) {
      printf("Line %d, error %d\n", __LINE__, error);
      return error;
  }

  return DisplayBase::GRAPHICS_OK;
}

DisplayBase::graphics_error_t EasyAttach_OV7725_Init(DisplayBase& Display, uint16_t cap_width, uint16_t cap_height) {
    return camera_init(Display, cap_width, cap_height);
}

DisplayBase::graphics_error_t EasyAttach_OV7725_CameraStart(DisplayBase& Display, DisplayBase::video_input_channel_t channel) {
  DisplayBase::graphics_error_t error;

  /* Video write process start */
  error = Display.Video_Start(channel);
  if (error != DisplayBase::GRAPHICS_OK) {
      printf("Line %d, error %d\n", __LINE__, error);
      return error;
  }

  /* Video write process stop */
  error = Display.Video_Stop(channel);
  if (error != DisplayBase::GRAPHICS_OK) {
      printf("Line %d, error %d\n", __LINE__, error);
      return error;
  }

  /* Video write process start */
  error = Display.Video_Start(channel);
  if (error != DisplayBase::GRAPHICS_OK) {
      printf("Line %d, error %d\n", __LINE__, error);
      return error;
  }
  return DisplayBase::GRAPHICS_OK;
}
