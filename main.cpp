/*******************************************************************************
* DISCLAIMER
* This software is supplied by Renesas Electronics Corporation and is only
* intended for use with Renesas products. No other uses are authorized. This
* software is owned by Renesas Electronics Corporation and is protected under
* all applicable laws, including copyright laws.
* THIS SOFTWARE IS PROVIDED "AS IS" AND RENESAS MAKES NO WARRANTIES REGARDING
* THIS SOFTWARE, WHETHER EXPRESS, IMPLIED OR STATUTORY, INCLUDING BUT NOT
* LIMITED TO WARRANTIES OF MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE
* AND NON-INFRINGEMENT. ALL SUCH WARRANTIES ARE EXPRESSLY DISCLAIMED.
* TO THE MAXIMUM EXTENT PERMITTED NOT PROHIBITED BY LAW, NEITHER RENESAS
* ELECTRONICS CORPORATION NOR ANY OF ITS AFFILIATED COMPANIES SHALL BE LIABLE
* FOR ANY DIRECT, INDIRECT, SPECIAL, INCIDENTAL OR CONSEQUENTIAL DAMAGES FOR
* ANY REASON RELATED TO THIS SOFTWARE, EVEN IF RENESAS OR ITS AFFILIATES HAVE
* BEEN ADVISED OF THE POSSIBILITY OF SUCH DAMAGES.
* Renesas reserves the right, without notice, to make changes to this software
* and to discontinue the availability of this software. By using this software,
* you agree to the additional terms and conditions found by accessing the
* following link:
* http://www.renesas.com/disclaimer
*
* Copyright (C) 2019 Renesas Electronics Corporation. All rights reserved.
*******************************************************************************/

#if !defined(TARGET_RZ_A2XX)
#error "DRP and MIPI are not supported."
#endif
#if MBED_CONF_APP_CAMERA_TYPE != CAMERA_RASPBERRY_PI
#error Please set the value of "camera-type" in "mbed_app.json" to "CAMERA_RASPBERRY_PI" and build.
#endif

#include "mbed.h"
#include "EasyAttach_CameraAndLCD.h"
#include "dcache-control.h"
#include "AsciiFont.h"
#include "r_dk2_if.h"
#include "r_drp_simple_isp.h"
#include "r_drp_bayer2grayscale.h"
#include "r_drp_median_blur.h"
#include "r_drp_canny_calculate.h"
#include "r_drp_canny_hysterisis.h"
#include "r_drp_resize_bilinear.h"

#define DRP_FLG_TILE_ALL       (R_DK2_TILE_0 | R_DK2_TILE_1 | R_DK2_TILE_2 | R_DK2_TILE_3 | R_DK2_TILE_4 | R_DK2_TILE_5)
#define DRP_FLG_CAMER_IN       (0x00000100)

/*! Frame buffer stride: Frame buffer stride should be set to a multiple of 32 or 128
    in accordance with the frame buffer burst transfer mode. */
#define VIDEO_PIXEL_HW         (640)
#define VIDEO_PIXEL_VW         (480)

#define DATA_SIZE_PER_PIC      (1u)
#define FRAME_BUFFER_STRIDE    (((VIDEO_PIXEL_HW * DATA_SIZE_PER_PIC) + 31u) & ~31u)
#define FRAME_BUFFER_STRIDE_2  (((VIDEO_PIXEL_HW * 2) + 31u) & ~31u)
#define FRAME_BUFFER_HEIGHT    (VIDEO_PIXEL_VW)

// Result window parameter and frame buffer
#define WINDOW_SCALE           4
#define WINDOW_COUNT           2
#define FRAME_BUFFER_STRIDE_RESULT    (((VIDEO_PIXEL_HW / WINDOW_SCALE * DATA_SIZE_PER_PIC) + 31u) & ~31u)
#define FRAME_BUFFER_HEIGHT_RESULT    (FRAME_BUFFER_HEIGHT / WINDOW_SCALE)
static uint8_t fbuf_result[FRAME_BUFFER_STRIDE * FRAME_BUFFER_HEIGHT_RESULT * WINDOW_COUNT]__attribute((aligned(32)));

static DisplayBase Display;
static uint8_t fbuf_bayer[FRAME_BUFFER_STRIDE * FRAME_BUFFER_HEIGHT]__attribute((aligned(128)));
static uint8_t fbuf_yuv[FRAME_BUFFER_STRIDE_2 * FRAME_BUFFER_HEIGHT]__attribute((aligned(32)));
static uint8_t fbuf_work0[FRAME_BUFFER_STRIDE * FRAME_BUFFER_HEIGHT]__attribute((aligned(32)));
static uint8_t fbuf_work1[FRAME_BUFFER_STRIDE * FRAME_BUFFER_HEIGHT]__attribute((aligned(32)));
static uint8_t fbuf_overlay[FRAME_BUFFER_STRIDE * FRAME_BUFFER_HEIGHT]__attribute((section("NC_BSS"),aligned(32)));
static uint8_t drp_work_buf[((FRAME_BUFFER_STRIDE * ((FRAME_BUFFER_HEIGHT / 3) + 2)) * 2) * 3]__attribute((section("NC_BSS")));
static uint8_t nc_memory[512] __attribute((section("NC_BSS")));
static uint8_t drp_lib_id[R_DK2_TILE_NUM] = {0};
static Thread drpTask(osPriorityHigh);

static uint8_t ram_drp_lib_bayer2grayscale[sizeof(g_drp_lib_bayer2grayscale)]__attribute((aligned(32)));
static uint8_t ram_drp_lib_median_blur[sizeof(g_drp_lib_median_blur)]__attribute((aligned(32)));
static uint8_t ram_drp_lib_canny_calculate[sizeof(g_drp_lib_canny_calculate)]__attribute((aligned(32)));
static uint8_t ram_drp_lib_canny_hysterisis[sizeof(g_drp_lib_canny_hysterisis)]__attribute((aligned(32)));
static uint8_t ram_drp_lib_resize_bilinear[sizeof(g_drp_lib_resize_bilinear)]__attribute((aligned(32)));
// static uint8_t ram_drp_lib_simple_isp_bayer2yuv_6[sizeof(g_drp_lib_simple_isp_bayer2yuv_6)]__attribute((aligned(32)));
static r_drp_simple_isp_t param_isp __attribute((section("NC_BSS")));

static const uint32_t clut_data_resut[] = {0x00000000, 0xff00ff00};  // ARGB8888

static void IntCallbackFunc_Vfield(DisplayBase::int_type_t int_type) {
    drpTask.flags_set(DRP_FLG_CAMER_IN);
}

static void cb_drp_finish(uint8_t id) {
    uint32_t tile_no;
    uint32_t set_flgs = 0;

    // Change the operation state of the DRP library notified by the argument to finish
    for (tile_no = 0; tile_no < R_DK2_TILE_NUM; tile_no++) {
        if (drp_lib_id[tile_no] == id) {
            set_flgs |= (1 << tile_no);
        }
    }
    drpTask.flags_set(set_flgs);
}

void Start_Video_Camera(void) {
    // Video capture setting (progressive form fixed)
    Display.Video_Write_Setting(
        DisplayBase::VIDEO_INPUT_CHANNEL_0,
        DisplayBase::COL_SYS_NTSC_358,
        (void *)fbuf_bayer,
        FRAME_BUFFER_STRIDE,
        DisplayBase::VIDEO_FORMAT_RAW8,
        DisplayBase::WR_RD_WRSWA_NON,
        VIDEO_PIXEL_VW,
        VIDEO_PIXEL_HW
    );
    EasyAttach_CameraStart(Display, DisplayBase::VIDEO_INPUT_CHANNEL_0);
}

static void Start_LCD_Display(void) {
    DisplayBase::rect_t rect;
    DisplayBase::clut_t clut_param;

    rect.vs = 0;
    rect.vw = VIDEO_PIXEL_VW;
    rect.hs = 0;
    rect.hw = VIDEO_PIXEL_HW;
    Display.Graphics_Read_Setting(
        DisplayBase::GRAPHICS_LAYER_0,
        (void *)fbuf_yuv,
        FRAME_BUFFER_STRIDE_2,
        DisplayBase::GRAPHICS_FORMAT_YCBCR422,
        DisplayBase::WR_RD_WRSWA_32_16_8BIT,
        &rect
    );
    Display.Graphics_Start(DisplayBase::GRAPHICS_LAYER_0);

    rect.vs = 0;
    rect.vw = VIDEO_PIXEL_VW/WINDOW_SCALE * 2;
    rect.hs = 0;
    rect.hw = VIDEO_PIXEL_HW/WINDOW_SCALE;
    Display.Graphics_Read_Setting(
        DisplayBase::GRAPHICS_LAYER_2,
        (void *)fbuf_result,
        FRAME_BUFFER_STRIDE_RESULT,
        DisplayBase::GRAPHICS_FORMAT_CLUT8,
        DisplayBase::WR_RD_WRSWA_32_16_8BIT,
        &rect
    );
    Display.Graphics_Start(DisplayBase::GRAPHICS_LAYER_2);

    memset(fbuf_overlay, 0, sizeof(fbuf_overlay));
    clut_param.color_num = sizeof(clut_data_resut) / sizeof(uint32_t);
    clut_param.clut = clut_data_resut;

    rect.vs = 0;
    rect.vw = VIDEO_PIXEL_VW;
    rect.hs = 0;
    rect.hw = VIDEO_PIXEL_HW;
    Display.Graphics_Read_Setting(
        DisplayBase::GRAPHICS_LAYER_3,
        (void *)fbuf_overlay,
        FRAME_BUFFER_STRIDE,
        DisplayBase::GRAPHICS_FORMAT_CLUT8,
        DisplayBase::WR_RD_WRSWA_32_16_8BIT,
        &rect,
        &clut_param
    );
    Display.Graphics_Start(DisplayBase::GRAPHICS_LAYER_3);

    ThisThread::sleep_for(50);
    EasyAttach_LcdBacklight(true);
}

static void drp_task(void) {
    uint32_t idx;
    char str[64];

    AsciiFont ascii_font(fbuf_overlay, VIDEO_PIXEL_HW, VIDEO_PIXEL_VW, FRAME_BUFFER_STRIDE, DATA_SIZE_PER_PIC);

    EasyAttach_Init(Display);
    Start_LCD_Display();
    // Interrupt callback function setting (Field end signal for recording function in scaler 0)
    Display.Graphics_Irq_Handler_Set(DisplayBase::INT_TYPE_S0_VFIELD, 0, IntCallbackFunc_Vfield);
    Start_Video_Camera();

    // Copy to RAM to increase the speed of dynamic loading.
    memcpy(ram_drp_lib_bayer2grayscale, g_drp_lib_bayer2grayscale, sizeof(ram_drp_lib_bayer2grayscale));
    memcpy(ram_drp_lib_median_blur, g_drp_lib_median_blur, sizeof(ram_drp_lib_median_blur));
    memcpy(ram_drp_lib_canny_calculate, g_drp_lib_canny_calculate, sizeof(ram_drp_lib_canny_calculate));
    memcpy(ram_drp_lib_canny_hysterisis, g_drp_lib_canny_hysterisis, sizeof(ram_drp_lib_canny_hysterisis));
    memcpy(ram_drp_lib_resize_bilinear, g_drp_lib_resize_bilinear, sizeof(ram_drp_lib_resize_bilinear));
    // memcpy(ram_drp_lib_simple_isp_bayer2yuv_6, g_drp_lib_simple_isp_bayer2yuv_6, sizeof(ram_drp_lib_simple_isp_bayer2yuv_6));

    R_DK2_Initialize();

    while (true) {
        ThisThread::flags_wait_all(DRP_FLG_CAMER_IN);

        // SimpleIsp bayer2yuv_6
        /* fbuf_bayer -> fbuf_yuv    */
        // R_DK2_Load(ram_drp_lib_simple_isp_bayer2yuv_6,
        R_DK2_Load(g_drp_lib_simple_isp_bayer2yuv_6,
                R_DK2_TILE_0,
                R_DK2_TILE_PATTERN_6, NULL, &cb_drp_finish, drp_lib_id);
        R_DK2_Activate(0, 0);
        memset(&param_isp, 0, sizeof(param_isp));
        param_isp.src    = (uint32_t)fbuf_bayer;
        param_isp.dst    = (uint32_t)fbuf_yuv;
        param_isp.width  = VIDEO_PIXEL_HW;
        param_isp.height = VIDEO_PIXEL_VW;
        param_isp.gain_r = 0x1266;
        param_isp.gain_g = 0x0CB0;
        param_isp.gain_b = 0x1359;
        R_DK2_Start(drp_lib_id[0], (void *)&param_isp, sizeof(r_drp_simple_isp_t));
        ThisThread::flags_wait_all(DRP_FLG_TILE_ALL);
        R_DK2_Unload(0, drp_lib_id);

        // Bayer2Grayscale
        /* fbuf_bayer -> fbuf_work0    */
        R_DK2_Load(
                   ram_drp_lib_bayer2grayscale,
                   R_DK2_TILE_0 | R_DK2_TILE_1 | R_DK2_TILE_2 | R_DK2_TILE_3 | R_DK2_TILE_4 | R_DK2_TILE_5,
                   R_DK2_TILE_PATTERN_1_1_1_1_1_1, NULL, &cb_drp_finish, drp_lib_id);
        R_DK2_Activate(0, 0);
        r_drp_bayer2grayscale_t * param_b2g = (r_drp_bayer2grayscale_t *)nc_memory;
        for (idx = 0; idx < R_DK2_TILE_NUM; idx++) {
            param_b2g[idx].src    = (uint32_t)fbuf_bayer + (VIDEO_PIXEL_HW * (VIDEO_PIXEL_VW / R_DK2_TILE_NUM) * idx);
            param_b2g[idx].dst    = (uint32_t)fbuf_work0 + (VIDEO_PIXEL_HW * (VIDEO_PIXEL_VW / R_DK2_TILE_NUM) * idx);
            param_b2g[idx].width  = VIDEO_PIXEL_HW;
            param_b2g[idx].height = VIDEO_PIXEL_VW / R_DK2_TILE_NUM;
            param_b2g[idx].top    = (idx == 0) ? 1 : 0;
            param_b2g[idx].bottom = (idx == 5) ? 1 : 0;
            R_DK2_Start(drp_lib_id[idx], (void *)&param_b2g[idx], sizeof(r_drp_bayer2grayscale_t));
        }
        ThisThread::flags_wait_all(DRP_FLG_TILE_ALL);
        R_DK2_Unload(0, drp_lib_id);


        // Result 1: Bayer2Grayscale
        // resize and write to the framebuffer for result (Bayer2Grayscale)
        R_DK2_Load(
                   ram_drp_lib_resize_bilinear,
                   R_DK2_TILE_0,
                   R_DK2_TILE_PATTERN_6, NULL, &cb_drp_finish, drp_lib_id);
        R_DK2_Activate(0, 0);
        r_drp_resize_bilinear_t * param_resize_bilinear = (r_drp_resize_bilinear_t *)nc_memory;
        param_resize_bilinear[0].src    = (uint32_t)fbuf_work0;
        param_resize_bilinear[0].dst    = (uint32_t)fbuf_result;
        param_resize_bilinear[0].src_width  = VIDEO_PIXEL_HW;
        param_resize_bilinear[0].src_height = VIDEO_PIXEL_VW;
        param_resize_bilinear[0].dst_width  = VIDEO_PIXEL_HW/WINDOW_SCALE;
        param_resize_bilinear[0].dst_height = VIDEO_PIXEL_VW/WINDOW_SCALE;
        R_DK2_Start(drp_lib_id[0], (void *)&param_resize_bilinear[0], sizeof(r_drp_resize_bilinear_t));
        ThisThread::flags_wait_all(DRP_FLG_TILE_ALL);
        R_DK2_Unload(0, drp_lib_id);


        // MedianBlur
        /* fbuf_work0 -> fbuf_work1    */
        R_DK2_Load(
                   ram_drp_lib_median_blur,
                   R_DK2_TILE_0 | R_DK2_TILE_1 | R_DK2_TILE_2 | R_DK2_TILE_3 | R_DK2_TILE_4 | R_DK2_TILE_5,
                   R_DK2_TILE_PATTERN_1_1_1_1_1_1, NULL, &cb_drp_finish, drp_lib_id);
        R_DK2_Activate(0, 0);
        r_drp_median_blur_t * param_median = (r_drp_median_blur_t *)nc_memory;
        for (idx = 0; idx < R_DK2_TILE_NUM; idx++) {
            param_median[idx].src    = (uint32_t)fbuf_work0 + (VIDEO_PIXEL_HW * (VIDEO_PIXEL_VW / R_DK2_TILE_NUM) * idx);
            param_median[idx].dst    = (uint32_t)fbuf_work1 + (VIDEO_PIXEL_HW * (VIDEO_PIXEL_VW / R_DK2_TILE_NUM) * idx);
            param_median[idx].width  = VIDEO_PIXEL_HW;
            param_median[idx].height = VIDEO_PIXEL_VW / R_DK2_TILE_NUM;
            param_median[idx].top    = (idx == 0) ? 1 : 0;
            param_median[idx].bottom = (idx == 5) ? 1 : 0;
            R_DK2_Start(drp_lib_id[idx], (void *)&param_median[idx], sizeof(r_drp_median_blur_t));
        }
        ThisThread::flags_wait_all(DRP_FLG_TILE_ALL);
        R_DK2_Unload(0, drp_lib_id);


        // CannyCalculate
        /* fbuf_work1 -> fbuf_work0    */
        R_DK2_Load(
                   ram_drp_lib_canny_calculate,
                   R_DK2_TILE_0 | R_DK2_TILE_2 | R_DK2_TILE_4,
                   R_DK2_TILE_PATTERN_2_2_2, NULL, &cb_drp_finish, drp_lib_id);
        R_DK2_Activate(0, 0);
        r_drp_canny_calculate_t * param_canny_cal = (r_drp_canny_calculate_t *)nc_memory;
        for (idx = 0; idx < 3; idx++) {
            param_canny_cal[idx].src    = (uint32_t)fbuf_work1 + (VIDEO_PIXEL_HW * (VIDEO_PIXEL_VW / 3) * idx);
            param_canny_cal[idx].dst    = (uint32_t)fbuf_work0 + (VIDEO_PIXEL_HW * (VIDEO_PIXEL_VW / 3) * idx);
            param_canny_cal[idx].width  = VIDEO_PIXEL_HW;
            param_canny_cal[idx].height = (VIDEO_PIXEL_VW / 3);
            param_canny_cal[idx].top    = ((idx * 2) == 0) ? 1 : 0;
            param_canny_cal[idx].bottom = ((idx * 2) == 4) ? 1 : 0;
            param_canny_cal[idx].work   = (uint32_t)&drp_work_buf[((VIDEO_PIXEL_HW * ((VIDEO_PIXEL_VW / 3) + 2)) * 2) * idx];
            param_canny_cal[idx].threshold_high = 0x28;
            param_canny_cal[idx].threshold_low  = 0x18;
            R_DK2_Start(drp_lib_id[(idx * 2)], (void *)&param_canny_cal[idx], sizeof(r_drp_canny_calculate_t));
        }
        ThisThread::flags_wait_all(DRP_FLG_TILE_ALL);
        R_DK2_Unload(0, drp_lib_id);


        // CannyHysterisis
        /* fbuf_work0 -> fbuf_clat8    */
        R_DK2_Load(
                   ram_drp_lib_canny_hysterisis,
                   R_DK2_TILE_0,
                   R_DK2_TILE_PATTERN_6, NULL, &cb_drp_finish, drp_lib_id);
        R_DK2_Activate(0, 0);
        r_drp_canny_hysterisis_t * param_canny_hyst = (r_drp_canny_hysterisis_t *)nc_memory;
        param_canny_hyst[0].src    = (uint32_t)fbuf_work0;
        param_canny_hyst[0].dst    = (uint32_t)fbuf_work1;
        param_canny_hyst[0].width  = VIDEO_PIXEL_HW;
        param_canny_hyst[0].height = VIDEO_PIXEL_VW;
        param_canny_hyst[0].work   = (uint32_t)drp_work_buf;
        param_canny_hyst[0].iterations = 2;
        R_DK2_Start(drp_lib_id[0], (void *)&param_canny_hyst[0], sizeof(r_drp_canny_hysterisis_t));
        ThisThread::flags_wait_all(DRP_FLG_TILE_ALL);
        R_DK2_Unload(0, drp_lib_id);


        // Result 2: CannyHysterisis
        // resize and write to the framebuffer for result (CannyHysterisis)
        R_DK2_Load(
                   ram_drp_lib_resize_bilinear,
                   R_DK2_TILE_0,
                   R_DK2_TILE_PATTERN_6, NULL, &cb_drp_finish, drp_lib_id);
        R_DK2_Activate(0, 0);
        param_resize_bilinear[0].src    = (uint32_t)fbuf_work1;
        param_resize_bilinear[0].dst    = (uint32_t)fbuf_result+(FRAME_BUFFER_STRIDE_RESULT * FRAME_BUFFER_HEIGHT_RESULT);
        param_resize_bilinear[0].src_width  = VIDEO_PIXEL_HW;
        param_resize_bilinear[0].src_height = VIDEO_PIXEL_VW;
        param_resize_bilinear[0].dst_width  = VIDEO_PIXEL_HW/WINDOW_SCALE;
        param_resize_bilinear[0].dst_height = VIDEO_PIXEL_VW/WINDOW_SCALE;
        R_DK2_Start(drp_lib_id[0], (void *)&param_resize_bilinear[0], sizeof(r_drp_resize_bilinear_t));
        ThisThread::flags_wait_all(DRP_FLG_TILE_ALL);
        R_DK2_Unload(0, drp_lib_id);


        // Overlay some message
        sprintf(str, "Multiple result display demo");
        ascii_font.DrawStr(str, 5, 5 + (AsciiFont::CHAR_PIX_HEIGHT + 1) * 0, 1, 2);
    }
}

int main(void) {
    // Start DRP task
    drpTask.start(callback(drp_task));

    wait(osWaitForever);
}
