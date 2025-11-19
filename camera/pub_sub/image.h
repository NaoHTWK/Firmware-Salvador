#pragma once

#include <fstream>
#include <memory>
#include <vector>

#include "odometer.h"
#include "robot_time.h"
#include "fluxlog_vision.h"

class CamPose;

struct Image {
    constexpr static int WIDTH = CAMERA_WIDTH;
    constexpr static int HEIGHT = CAMERA_HEIGHT;

    constexpr static int DEPTH_WIDTH = DEPTH_CAMERA_WIDTH;
    constexpr static int DEPTH_HEIGHT = DEPTH_CAMERA_HEIGHT;

    constexpr static int YUV_SIZE = WIDTH * HEIGHT * 3;
    constexpr static int BGR_SIZE = WIDTH * HEIGHT * 3;
    constexpr static int BGRA_SIZE = WIDTH * HEIGHT * 4;
    constexpr static int DEPTH_SIZE = DEPTH_WIDTH * DEPTH_HEIGHT;

    constexpr static float DEPTH_SCALE = 0.001f; 

    alignas(512) std::array<float, DEPTH_SIZE> data_depth_m; //meters use this if you are cool
    alignas(512) std::array<uint16_t, DEPTH_SIZE> data_depth_scaled; //supposed to be mm
    alignas(512) std::array<uint8_t, BGRA_SIZE> data_bgra;
    alignas(512) std::array<uint8_t, YUV_SIZE> data_yuv; // YUV444

    uint64_t frame_id;
    int64_t timestamp_us;
    int64_t timestamp_camera;  // Not filled
    int64_t last_image_time;
    float fx;
    float fy;
    float cx;
    float cy;
    std::shared_ptr<CamPose> cam_pose_ptr;  // Pointer to CamPose for camera pose information
    int depth_width;
    int depth_height;
    std::vector<uint16_t> depth_data;

    bool log_image_ = false;
    bool log_depth_image_ = false;

    void log() {
        if (log_image_) {
            htwk::log_encoded_image_jpeg_raw("image/rgb", data_bgra.data());
        }

        if (log_depth_image_) {
            static std::ofstream file("depth_images_" + std::to_string(time_us()) + ".raw",
                                        std::ios::out | std::ios::binary);
            file.write(reinterpret_cast<const char*>(&timestamp_us), sizeof(int64_t));
            file.write(reinterpret_cast<const char*>(&depth_width), sizeof(int));
            file.write(reinterpret_cast<const char*>(&depth_height), sizeof(int));
            file.write(reinterpret_cast<const char*>(depth_data.data()),
                    depth_data.size() * sizeof(uint16_t));
        }
    }

	// converts BGRA or BGR to fullrange YUV (Y Cb Cr)
	// slightly modified version of https://github.com/yszheda/rgb2yuv-neon/blob/master/yuv444.cpp ; bugfix and bgr match
	static inline void BGR2YUV444(unsigned char * __restrict__ yuv, unsigned char * __restrict__ bgr, const int pixel_num, const int bgrChannelNum)
     {
            const uint8x8_t u8_zero = vdup_n_u8(0);
            const uint16x8_t u16_rounding = vdupq_n_u16(128);
            const int16x8_t s16_rounding = vdupq_n_s16(128);
            const int8x16_t s8_rounding = vdupq_n_s8(128);

            int count = pixel_num / 16;

            int i;
            for (i = 0; i < count; ++i) {
                // Load bgr
                uint8x16x3_t pixel_rgb;
                if (bgrChannelNum == 3) {
                    uint8x16x3_t t = vld3q_u8(bgr);              // t: B,G,R
                    pixel_rgb.val[0] = t.val[2]; // R
                    pixel_rgb.val[1] = t.val[1]; // G
                    pixel_rgb.val[2] = t.val[0]; // B
                } else {
                    uint8x16x4_t t = vld4q_u8(bgr);              // t: B,G,R,A
                    pixel_rgb.val[0] = t.val[2]; // R
                    pixel_rgb.val[1] = t.val[1]; // G
                    pixel_rgb.val[2] = t.val[0]; // B
                }
                bgr += bgrChannelNum * 16;

                uint8x8_t high_r = vget_high_u8(pixel_rgb.val[0]);
                uint8x8_t low_r = vget_low_u8(pixel_rgb.val[0]);
                uint8x8_t high_g = vget_high_u8(pixel_rgb.val[1]);
                uint8x8_t low_g = vget_low_u8(pixel_rgb.val[1]);
                uint8x8_t high_b = vget_high_u8(pixel_rgb.val[2]);
                uint8x8_t low_b = vget_low_u8(pixel_rgb.val[2]);
                int16x8_t signed_high_r = vreinterpretq_s16_u16(vaddl_u8(high_r, u8_zero));
                int16x8_t signed_low_r = vreinterpretq_s16_u16(vaddl_u8(low_r, u8_zero));
                int16x8_t signed_high_g = vreinterpretq_s16_u16(vaddl_u8(high_g, u8_zero));
                int16x8_t signed_low_g = vreinterpretq_s16_u16(vaddl_u8(low_g, u8_zero));
                int16x8_t signed_high_b = vreinterpretq_s16_u16(vaddl_u8(high_b, u8_zero));
                int16x8_t signed_low_b = vreinterpretq_s16_u16(vaddl_u8(low_b, u8_zero));

                // NOTE:
                // declaration may not appear after executable statement in block
                uint16x8_t high_y;
                uint16x8_t low_y;
                uint8x8_t scalar = vdup_n_u8(77);
                int16x8_t high_u;
                int16x8_t low_u;
                int16x8_t signed_scalar = vdupq_n_s16(-43);
                int16x8_t high_v;
                int16x8_t low_v;
                uint8x16x3_t pixel_yuv;
                int8x16_t u;
                int8x16_t v;

                // 1. Multiply transform matrix (Y′: unsigned, U/V: signed)
                high_y = vmull_u8(high_r, scalar);
                low_y = vmull_u8(low_r, scalar);

                high_u = vmulq_s16(signed_high_r, signed_scalar);
                low_u = vmulq_s16(signed_low_r, signed_scalar);

                signed_scalar = vdupq_n_s16(128);
                high_v = vmulq_s16(signed_high_r, signed_scalar);
                low_v = vmulq_s16(signed_low_r, signed_scalar);

                scalar = vdup_n_u8(150);
                high_y = vmlal_u8(high_y, high_g, scalar);
                low_y = vmlal_u8(low_y, low_g, scalar);

                signed_scalar = vdupq_n_s16(-85);
                high_u = vmlaq_s16(high_u, signed_high_g, signed_scalar);
                low_u = vmlaq_s16(low_u, signed_low_g, signed_scalar);

                signed_scalar = vdupq_n_s16(-107);
                high_v = vmlaq_s16(high_v, signed_high_g, signed_scalar);
                low_v = vmlaq_s16(low_v, signed_low_g, signed_scalar);

                scalar = vdup_n_u8(29);
                high_y = vmlal_u8(high_y, high_b, scalar);
                low_y = vmlal_u8(low_y, low_b, scalar);

                signed_scalar = vdupq_n_s16(128);
                high_u = vmlaq_s16(high_u, signed_high_b, signed_scalar);
                low_u = vmlaq_s16(low_u, signed_low_b, signed_scalar);

                signed_scalar = vdupq_n_s16(-21);
                high_v = vmlaq_s16(high_v, signed_high_b, signed_scalar);
                low_v = vmlaq_s16(low_v, signed_low_b, signed_scalar);
                // 2. Scale down (">>8") to 8-bit values with rounding ("+128") (Y′: unsigned, U/V: signed)
                // 3. Add an offset to the values to eliminate any negative values (all results are 8-bit unsigned)

                high_y = vaddq_u16(high_y, u16_rounding);
                low_y = vaddq_u16(low_y, u16_rounding);

                high_u = vaddq_s16(high_u, s16_rounding);
                low_u = vaddq_s16(low_u, s16_rounding);

                high_v = vaddq_s16(high_v, s16_rounding);
                low_v = vaddq_s16(low_v, s16_rounding);

                pixel_yuv.val[0] = vcombine_u8(vqshrn_n_u16(low_y, 8), vqshrn_n_u16(high_y, 8));

                u = vcombine_s8(vqshrn_n_s16(low_u, 8), vqshrn_n_s16(high_u, 8));

                v = vcombine_s8(vqshrn_n_s16(low_v, 8), vqshrn_n_s16(high_v, 8));

                u = vaddq_s8(u, s8_rounding);
                pixel_yuv.val[1] = vreinterpretq_u8_s8(u);

                v = vaddq_s8(v, s8_rounding);
                pixel_yuv.val[2] = vreinterpretq_u8_s8(v);

                // Store
                vst3q_u8(yuv, pixel_yuv);

                yuv += 3 * 16;
            }

            // Handle leftovers
			int tail = pixel_num - count * 16;
            for (i = 0; i < tail; ++i) {
                uint8_t r = bgr[i * bgrChannelNum + 2];
                uint8_t g = bgr[i * bgrChannelNum + 1];
                uint8_t b = bgr[i * bgrChannelNum + 0];

                uint16_t y_tmp = 77 * r + 150 * g + 29 * b;
                int16_t u_tmp = -43 * r - 85 * g + 128 * b;
                int16_t v_tmp = 128 * r - 107 * g - 21 * b;

                y_tmp = (y_tmp + 128) >> 8;
                u_tmp = (u_tmp + 128) >> 8;
                v_tmp = (v_tmp + 128) >> 8;

                yuv[i * 3] = (uint8_t) y_tmp;
                yuv[i * 3 + 1] = (uint8_t) (u_tmp + 128);
                yuv[i * 3 + 2] = (uint8_t) (v_tmp + 128);
            }
        }
};