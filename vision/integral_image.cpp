#include "integral_image.h"

#include <cstdlib>
#include <iostream>
#include <arm_neon.h> // ARM NEON intrinsics

namespace htwk {

IntegralImage::IntegralImage(HtwkVisionConfig &config)
: BaseDetector(lutCb, lutCr, config), iWidth(config.width / INTEGRAL_SCALE), iHeight(config.height / INTEGRAL_SCALE) {
    integralImg = static_cast<int*>(aligned_alloc(16, iWidth*iHeight*sizeof(int)));
}

IntegralImage::~IntegralImage() {
    free(integralImg);
}


#define getValue(img, x, y) (getY((img), (x), (y)) + 3*getCr((img), (x), (y)))

void IntegralImage::proceed(uint8_t *img) const {
    Timer t("IntegralImage", 50);
    //EASY_FUNCTION(profiler::colors::Red100);

    integralImg[0] = getValue(img, 0, 0);
    for(int x = 1; x < iWidth; x++) {
        integralImg[x] = integralImg[x-1] + (getValue(img, x*INTEGRAL_SCALE, 0));
    }

    if (INTEGRAL_SCALE == 2) {
        const int factor = 4;
        
        // Using NEON intrinsics for ARM
        uint32x4_t y_mask = vdupq_n_u32(0xFF);
        uint32x4_t v_mask = vdupq_n_u32(0xFF000000);
        
        // Create a mask for the first 3 elements (equivalent to _mm_setr_epi32(-1, -1, -1, 0))
        int32_t fff0_array[4] = {-1, -1, -1, 0};
        int32x4_t fff0 = vld1q_s32(fff0_array);
        
        for(int y = 1; y < iHeight; y++) {
            int32x4_t sum = vdupq_n_s32(0);
            
            for(int x = 0; x < iWidth/factor; x++) {
                const int addr0 = x*factor + y*iWidth;

                // Load previous row's integral values
                int32x4_t i0 = vld1q_s32(&(integralImg[addr0-iWidth]));
                
                // Calculate image position
                int xr = x * factor * INTEGRAL_SCALE;
                
                // Load YCrCb values
                uint32x4_t ycr = vld1q_u32((const uint32_t*)&img[(xr + y*INTEGRAL_SCALE*width) << 1]);
                
                // Extract Y and V components
                uint32x4_t y_comp = vandq_u32(ycr, y_mask);
                uint32x4_t v = vandq_u32(ycr, v_mask);
                
                // Shift V component (equivalent to _mm_srli_epi32)
                uint32x4_t vs7 = vshrq_n_u32(v, 23);
                uint32x4_t vs8 = vshrq_n_u32(v, 24);
                
                // Add components
                int32x4_t val = vreinterpretq_s32_u32(
                    vaddq_u32(vaddq_u32(vs7, vs8), y_comp)
                );
                
                // Apply mask to get first 3 elements
                int32x4_t val012c = vandq_s32(val, fff0);
                
                // Create arrays for constructing shuffle equivalents
                int32_t valc000_array[4];
                int32_t valcc11_array[4];
                int32_t valccc2_array[4];
                
                // Store val012c to temporary array to access elements
                vst1q_s32(valc000_array, val012c);
                
                // Create valc000 (equivalent to _mm_shuffle_epi32(val012c, _MM_SHUFFLE(0, 0, 0, 3)))
                valc000_array[0] = valc000_array[3];  // element 3
                valc000_array[1] = 0;
                valc000_array[2] = 0;
                valc000_array[3] = 0;
                int32x4_t valc000 = vld1q_s32(valc000_array);
                
                // Create valcc11 (equivalent to _mm_shuffle_epi32(val012c, _MM_SHUFFLE(1, 1, 3, 3)))
                vst1q_s32(valcc11_array, val012c);
                valcc11_array[0] = valcc11_array[3];  // element 3
                valcc11_array[1] = valcc11_array[3];  // element 3
                valcc11_array[2] = valcc11_array[1];  // element 1
                valcc11_array[3] = valcc11_array[1];  // element 1
                int32x4_t valcc11 = vld1q_s32(valcc11_array);
                
                // Create valccc2 (equivalent to _mm_shuffle_epi32(val012c, _MM_SHUFFLE(2, 3, 3, 3)))
                vst1q_s32(valccc2_array, val012c);
                valccc2_array[0] = valccc2_array[3];  // element 3
                valccc2_array[1] = valccc2_array[3];  // element 3
                valccc2_array[2] = valccc2_array[3];  // element 3
                valccc2_array[3] = valccc2_array[2];  // element 2
                int32x4_t valccc2 = vld1q_s32(valccc2_array);
                
                // Calculate sum
                int32x4_t add1 = vaddq_s32(val, sum);
                int32x4_t add2 = vaddq_s32(valc000, valcc11);
                int32x4_t add3 = vaddq_s32(add1, add2);
                int32x4_t iSum0 = vaddq_s32(add3, valccc2);
                
                // Update sum with last element
                int32_t iSum0_array[4];
                vst1q_s32(iSum0_array, iSum0);
                sum = vdupq_n_s32(iSum0_array[3]);
                
                // Add with previous row and store result
                int32x4_t r0 = vaddq_s32(i0, iSum0);
                vst1q_s32(&integralImg[addr0], r0);
            }
        }
    } else {
        const int factor = 16;
        for(int y = 1; y < iHeight; y++) {
            int sum = 0;
            for(int x = 0; x < iWidth/factor; x++) {
                const int addr0 = x*factor + y*iWidth + 0;
                const int addr1 = x*factor + y*iWidth + 4;
                const int addr2 = x*factor + y*iWidth + 8;
                const int addr3 = x*factor + y*iWidth + 12;

                // Load integral values from previous row using NEON
                int32x4_t i0 = vld1q_s32(&(integralImg[addr0-iWidth]));
                int32x4_t i1 = vld1q_s32(&(integralImg[addr1-iWidth]));
                int32x4_t i2 = vld1q_s32(&(integralImg[addr2-iWidth]));
                int32x4_t i3 = vld1q_s32(&(integralImg[addr3-iWidth]));

                // Calculate the sum values
                const int cr0  = sum  + getValue(img, (x*factor+ 0)*INTEGRAL_SCALE, y*INTEGRAL_SCALE);
                const int cr1  = cr0  + getValue(img, (x*factor+ 1)*INTEGRAL_SCALE, y*INTEGRAL_SCALE);
                const int cr2  = cr1  + getValue(img, (x*factor+ 2)*INTEGRAL_SCALE, y*INTEGRAL_SCALE);
                const int cr3  = cr2  + getValue(img, (x*factor+ 3)*INTEGRAL_SCALE, y*INTEGRAL_SCALE);
                const int cr4  = cr3  + getValue(img, (x*factor+ 4)*INTEGRAL_SCALE, y*INTEGRAL_SCALE);
                const int cr5  = cr4  + getValue(img, (x*factor+ 5)*INTEGRAL_SCALE, y*INTEGRAL_SCALE);
                const int cr6  = cr5  + getValue(img, (x*factor+ 6)*INTEGRAL_SCALE, y*INTEGRAL_SCALE);
                const int cr7  = cr6  + getValue(img, (x*factor+ 7)*INTEGRAL_SCALE, y*INTEGRAL_SCALE);
                const int cr8  = cr7  + getValue(img, (x*factor+ 8)*INTEGRAL_SCALE, y*INTEGRAL_SCALE);
                const int cr9  = cr8  + getValue(img, (x*factor+ 9)*INTEGRAL_SCALE, y*INTEGRAL_SCALE);
                const int cr10 = cr9  + getValue(img, (x*factor+10)*INTEGRAL_SCALE, y*INTEGRAL_SCALE);
                const int cr11 = cr10 + getValue(img, (x*factor+11)*INTEGRAL_SCALE, y*INTEGRAL_SCALE);
                const int cr12 = cr11 + getValue(img, (x*factor+12)*INTEGRAL_SCALE, y*INTEGRAL_SCALE);
                const int cr13 = cr12 + getValue(img, (x*factor+13)*INTEGRAL_SCALE, y*INTEGRAL_SCALE);
                const int cr14 = cr13 + getValue(img, (x*factor+14)*INTEGRAL_SCALE, y*INTEGRAL_SCALE);
                const int cr15 = cr14 + getValue(img, (x*factor+15)*INTEGRAL_SCALE, y*INTEGRAL_SCALE);
                sum = cr15;

                // Create arrays for sum values
                int32_t data0[4] = {cr0, cr1, cr2, cr3};
                int32_t data1[4] = {cr4, cr5, cr6, cr7};
                int32_t data2[4] = {cr8, cr9, cr10, cr11};
                int32_t data3[4] = {cr12, cr13, cr14, cr15};
                
                // Load into NEON vectors
                int32x4_t iSum0 = vld1q_s32(data0);
                int32x4_t iSum1 = vld1q_s32(data1);
                int32x4_t iSum2 = vld1q_s32(data2);
                int32x4_t iSum3 = vld1q_s32(data3);

                // Add with previous row
                int32x4_t r0 = vaddq_s32(i0, iSum0);
                int32x4_t r1 = vaddq_s32(i1, iSum1);
                int32x4_t r2 = vaddq_s32(i2, iSum2);
                int32x4_t r3 = vaddq_s32(i3, iSum3);

                // Store results
                vst1q_s32(&integralImg[addr0], r0);
                vst1q_s32(&integralImg[addr1], r1);
                vst1q_s32(&integralImg[addr2], r2);
                vst1q_s32(&integralImg[addr3], r3);
            }
        }
    }
}

}//namespace htwk
