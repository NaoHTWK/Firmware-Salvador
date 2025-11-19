#include "image_preprocessor.h"

#include <algorithm>
#include <cstring>
#include <iostream>

#include "image.h"


// #include <emmintrin.h>

namespace htwk {

ImagePreprocessor::ImagePreprocessor(int srcWidth, int srcHeight, int scaledWidth,
                      int scaledHeight)
    : srcWidth(srcWidth),
      srcHeight(srcHeight),
      scaledWidth(scaledWidth),
      scaledHeight(scaledHeight),
      scaledImage(scaledWidth * scaledHeight * 3) {}

void ImagePreprocessor::proceed(Image* img) {
    scaleImage(img);
}


// scales interleaved yuv444 down
void ImagePreprocessor::scaleImage(Image* img) {
    const int blockWidth  = srcWidth  / scaledWidth;
    const int blockHeight = srcHeight / scaledHeight;

    if (blockWidth <= 0 || blockHeight <= 0) {
        std::cerr << __PRETTY_FUNCTION__ << ": Invalid scale factors! "
                  << "src: " << srcWidth << "x" << srcHeight
                  << " scaled: " << scaledWidth << "x" << scaledHeight << std::endl;
        return;
    }

    if (blockWidth * blockHeight > 256 * 4)
        std::cerr << __PRETTY_FUNCTION__ << ": Block size too big!!!" << std::endl;

    // normalization fac
    const float fac = 1.0f / (static_cast<float>(blockWidth) *
                                          static_cast<float>(blockHeight) * 255.0f);

    for (int y = 0; y < scaledHeight; ++y) {
        for (int x = 0; x < scaledWidth; ++x) {
            uint32_t sumY = 0;
            uint32_t sumU = 0;
            uint32_t sumV = 0;

            // top left
            const uint8_t* const blockStart =
                img->data_yuv.data() + ( (y * blockHeight) * srcWidth + (x * blockWidth) ) * 3;

            // bounds check
            const size_t imgIndex = static_cast<size_t>(blockStart - img->data_yuv.data());
            const size_t lastOffset =
                static_cast<size_t>((blockHeight - 1) * srcWidth + (blockWidth - 1)) * 3 + 2;

            if (imgIndex + lastOffset >= Image::YUV_SIZE) {
                std::cerr << __PRETTY_FUNCTION__ << ": Memory access out of bounds! "
                          << "imgIndex: " << imgIndex
                          << ", lastOffset: " << lastOffset
                          << ", buffer: " << Image::YUV_SIZE << std::endl;
                return;
            }

            // accumulate
            for (int by = 0; by < blockHeight; ++by) {
                const uint8_t* row = blockStart + static_cast<size_t>(by) * srcWidth * 3;
                for (int bx = 0; bx < blockWidth; ++bx) {
                    const uint8_t* p = row + static_cast<size_t>(bx) * 3;
                    sumY += p[0];
                    sumU += p[1];
                    sumV += p[2];
                }
            }

            const size_t dstIdx = static_cast<size_t>(y) * scaledWidth * 3 + static_cast<size_t>(x) * 3;
            scaledImage[dstIdx + 0] = static_cast<float>(sumY) * fac;
            scaledImage[dstIdx + 1] = static_cast<float>(sumU) * fac;
            scaledImage[dstIdx + 2] = static_cast<float>(sumV) * fac;
        }
    }
}


}  // namespace htwk
