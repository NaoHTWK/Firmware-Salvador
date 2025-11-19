#include "field_border_detector.h"

#include <algorithm_ext.h>
// Architecture-specific SIMD headers
#if defined(__x86_64__) || defined(_M_X64) || defined(__i386) || defined(_M_IX86)
#include <emmintrin.h>  // x86 SSE2 intrinsics
#elif defined(__ARM_NEON) || defined(__aarch64__)
#include <arm_neon.h>  // ARM NEON intrinsics
#endif
#include <algorithm>  // For std::min, std::max
#include <cstdlib>    // For aligned_alloc and free
#include <image.h>

#include "logging.h"
namespace htwk {

FieldBorderDetector::FieldBorderDetector()
    : BaseDetector(),
      fieldBorderFull(CAMERA_WIDTH, 0),
      imgPreprocessor(Image::WIDTH, Image::HEIGHT, fieldBorderWidth, fieldBorderHeight) {

    if (fieldBorderClassifyData) {
        tflite.loadModelFromFile(TFLiteExecuter::getTFliteModelPath() + "/uc-field-border-classifier.tflite",
                                 {1, fieldBorderHeight, fieldBorderWidth, channels});
        input = tflite.getInputTensor();
    } else {
        input = (float*)aligned_alloc(
                16, (((channels * fieldBorderWidth * fieldBorderHeight * sizeof(float)) + 15) / 16) * 16);
    }
}

FieldBorderDetector::~FieldBorderDetector() {
    if (!fieldBorderClassifyData)
        free(input);
}

void FieldBorderDetector::proceed(Image* img) {
    Timer t("FieldBorderDetector", 50);
    // EASY_FUNCTION();

    //    createInputData(img);
    imgPreprocessor.proceed(img);
    memcpy(input, imgPreprocessor.getScaledImage().data(),
           sizeof(*input) * channels * fieldBorderWidth * fieldBorderHeight);

    if (!fieldBorderClassifyData)
        return;

    // EASY_BLOCK("TFL FieldBorder");
    tflite.execute();
    // EASY_END_BLOCK;

    outputToFieldBorder();
}

void FieldBorderDetector::outputToFieldBorder() {
    const float* features = tflite.getOutputTensor();
    int numBins = fieldBorderWidth;
    int stepWidth = width / numBins;
    for (int x = 0; x < width; x++) {
        int x1 = (x - stepWidth / 2) / stepWidth;
        if (x1 < 0) {
            x1 = 0;
        }
        if (x1 >= numBins - 2) {
            x1 = numBins - 2;
        }
        int y1 = (int)(features[x1] * height);
        int x2 = x1 + 1;
        int y2 = (int)(features[x2] * height);
        double f = (x - stepWidth / 2 - x1 * stepWidth * 1.) / stepWidth;
        int y = (int)(y2 * f + y1 * (1 - f));
        if (y < 0)
            y = 0;
        if (y >= height) {
            y = height - 1;
        }
        fieldBorderFull[x] = y;
    }
}

}  // namespace htwk
