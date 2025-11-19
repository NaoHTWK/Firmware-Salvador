#ifndef FIELD_BORDER_DETECTOR_H
#define FIELD_BORDER_DETECTOR_H

#include <base_detector.h>
#include <image_preprocessor.h>
#include <tfliteexecuter.h>

#include <cstring>
#include <memory>

namespace htwk {

class FieldBorderDetector : public BaseDetector {
public:
    static constexpr int fieldBorderHeight = 30;
    static constexpr int fieldBorderWidth = 40;
    static constexpr int fieldBorderClassifyData = true;

    FieldBorderDetector();

    FieldBorderDetector(const FieldBorderDetector&) = delete;
    FieldBorderDetector(const FieldBorderDetector&&) = delete;
    FieldBorderDetector& operator=(const FieldBorderDetector&) = delete;
    FieldBorderDetector& operator=(FieldBorderDetector&&) = delete;
    ~FieldBorderDetector();

    void proceed(Image* img);

    const std::vector<int>& getConvexFieldBorder() const {
        return fieldBorderFull;
    }

    const std::vector<float> getInputParameter() {
        //        std::vector<float> tmpInput(channels * inputWidth * inputHeight);
        //        memcpy(tmpInput.data(), input, tmpInput.size() * sizeof(float));
        //        return tmpInput;
        return imgPreprocessor.getScaledImage();
    }

private:
    static constexpr int channels = 3;

    std::vector<int> fieldBorderFull;

    ImagePreprocessor imgPreprocessor;

    TFLiteExecuter tflite;
    float* input;

    void outputToFieldBorder();
};

}  // namespace htwk

#endif  // FIELD_BORDER_DETECTOR_H
