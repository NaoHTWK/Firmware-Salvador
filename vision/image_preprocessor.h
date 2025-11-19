#ifndef IMAGEPREPROCESSOR_H
#define IMAGEPREPROCESSOR_H

#include <base_detector.h>

#include <vector>

namespace htwk {

class ImagePreprocessor : BaseDetector {
public:
    ImagePreprocessor();

    ImagePreprocessor(int srcWidth, int srcHeight, int scaledWidth,
                      int scaledHeight);
    ImagePreprocessor(const ImagePreprocessor&) = delete;
    ImagePreprocessor(const ImagePreprocessor&&) = delete;
    ImagePreprocessor& operator=(const ImagePreprocessor&) = delete;
    ImagePreprocessor& operator=(ImagePreprocessor&&) = delete;
    ~ImagePreprocessor() = default;

    void proceed(Image* img);

    const std::vector<float>& getScaledImage() {
        return scaledImage;
    }

    const int scaledWidth;
    const int scaledHeight;
    const int srcWidth;
    const int srcHeight;

private:
    std::vector<float> scaledImage;

    void scaleImage(Image* img);
};

}  // namespace htwk
#endif  // IMAGEPREPROCESSOR_H
