#pragma once

#include <memory>
#include <opencv2/opencv.hpp>
#include <string>
#include <vector>

#include "common/data_types.h"

namespace booster_vision {

class YoloV8Detector {
public:
    virtual ~YoloV8Detector() = default;
    virtual void Init(std::string model_path){};
    virtual std::vector<DetectionRes> Inference(const cv::Mat &img) = 0;
    virtual Detections proceed(uint8_t* img_raw) = 0;
    void setConfidenceThreshold(float confidence_threshold) {
        confidence_ = confidence_threshold;
    }

    float getConfidenceThreshold() {
        return confidence_;
    }

    std::string getModelPath() {
        return model_path_;
    }

    static std::shared_ptr<YoloV8Detector> CreateYoloV8Detector();
    static const std::vector<std::string> kClassLabels;

protected:
    YoloV8Detector(const std::string &model_path, const float &conf) :
        model_path_(model_path), confidence_(conf) {
    }

    std::string model_path_;
    float confidence_;
};

} // namespace booster_vision