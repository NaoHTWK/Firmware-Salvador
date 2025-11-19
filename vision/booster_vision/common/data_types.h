#pragma once

#include <string>
#include <vector>

#include <opencv2/opencv.hpp>

namespace booster_vision {

struct DetectionRes {
    cv::Rect bbox;
    int class_id;
    std::string class_name;
    float confidence;
};

struct SegmentationRes {
    cv::Mat mask;
    std::vector<std::vector<cv::Point>> contour;
    cv::Rect bbox;
    int class_id;
    std::string class_name;
    float confidence;
};

struct Detections{
    std::vector<DetectionRes> ball;
    std::vector<DetectionRes> goalpost;
    std::vector<DetectionRes> person;
    std::vector<DetectionRes> lcross;
    std::vector<DetectionRes> tcross;
    std::vector<DetectionRes> xcross;
    std::vector<DetectionRes> penaltypoint;
    std::vector<DetectionRes> opponent;
    std::vector<DetectionRes> brmarker;
};

}