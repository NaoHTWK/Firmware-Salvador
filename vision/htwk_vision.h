#ifndef HTWK_VISION_H
#define HTWK_VISION_H

#include <field_border_detector.h>
#include <field_color_detector.h>
#include <line_detector.h>

#include "async.h"
// #include <localization_utils.h>
#include <region_classifier.h>

#include <cstdint>
#include <memory>
#include <string>
#include <thread>
#include <vector>

#include "cam_pose.h"
#include "camera_pub_sub.h"
#include "obstacle_detector_legacy.h"
// #include "yolov8/yolo_runner.h"
#include <detector.h>

namespace htwk {

class HTWKVision {
private:
    void createAdressLookups();

    ::ThreadPool* thread_pool = nullptr;

    // YoloRunner yoloV8_runner;
    std::shared_ptr<booster_vision::YoloV8Detector> booster_yoloV8_detector;
    booster_vision::Detections booster_yolo_results;

    // pubsub channels

public:
    FieldColorDetector* fieldColorDetector = nullptr;
    std::shared_ptr<FieldBorderDetector> fieldBorderDetector = nullptr;
    RegionClassifier* regionClassifier = nullptr;
    LineDetector* lineDetector = nullptr;
    ObstacleDetectorLegacy* obstacle_detector_legacy = nullptr;

    HTWKVision();  // Default constructor
    HTWKVision(HTWKVision& h) = delete;
    HTWKVision(HTWKVision&& h) = delete;
    HTWKVision& operator=(const HTWKVision&) = delete;
    HTWKVision& operator=(HTWKVision&&) = delete;
    ~HTWKVision();

    void proceed(std::shared_ptr<Image> img);

    void publish_booster_vision_to_pubsub(const Image& img);
    void log();
};

}  // namespace htwk

#endif  // HTWK_VISION_H
