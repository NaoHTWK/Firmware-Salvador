#include "detector.h"

#include <stdexcept>
#include <filesystem>

#if defined(TRT_6_0)
#include "trt_6.0/impl.h"
#elif defined(TRT_6_2)
#include "trt_6.2/impl.h"
#endif

#include "config.h"

namespace booster_vision {

const std::vector<std::string> YoloV8Detector::kClassLabels{"Ball", "Goalpost", "Person", "LCross",
                                                            "TCross", "XCross", "PenaltyPoint", "Opponent", "BRMarker"};

std::shared_ptr<YoloV8Detector> YoloV8Detector::CreateYoloV8Detector() {
    VisionConfig config;
    try {
        std::string model_path = config.detection_model.model_path;
        float conf_thresh = config.detection_model.confidence_threshold;

        return std::shared_ptr<YoloV8Detector>(new YoloV8DetectorTRT(model_path, conf_thresh));
        //return std::shared_ptr<YoloV8Detector>(new YoloV8DetectorTRT(model_path, conf_thresh));
        //return nullptr;
    } catch (const std::exception &e) {
        std::cerr << e.what() << std::endl;
        return nullptr;
    }
}


}
