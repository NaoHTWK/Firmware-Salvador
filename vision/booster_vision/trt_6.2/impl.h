#pragma once
#include <NvInfer.h>
#include <NvInferRuntime.h>

#include <random>
#include <string>

#include "../detector.h"

using namespace nvinfer1;

struct InferDeleter {
    template <typename T>
    void operator()(T* obj) const {
        delete obj;
    }
};

class YoloV8DetectorTRT : public booster_vision::YoloV8Detector {
public:
    YoloV8DetectorTRT(const std::string& path, const float& conf)
        : booster_vision::YoloV8Detector(path, conf) {
        Init(path);
    }
    ~YoloV8DetectorTRT();

    void Init(std::string model_path) override;
    std::vector<booster_vision::DetectionRes> Inference(const cv::Mat& img) override;

    booster_vision::Detections proceed(uint8_t* img_raw) override;

private:
    bool LoadEngine();

    void MemcpyBuffers(void* dstPtr, void const* srcPtr, size_t byteSize, cudaMemcpyKind memcpyType,
                       bool const async = false);
    bool PreProcess(const cv::Mat& img, std::vector<float>& factors);
    std::vector<booster_vision::DetectionRes> PostProcess(const std::vector<float>& factors);

    std::shared_ptr<nvinfer1::IRuntime> runtime_;
    std::shared_ptr<nvinfer1::ICudaEngine> engine_;
    std::shared_ptr<nvinfer1::IExecutionContext> context_;
    cudaStream_t stream_ = 0;

    nvinfer1::Dims model_input_dims_;
    nvinfer1::Dims model_output_dims_;

    bool async_infer_ = true;
    size_t input_size_;
    size_t output_size_;
    int img_width_ = 0;
    int img_height_ = 0;
    std::vector<void*> bindings_;
    void* input_mem_{nullptr};
    void* output_mem_{nullptr};

    float* input_buff_;
    float* output_buff_;

    bool squre_input_ = true;
};
