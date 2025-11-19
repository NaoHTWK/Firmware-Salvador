#pragma once
#include <string>
#include <NvInfer.h>
#include <NvInferRuntime.h>
#include "detector.h"
#include "trt/config.h"

using namespace nvinfer1;
void serialize_det_engine(std::string& wts_name, std::string& engine_name, int& is_p, std::string& sub_type, float& gd,
                      float& gw, int& max_channels);
void deserialize_det_engine(std::string& engine_name, IRuntime** runtime, ICudaEngine** engine,
                        IExecutionContext** context);

void prepare_det_buffer(ICudaEngine* engine, float** input_buffer_device, float** output_buffer_device,
                    float** output_buffer_host, float** decode_ptr_host, float** decode_ptr_device,
                    std::string cuda_post_process);

void infer_det(IExecutionContext& context, cudaStream_t& stream, void** buffers, float* output, int batchsize,
           float* decode_ptr_host, float* decode_ptr_device, int model_bboxes, std::string cuda_post_process,
           float confidence_threshold = kConfThresh, float nms_threhosld = kNmsThresh);

class YoloV8DetectorTRT : public booster_vision::YoloV8Detector {
 public:
  YoloV8DetectorTRT(const std::string& path, const float& conf) : booster_vision::YoloV8Detector(path, conf) {
    Init(path);
  }
  ~YoloV8DetectorTRT();

  void Init(std::string model_path) override;
  std::vector<booster_vision::DetectionRes> Inference(const cv::Mat& img) override;
  booster_vision::Detections proceed(const cv::Mat& img) override;
 private:
  IRuntime* runtime = nullptr;
  ICudaEngine* engine = nullptr;
  IExecutionContext* context = nullptr;

  // Prepare cpu and gpu buffers
  int model_bboxes;
  cudaStream_t stream;
  float* device_buffers[2];
  float* output_buffer_host = nullptr;
  float* decode_ptr_host = nullptr;
  float* decode_ptr_device = nullptr;
  std::string cuda_post_process = "g";
};