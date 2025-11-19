#include <object_hypothesis.h>

#include <filesystem>
#include <fstream>
#include <iostream>
#include <opencv2/opencv.hpp>
#include <rerun.hpp>
#include <stdexcept>

#include "cuda_utils.h"
#include "impl.h"
#include "logging.h"
#include "model.h"
#include "postprocess.h"
#include "preprocess.h"
#include "utils.h"

Logger gLogger;
const int kOutputSize = kMaxNumOutputBbox * sizeof(Detection) / sizeof(float) + 1;
const static int kOutputSegSize = 32 * (kInputH / 4) * (kInputW / 4);

void serialize_det_engine(std::string& wts_name, std::string& engine_name, int& is_p,
                          std::string& sub_type, float& gd, float& gw, int& max_channels) {
    IBuilder* builder = createInferBuilder(gLogger);
    IBuilderConfig* config = builder->createBuilderConfig();
    IHostMemory* serialized_engine = nullptr;

    if (is_p == 6) {
        serialized_engine = buildEngineYolov8DetP6(builder, config, DataType::kFLOAT, wts_name, gd,
                                                   gw, max_channels);
    } else if (is_p == 2) {
        serialized_engine = buildEngineYolov8DetP2(builder, config, DataType::kFLOAT, wts_name, gd,
                                                   gw, max_channels);
    } else {
        serialized_engine = buildEngineYolov8Det(builder, config, DataType::kFLOAT, wts_name, gd,
                                                 gw, max_channels);
    }

    assert(serialized_engine);
    std::ofstream p(engine_name, std::ios::binary);
    if (!p) {
        std::cout << "could not open plan output file" << std::endl;
        assert(false);
    }
    p.write(reinterpret_cast<const char*>(serialized_engine->data()), serialized_engine->size());

    delete serialized_engine;
    delete config;
    delete builder;
}

void deserialize_det_engine(std::string& engine_name, IRuntime** runtime, ICudaEngine** engine,
                            IExecutionContext** context) {
    std::cout << "loading det engine: " << engine_name << std::endl;
    std::ifstream file(engine_name, std::ios::binary);
    if (!file.good()) {
        std::cerr << "read " << engine_name << " error!" << std::endl;
        assert(false);
    }
    size_t size = 0;
    file.seekg(0, file.end);
    size = file.tellg();
    file.seekg(0, file.beg);
    char* serialized_engine = new char[size];
    assert(serialized_engine);
    file.read(serialized_engine, size);
    file.close();

    *runtime = createInferRuntime(gLogger);
    assert(*runtime);
    *engine = (*runtime)->deserializeCudaEngine(serialized_engine, size);
    assert(*engine);
    *context = (*engine)->createExecutionContext();
    assert(*context);
    delete[] serialized_engine;
}

void prepare_det_buffer(ICudaEngine* engine, float** input_buffer_device,
                        float** output_buffer_device, float** output_buffer_host,
                        float** decode_ptr_host, float** decode_ptr_device,
                        std::string cuda_post_process) {
    assert(engine->getNbBindings() == 2);
    // In order to bind the buffers, we need to know the names of the input and output tensors.
    // Note that indices are guaranteed to be less than IEngine::getNbBindings()
    const int inputIndex = engine->getBindingIndex(kInputTensorName);
    const int outputIndex = engine->getBindingIndex(kOutputTensorName);
    assert(inputIndex == 0);
    assert(outputIndex == 1);
    // Create GPU buffers on device
    CUDA_CHECK(cudaMalloc((void**)input_buffer_device,
                          kBatchSize * 3 * kInputH * kInputW * sizeof(float)));
    CUDA_CHECK(cudaMalloc((void**)output_buffer_device, kBatchSize * kOutputSize * sizeof(float)));
    if (cuda_post_process == "c") {
        *output_buffer_host = new float[kBatchSize * kOutputSize];
    } else if (cuda_post_process == "g") {
        if (kBatchSize > 1) {
            std::cerr << "Do not yet support GPU post processing for multiple batches" << std::endl;
            exit(0);
        }
        // Allocate memory for decode_ptr_host and copy to device
        *decode_ptr_host = new float[1 + kMaxNumOutputBbox * bbox_element];
        CUDA_CHECK(cudaMalloc((void**)decode_ptr_device,
                              sizeof(float) * (1 + kMaxNumOutputBbox * bbox_element)));
    }
}

void infer_det(IExecutionContext& context, cudaStream_t& stream, void** buffers, float* output,
               int batchsize, float* decode_ptr_host, float* decode_ptr_device, int model_bboxes,
               std::string cuda_post_process, float confidence_threshold, float nms_threshold) {
    // infer_det on the batch asynchronously, and DMA output back to host
    context.enqueue(batchsize, buffers, stream, nullptr);
    if (cuda_post_process == "c") {
        CUDA_CHECK(cudaMemcpyAsync(output, buffers[1], batchsize * kOutputSize * sizeof(float),
                                   cudaMemcpyDeviceToHost, stream));
    } else if (cuda_post_process == "g") {
        CUDA_CHECK(cudaMemsetAsync(decode_ptr_device, 0,
                                   sizeof(float) * (1 + kMaxNumOutputBbox * bbox_element), stream));
        cuda_decode((float*)buffers[1], model_bboxes, confidence_threshold, decode_ptr_device,
                    kMaxNumOutputBbox, stream);
        cuda_nms(decode_ptr_device, nms_threshold, kMaxNumOutputBbox, stream);  // cuda nms
        CUDA_CHECK(cudaMemcpyAsync(decode_ptr_host, decode_ptr_device,
                                   sizeof(float) * (1 + kMaxNumOutputBbox * bbox_element),
                                   cudaMemcpyDeviceToHost, stream));
    }

    CUDA_CHECK(cudaStreamSynchronize(stream));
}

void YoloV8DetectorTRT::Init(std::string model_path) {
    if (model_path.find(".engine") == std::string::npos) {
        throw std::runtime_error("incorrect model name: " + model_path);
    }

    deserialize_det_engine(model_path, &runtime, &engine, &context);

    CUDA_CHECK(cudaStreamCreate(&stream));
    cuda_preprocess_init(kMaxInputImageSize);
    auto out_dims = engine->getBindingDimensions(1);
    model_bboxes = out_dims.d[0];

    prepare_det_buffer(engine, &device_buffers[0], &device_buffers[1], &output_buffer_host,
                       &decode_ptr_host, &decode_ptr_device, cuda_post_process);
    std::cout << "det model initialization, done!" << std::endl;
}

booster_vision::Detections YoloV8DetectorTRT::proceed(const cv::Mat& img) {
    std::vector<booster_vision::DetectionRes> det_res = Inference(img);
    booster_vision::Detections detections;

    for (auto& res : det_res) {
        switch (res.class_id) {
            case 0: {
                detections.ball.push_back(res);
                break;
            }
            case 1: {
                detections.goalpost.push_back(res);
                break;
            }
            case 2: {
                detections.person.push_back(res);
                break;
            }
            case 3: {
                detections.lcross.push_back(res);
                break;
            }
            case 4: {
                detections.tcross.push_back(res);
                break;
            }
            case 5: {
                detections.xcross.push_back(res);
                break;
            }
            case 6: {
                detections.penaltypoint.push_back(res);
                break;
            }
            case 7: {
                detections.opponent.push_back(res);
                break;
            }
            case 8: {
                detections.brmarker.push_back(res);
                break;
            }
        }
    }

    return detections;
}

std::vector<booster_vision::DetectionRes> YoloV8DetectorTRT::Inference(const cv::Mat& img) {
    auto start = std::chrono::system_clock::now();
    // Preprocess
    std::vector<cv::Mat> img_batch = {img};
    cuda_batch_preprocess(img_batch, device_buffers[0], kInputW, kInputH, stream);
    // Run inference
    infer_det(*context, stream, (void**)device_buffers, output_buffer_host, kBatchSize,
              decode_ptr_host, decode_ptr_device, model_bboxes, cuda_post_process);
    std::vector<std::vector<Detection>> res_batch;
    if (cuda_post_process == "c") {
        // NMS
        batch_nms(res_batch, output_buffer_host, img_batch.size(), kOutputSize, kConfThresh,
                  kNmsThresh);
    } else if (cuda_post_process == "g") {
        // Process gpu decode and nms results
        batch_process(res_batch, decode_ptr_host, img_batch.size(), bbox_element, img_batch);
    }

    std::vector<booster_vision::DetectionRes> ret;
    auto scale = std::min(kInputW / static_cast<float>(img.cols),
                          kInputH / static_cast<float>(img.rows));
    auto offset_x = (kInputW - img.cols * scale) / 2;
    auto offset_y = (kInputH - img.rows * scale) / 2;

    cv::Mat s2d = (cv::Mat_<float>(2, 3) << scale, 0, offset_x, 0, scale, offset_y);
    cv::Mat d2s;
    cv::invertAffineTransform(s2d, d2s);

    for (auto res : res_batch[0]) {
        booster_vision::DetectionRes det_res;
        int x_min = std::max(
                0, static_cast<int>(res.bbox[0] * d2s.at<float>(0, 0) + d2s.at<float>(0, 2)));
        int y_min = std::max(
                0, static_cast<int>(res.bbox[1] * d2s.at<float>(1, 1) + d2s.at<float>(1, 2)));
        int x_max = std::min(img.cols - 1, static_cast<int>(res.bbox[2] * d2s.at<float>(0, 0) +
                                                            d2s.at<float>(0, 2)));
        int y_max = std::min(img.rows - 1, static_cast<int>(res.bbox[3] * d2s.at<float>(1, 1) +
                                                            d2s.at<float>(1, 2)));
        det_res.bbox = cv::Rect(x_min, y_min, x_max - x_min, y_max - y_min);
        det_res.confidence = res.conf;
        det_res.class_id = res.class_id;
        ret.push_back(det_res);
    }
    auto end = std::chrono::system_clock::now();
    return ret;
}

YoloV8DetectorTRT::~YoloV8DetectorTRT() {
    // Release stream and buffers
    cudaStreamDestroy(stream);
    CUDA_CHECK(cudaFree(device_buffers[0]));
    CUDA_CHECK(cudaFree(device_buffers[1]));
    CUDA_CHECK(cudaFree(decode_ptr_device));
    delete[] decode_ptr_host;
    delete[] output_buffer_host;
    cuda_preprocess_destroy();
    // Destroy the engine
    delete context;
    delete engine;
    delete runtime;
}
