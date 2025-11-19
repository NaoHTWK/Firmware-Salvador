#include "impl.h"

#include <algorithm>
#include <filesystem>
#include <fstream>
#include <iostream>
#include <loguru.hpp>
#include <opencv2/opencv.hpp>
#include <stdexcept>

#include "../common/logging.h"
#include "robot_time.h"

Logger logger;

void YoloV8DetectorTRT::Init(std::string model_path) {
    if (model_path.find(".engine") == std::string::npos) {
        throw std::runtime_error("incorrect model name: " + model_path);
    }

    if (!LoadEngine()) {
        throw std::runtime_error("Failed to load engine from " + model_path);
    }

    input_size_ = model_input_dims_.d[0] * model_input_dims_.d[1] * model_input_dims_.d[2] *
                  model_input_dims_.d[3];
    output_size_ = model_output_dims_.d[0] * model_output_dims_.d[1] * model_output_dims_.d[2];
    // Initialize input buf with zeroes so we don't have to set them every frame when letterboxing.
    input_buff_ = (float*)calloc(input_size_, sizeof(float));
    output_buff_ = (float*)malloc(output_size_ * sizeof(float));
    cudaMalloc(&input_mem_, input_size_ * sizeof(float));
    cudaMalloc(&output_mem_, output_size_ * sizeof(float));
    if (async_infer_) {
        cudaStreamCreate(&stream_);
    } else {
        bindings_.emplace_back(input_mem_);
        bindings_.emplace_back(output_mem_);
    }

    context_ = std::shared_ptr<nvinfer1::IExecutionContext>(engine_->createExecutionContext());
    if (!context_) {
        // return false;
        throw std::runtime_error("Failed to create execution context");
    }

    context_->setTensorAddress(engine_->getIOTensorName(0), input_mem_);
    context_->setTensorAddress(engine_->getIOTensorName(engine_->getNbIOTensors() - 1),
                               output_mem_);

    std::cout << "det model initialization, done!" << std::endl;
}

booster_vision::Detections YoloV8DetectorTRT::proceed(uint8_t* img_raw) {
    cv::Mat img = cv::Mat(CAMERA_HEIGHT, CAMERA_WIDTH, CV_8UC4, img_raw);

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
    img.u = nullptr;  // prevent cv::Mat from freeing the data
    return detections;
}

std::vector<booster_vision::DetectionRes> YoloV8DetectorTRT::Inference(const cv::Mat& img) {
    static int inference_count = 0;
    static int64_t preprocess_time_acc = 0;
    static int64_t inference_exec_time_acc = 0;
    static int64_t postprocess_time_acc = 0;
    static int64_t total_time_acc = 0;
    static int64_t last_inference_log_time = time_us();

    int64_t total_start_time = time_us();
    std::vector<float> factors;
    int64_t start_time = time_us();
    if (!PreProcess(img, factors)) {
        return {};
    }
    int64_t preprocess_time = time_us() - start_time;
    preprocess_time_acc += preprocess_time;

    // Memcpy from host input buffers to device input buffers
    MemcpyBuffers(input_mem_, input_buff_, input_size_ * sizeof(float), cudaMemcpyHostToDevice,
                  async_infer_);

    start_time = time_us();
    bool status = false;
    if (async_infer_) {
        status = context_->enqueueV3(stream_);
    } else {
        status = context_->executeV2(bindings_.data());
    }

    if (!status) {
        return {};
    }

    if (async_infer_) {
        cudaStreamSynchronize(stream_);
    }
    // Memcpy from device output buffers to host output buffers
    MemcpyBuffers(output_buff_, output_mem_, output_size_ * sizeof(float), cudaMemcpyDeviceToHost,
                  async_infer_);

    int64_t inference_exec_time = time_us() - start_time;
    inference_exec_time_acc += inference_exec_time;

    img_width_ = img.cols;
    img_height_ = img.rows;
    start_time = time_us();
    std::vector<booster_vision::DetectionRes> outputs = PostProcess(factors);
    int64_t postprocess_time = time_us() - start_time;
    postprocess_time_acc += postprocess_time;

    int64_t total_time = time_us() - total_start_time;
    total_time_acc += total_time;

    inference_count++;
    int64_t now = time_us();
    if (now - last_inference_log_time > 1'000'000) {
        if (inference_count > 0) {
            LOG_F(1,
                  "yolov8 avg times (ms): total: %.2f, preprocess: %.2f, inference_exec: %.2f, "
                  "postprocess: %.2f, fps: %d",
                  total_time_acc / (float)inference_count / 1000.f,
                  preprocess_time_acc / (float)inference_count / 1000.f,
                  inference_exec_time_acc / (float)inference_count / 1000.f,
                  postprocess_time_acc / (float)inference_count / 1000.f, inference_count);
        }
        last_inference_log_time += 1'000'000;
        inference_count = 0;
        preprocess_time_acc = 0;
        inference_exec_time_acc = 0;
        postprocess_time_acc = 0;
        total_time_acc = 0;
    }

    return outputs;
}

YoloV8DetectorTRT::~YoloV8DetectorTRT() {
    cudaStreamDestroy(stream_);
    cudaFree(input_mem_);
    cudaFree(output_mem_);
    free(input_buff_);
    free(output_buff_);
}

bool YoloV8DetectorTRT::LoadEngine() {
    std::ifstream input(model_path_, std::ios::binary);
    if (!input) {
        return false;
    }
    input.seekg(0, input.end);
    const size_t fsize = input.tellg();
    input.seekg(0, input.beg);
    std::vector<char> bytes(fsize);
    input.read(bytes.data(), fsize);

    runtime_ = std::shared_ptr<nvinfer1::IRuntime>(createInferRuntime(logger));
    engine_ = std::shared_ptr<nvinfer1::ICudaEngine>(
            runtime_->deserializeCudaEngine(bytes.data(), bytes.size()), InferDeleter());
    if (!engine_)
        return false;

    int nbio = engine_->getNbIOTensors();
    const char* inputname = engine_->getIOTensorName(0);
    const char* outputname = engine_->getIOTensorName(engine_->getNbIOTensors() - 1);
    Dims input_shape = engine_->getTensorShape(inputname);
    Dims output_shape = engine_->getTensorShape(outputname);
    model_input_dims_ =
            Dims4(input_shape.d[0], input_shape.d[1], input_shape.d[2], input_shape.d[3]);
    model_output_dims_ =
            Dims4(output_shape.d[0], output_shape.d[1], output_shape.d[2], output_shape.d[3]);
    std::cout << "model input dims: " << input_shape.d[0] << " " << input_shape.d[1] << " "
              << input_shape.d[2] << " " << input_shape.d[3] << std::endl;
    std::cout << "model output dims: " << output_shape.d[0] << " " << output_shape.d[1] << " "
              << output_shape.d[2] << std::endl;

    return true;
}

bool YoloV8DetectorTRT::PreProcess(const cv::Mat& img, std::vector<float>& factors) {
#ifdef DROBOTICS_CAMERA
    if (img.cols != 544 || img.rows != 448) {
        LOG_F(ERROR, "Image size is not 544x448, but %dx%d", img.cols, img.rows);
        exit(1);
    }
#else
    if (img.cols != 1280 || img.rows > 800) {
        LOG_F(ERROR, "Image size is not 1280x800, but %dx%d", img.cols, img.rows);
        exit(1);
    }
#endif
    if (img.channels() != 4) {
        LOG_F(ERROR, "Image channels is not 4, but %d", img.channels());
        exit(1);
    }

    const int target_w = model_input_dims_.d[3];
    const int target_h = model_input_dims_.d[2];

    const int src_w = img.cols;
    const int src_h = img.rows;
    // We are scaling 1280x800 to fit in 640x400.
    // The scale factor is 640/1280 = 0.5.
    // The new image dimensions will be 1280*0.5=640 and 800*0.5=400.
    // If the original image is less than 800 in height. We use letterboxing
    // and fill the missing bottom rows with black.
    const int new_w = 640;
    const int new_h = 400;

    // factors are always 0.5 with the current downscaling.
    factors.clear();
#ifdef DROBOTICS_CAMERA
    factors.emplace_back(1);
    factors.emplace_back(1);
#else
    // source width is always 1280
    factors.emplace_back(static_cast<float>(src_w) / new_w);
    // source height can differ. Pixel coordinates are in range,
    // because bottom rows are filled with 0
    factors.emplace_back(static_cast<float>(new_h * 2) / new_h);
#endif

    float* r_plane = input_buff_;
    float* g_plane = input_buff_ + target_w * target_h;
    float* b_plane = input_buff_ + 2 * target_w * target_h;

    const uchar* src_data = img.data;

#ifdef DROBOTICS_CAMERA
    int lb_w = (640 - 544) / 2;
    for (int y = 0; y < new_h; ++y) {
        for (int x = 0; x < lb_w; ++x) {
            int dst_idx = y * target_w + x;
            r_plane[dst_idx] = 0;
            g_plane[dst_idx] = 0;
            b_plane[dst_idx] = 0;
        }
        for (int x = lb_w; x < lb_w + 544; ++x) {
            int dst_idx = y * target_w + x;
            int src_x = x - lb_w;
            // leave out the first 48 rows because we can only fit 400px in height
            int src_y = y + 48;
            r_plane[dst_idx] = src_data[(src_y * 544 + src_x) * 4] * (1.f / 255.f);;
            g_plane[dst_idx] = src_data[(src_y * 544 + src_x) * 4 + 1] * (1.f / 255.f);;
            b_plane[dst_idx] = src_data[(src_y * 544 + src_x) * 4 + 2] * (1.f / 255.f);;
        }
        for (int x = lb_w + 544; x < new_w; ++x) {
            int dst_idx = y * target_w + x;
            r_plane[dst_idx] = 0;
            g_plane[dst_idx] = 0;
            b_plane[dst_idx] = 0;
        }
    }
#else
    // Fill the output buffer for the image area
    for (int y = 0; y < new_h; ++y) {
        for (int x = 0; x < new_w; ++x) {
            // Point in source is (x*2, y*2). We do linear interpolation by averaging a 2x2 block.
            int src_x = x * 2;
            int src_y = y * 2;

            if (src_y >= CAMERA_WIDTH) {
                int dst_idx = y * target_w + x;
                r_plane[dst_idx] = 0;
                g_plane[dst_idx] = 0;
                b_plane[dst_idx] = 0;
            } else {
                const uchar* p1 = src_data + (src_y * src_w + src_x) * 4;
                const uchar* p2 = src_data + (src_y * src_w + src_x + 1) * 4;
                const uchar* p3 = src_data + ((src_y + 1) * src_w + src_x) * 4;
                const uchar* p4 = src_data + ((src_y + 1) * src_w + src_x + 1) * 4;

                // BGR is the layout in the source image
                float b_avg = ((p1[0] >> 2) + (p2[0] >> 2) + (p3[0] >> 2) + (p4[0] >> 2));
                float g_avg = ((p1[1] >> 2) + (p2[1] >> 2) + (p3[1] >> 2) + (p4[1] >> 2));
                float r_avg = ((p1[2] >> 2) + (p2[2] >> 2) + (p3[2] >> 2) + (p4[2] >> 2));

                int dst_idx = y * target_w + x;
                r_plane[dst_idx] = b_avg * (1.f / 255.f);
                g_plane[dst_idx] = g_avg * (1.f / 255.f);
                b_plane[dst_idx] = r_avg * (1.f / 255.f);
            }
        }
    }
#endif

    return true;
}

std::vector<booster_vision::DetectionRes> YoloV8DetectorTRT::PostProcess(
        const std::vector<float>& factors) {
    const int outputSize = model_output_dims_.d[1];
    // float* output = static_cast<float*>(output_buff_);
    cv::Mat outputs(outputSize, 8400, CV_32F, output_buff_);

    std::vector<int> class_ids;
    std::vector<float> confidences;
    std::vector<cv::Rect> boxes;
    // Preprocessing output results
    const int class_num = outputSize - 4;  // 4 for box[x,y,w,h]
    int rows = outputs.size[0];
    int dimensions = outputs.size[1];
    bool yolov8 = false;

    // yolov5 has an output of shape (batchSize, 25200, 85) (Num classes + box[x,y,w,h] +
    // confidence[c]) yolov8 has an output of shape (batchSize, 84,  8400) (Num classes +
    // box[x,y,w,h])
    if (dimensions > rows)  // Check if the shape[2] is more than shape[1] (yolov8)
    {
        yolov8 = true;
        rows = outputs.size[1];
        dimensions = outputs.size[0];

        outputs = outputs.reshape(1, dimensions);
        cv::transpose(outputs, outputs);
    }

    float* data = (float*)outputs.data;
    for (int i = 0; i < rows; ++i) {
        float* classes_scores = data + 4;

        cv::Mat scores(1, class_num - 1, CV_32FC1, classes_scores + 1);
        // std::cout << "scores: " << scores << std::endl;
        cv::Point class_id;
        double maxClassScore;
        int ball_id = 0;
        double ballClassScore;

        ballClassScore = classes_scores[0];

        minMaxLoc(scores, 0, &maxClassScore, 0, &class_id);

        if (maxClassScore > confidence_) {
#ifdef DROBOTICS_CAMERA
            float x = data[0] - (640 - 544) / 2;
            float y = data[1] + 48;
            float w = data[2];
            float h = data[3];
#else
            float x = data[0];
            float y = data[1];
            float w = data[2];
            float h = data[3];
#endif

            int left = int((x - 0.5 * w) * factors[0]);
            int top = int((y - 0.5 * h) * factors[1]);

            int width = int(w * factors[0]);
            int height = int(h * factors[1]);

            if (left < 0 || left > img_width_ - 1)
                continue;
            if (top < 0 || top > img_height_ - 1)
                continue;

            int right = std::min(img_width_ - 1, left + width);
            int bottom = std::min(img_height_ - 1, top + height);
            width = right - left;
            height = bottom - top;
            if (width < 3 || height < 3)
                continue;

            boxes.push_back(cv::Rect(left, top, width, height));
            confidences.push_back(maxClassScore);
            class_ids.push_back(class_id.x + 1);
        }
        if (ballClassScore > confidence_) {
#ifdef DROBOTICS_CAMERA
            float x = data[0] - (640 - 544) / 2;
            float y = data[1] + 48;
            float w = data[2];
            float h = data[3];
#else
            float x = data[0];
            float y = data[1];
            float w = data[2];
            float h = data[3];
#endif

            int left = int((x - 0.5 * w) * factors[0]);
            int top = int((y - 0.5 * h) * factors[1]);

            int width = int(w * factors[0]);
            int height = int(h * factors[1]);

            if (left < 0 || left > img_width_ - 1)
                continue;
            if (top < 0 || top > img_height_ - 1)
                continue;

            int right = std::min(img_width_ - 1, left + width);
            int bottom = std::min(img_height_ - 1, top + height);
            width = right - left;
            height = bottom - top;
            if (width < 3 || height < 3)
                continue;

            boxes.push_back(cv::Rect(left, top, width, height));
            confidences.push_back(ballClassScore);
            class_ids.push_back(ball_id);
        }

        data += dimensions;
    }
    std::vector<int> no_ball_nms_result;
    std::vector<int> ball_nms_result;
    // creat extra ball boxes
    std::vector<int> ball_class_ids;
    std::vector<float> ball_confidences;
    std::vector<cv::Rect> ball_boxes;
    std::vector<int> no_ball_class_ids;
    std::vector<float> no_ball_confidences;
    std::vector<cv::Rect> no_ball_boxes;

    // devide everything into ball and no_ball
    for (size_t i = 0; i < class_ids.size(); ++i) {
        if (class_ids[i] == 0) {  // ball
            ball_class_ids.push_back(class_ids[i]);
            ball_confidences.push_back(confidences[i]);
            ball_boxes.push_back(boxes[i]);
        } else {  // no_ball
            no_ball_class_ids.push_back(class_ids[i]);
            no_ball_confidences.push_back(confidences[i]);
            no_ball_boxes.push_back(boxes[i]);
        }
    }

    cv::dnn::NMSBoxes(no_ball_boxes, no_ball_confidences, 0.25, 0.4, no_ball_nms_result);
    cv::dnn::NMSBoxes(ball_boxes, ball_confidences, 0.25, 0.4, ball_nms_result);

    std::vector<booster_vision::DetectionRes> detections{};
    for (unsigned long i = 0; i < no_ball_nms_result.size(); ++i) {
        int idx = no_ball_nms_result[i];

        booster_vision::DetectionRes result;
        result.class_id = no_ball_class_ids[idx];
        result.confidence = no_ball_confidences[idx];
        result.bbox = no_ball_boxes[idx];

        detections.push_back(result);
    }
    for (unsigned long i = 0; i < ball_nms_result.size(); ++i) {
        int idx = ball_nms_result[i];

        booster_vision::DetectionRes result;
        result.class_id = ball_class_ids[idx];
        result.confidence = ball_confidences[idx];
        result.bbox = ball_boxes[idx];

        detections.push_back(result);
    }
    return detections;
}

void YoloV8DetectorTRT::MemcpyBuffers(void* dstPtr, void const* srcPtr, size_t byteSize,
                                      cudaMemcpyKind memcpyType, bool const async) {
    if (async) {
        cudaMemcpyAsync(dstPtr, srcPtr, byteSize, memcpyType, stream_);
    } else {
        cudaMemcpy(dstPtr, srcPtr, byteSize, memcpyType);
    }
}
