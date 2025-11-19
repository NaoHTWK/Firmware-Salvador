// #include <turbojpeg.h>

#include <cmath>
#include <fstream>
#include <map>
#include <opencv2/opencv.hpp>
#include <rerun.hpp>
#include <string>

#include "logging.h"

constexpr float vision_scale_factor = 0.25f;

namespace htwk {

void log_image_vision(std::string_view entity_path, cv::Mat& image) {
    // res realsense 16:10
    // res zed2 16:9
    cv::Mat small_image;
    // 640 by 360
    // check the pixel format of the image

    cv::resize(cv::InputArray(image), cv::OutputArray(small_image), cv::Size(), vision_scale_factor,
               vision_scale_factor, cv::INTER_NEAREST);
    rerun::RecordingStream& online_stream = logging_utils::rerun_online_stream();
    std::vector<uint8_t> image_data(
            small_image.data, small_image.data + (small_image.total() * small_image.elemSize()));
    rerun::WidthHeight resolution{static_cast<uint32_t>(small_image.cols),
                                  static_cast<uint32_t>(small_image.rows)};

    switch (image.type()) {
        case CV_32FC1:  // 32-bit float Grayscale
            LOG_F(ERROR, "CV_32FC1 is not supported");
            return;
        case CV_32FC2:  // 32-bit float RGB
            LOG_F(ERROR, "CV_32FC2 is not supported");
            return;
        case CV_32FC3:  // 32-bit float RGB
            LOG_F(ERROR, "CV_32FC3 is not supported");
            return;
        case CV_32FC4:  // 32-bit float RGBA
            LOG_F(ERROR, "CV_32FC4 is not supported");
            return;

        case CV_8UC1: {  // Grayscale
            cv::Mat rgba_image;
            cv::cvtColor(cv::_InputArray(small_image), cv::OutputArray(rgba_image),
                         cv::COLOR_GRAY2RGBA);
            std::vector<uint8_t> rgba_bytes(
                    rgba_image.data,
                    rgba_image.data + (rgba_image.total() * rgba_image.elemSize()));
            online_stream.log(entity_path, rerun::Image::from_rgba32(rgba_bytes, resolution));
            break;
        }
        case CV_8UC2:  // 8-bit unsigned char RGB
            LOG_F(ERROR, "CV_8UC2 is not supported");
            break;
        case CV_8UC3:
            LOG_F(ERROR, "CV_8UC3 is not supported");
            return;
        case CV_8UC4: {  // BGRA
            online_stream.log(entity_path, rerun::Image::from_rgba32(image_data, resolution));
            break;
        }
        default:
            LOG_F(ERROR, "Unknown image type: %d", image.type());
            return;
    }
}

void log_image_vision_bgra(std::string_view entity_path, uint8_t* bgra_data, int width, int height){
    cv::Mat image(height, width, CV_8UC4, bgra_data);
    log_image_vision(entity_path, image);
}

void log_vision(std::string_view entity_path) {
    rerun::RecordingStream& online_stream = logging_utils::rerun_online_stream();
    rerun::RecordingStream& file_stream = logging_utils::rerun_file_stream();

    online_stream.log(entity_path, rerun::Clear::FLAT);
    file_stream.log(entity_path, rerun::Clear::FLAT);
}

void log_vision_lines(std::string_view entity_path,
                      const std::vector<rerun::LineStrip2D>& line_strips, float radius,
                      const rerun::Color& color, const std::vector<std::string>& labels) {
    rerun::RecordingStream& online_stream = logging_utils::rerun_online_stream();
    rerun::RecordingStream& file_stream = logging_utils::rerun_file_stream();

    std::vector<rerun::LineStrip2D> scaled_line_strips;
    scaled_line_strips.reserve(line_strips.size());

    for (const auto& line_strip : line_strips) {
        rerun::LineStrip2D line({{line_strip.points[0].x() * vision_scale_factor,
                                  line_strip.points[0].y() * vision_scale_factor},
                                 {line_strip.points[1].x() * vision_scale_factor,
                                  line_strip.points[1].y() * vision_scale_factor}});

        scaled_line_strips.push_back(line);
    }

    // Log scaled version to online stream
    online_stream.log(entity_path, rerun::LineStrips2D(scaled_line_strips)
                                           .with_radii(rerun::Radius::ui_points(radius))
                                           .with_colors(color)
                                           .with_labels(labels));

    // Log original version to file stream
    file_stream.log(entity_path, rerun::LineStrips2D(line_strips)
                                         .with_radii(rerun::Radius::ui_points(radius))
                                         .with_colors(color)
                                         .with_labels(labels));
};

void log_vision_points(std::string_view entity_path, const std::vector<rerun::Position2D>& points,
                       float radius, const rerun::Color& color,
                       const std::vector<std::string>& labels) {

    std::vector<rerun::Position2D> scaled_points;
    for (rerun::Position2D point : points) {
        scaled_points.emplace_back(point.x() * vision_scale_factor,
                                   point.y() * vision_scale_factor);
    }

    rerun::RecordingStream& online_stream = logging_utils::rerun_online_stream();
    rerun::RecordingStream& file_stream = logging_utils::rerun_file_stream();

    online_stream.log(entity_path, rerun::Points2D(scaled_points)
                                           .with_radii(rerun::Radius::ui_points(radius))
                                           .with_colors(color)
                                           .with_labels(labels));
    file_stream.log(entity_path, rerun::Points2D(points)
                                         .with_radii(rerun::Radius::ui_points(radius))
                                         .with_colors(color)
                                         .with_labels(labels));
};

void log_vision_boxes_from_center_and_sizes(std::string_view entity_path,
                                            const std::vector<std::array<float, 4>>& coordinates,
                                            float radius, const rerun::Color& color,
                                            const std::vector<std::string>& labels) {

    // std::vector<std::array<float, 4>> coordinates_scaled;
    std::vector<rerun::datatypes::Vec2D> mins;
    std::vector<rerun::datatypes::Vec2D> sizes;
    std::vector<rerun::datatypes::Vec2D> mins_scaled;
    std::vector<rerun::datatypes::Vec2D> sizes_scaled;
    for (const auto& box : coordinates) {
        mins.emplace_back(box[0], box[1]);
        sizes.emplace_back(box[2], box[3]);
        mins_scaled.emplace_back(box[0] * vision_scale_factor, box[1] * vision_scale_factor);
        sizes_scaled.emplace_back(box[2] * vision_scale_factor, box[3] * vision_scale_factor);
    }

    rerun::Boxes2D scaled_boxes = rerun::Boxes2D::from_mins_and_sizes(mins_scaled, sizes_scaled)
                                          .with_colors(color)
                                          .with_labels(labels)
                                          .with_radii(rerun::Radius::ui_points(radius));

    rerun::Boxes2D original_boxes = rerun::Boxes2D::from_mins_and_sizes(mins, sizes)
                                            .with_colors(color)
                                            .with_labels(labels)
                                            .with_radii(rerun::Radius::ui_points(radius));

    rerun::RecordingStream& online_stream = logging_utils::rerun_online_stream();
    rerun::RecordingStream& file_stream = logging_utils::rerun_file_stream();

    online_stream.log(entity_path, scaled_boxes);
    file_stream.log(entity_path, original_boxes);
}

void log_vision_OpencvRect(std::string_view entity_path, std::vector<cv::Rect> rects, float radius,
                           const rerun::Color& color, const std::vector<std::string>& labels,
                           bool show_labels) {
    std::vector<rerun::datatypes::Vec2D> mins;
    std::vector<rerun::datatypes::Vec2D> sizes;
    std::vector<rerun::datatypes::Vec2D> mins_scaled;
    std::vector<rerun::datatypes::Vec2D> sizes_scaled;
    for (auto box : rects) {
        mins.emplace_back(box.x, box.y);
        sizes.emplace_back(box.width, box.height);
        mins_scaled.emplace_back(box.x * vision_scale_factor, box.y * vision_scale_factor);
        sizes_scaled.emplace_back(box.width * vision_scale_factor,
                                  box.height * vision_scale_factor);
    }

    rerun::Boxes2D scaled_boxes = rerun::Boxes2D::from_mins_and_sizes(mins_scaled, sizes_scaled)
                                          .with_colors(color)
                                          .with_labels(labels)
                                          .with_show_labels(show_labels)
                                          .with_radii(rerun::Radius::ui_points(radius));

    rerun::Boxes2D original_boxes = rerun::Boxes2D::from_mins_and_sizes(mins, sizes)
                                            .with_colors(color)
                                            .with_labels(labels)
                                            .with_show_labels(show_labels)
                                            .with_radii(rerun::Radius::ui_points(radius));

    rerun::RecordingStream& online_stream = logging_utils::rerun_online_stream();
    rerun::RecordingStream& file_stream = logging_utils::rerun_file_stream();

    online_stream.log(entity_path, scaled_boxes);
    file_stream.log(entity_path, original_boxes);
}

void log_encoded_image_jpeg(std::string_view entity_path, cv::Mat& image, int jpeg_quality_online,
                            int jpeg_quality_file, std::optional<float> online_scale_factor) {

    // Use default scale factor if not provided
    float actual_scale_factor = online_scale_factor.value_or(vision_scale_factor);

    // Prepare the image for encoding
    cv::Mat online_image;
    if (online_scale_factor != 1.0f) {
        cv::resize(image, online_image, cv::Size(), actual_scale_factor, actual_scale_factor,
                   cv::INTER_LINEAR);
    } else {
        online_image = image;
    }

    // Convert to BGR format for OpenCV JPEG encoding (JPEG doesn't support alpha)
    cv::Mat online_bgr_image;
    cv::Mat file_bgr_image;
    if (online_image.channels() == 4) {
        // RGBA -> BGR (remove alpha channel)
        cv::cvtColor(online_image, online_bgr_image, cv::COLOR_RGBA2BGR);
    } else if (online_image.channels() == 3) {
        // Already BGR format, use as-is
        online_bgr_image = online_image;
    } else {
        LOG_F(ERROR, "Unsupported image format with %d channels", online_image.channels());
        return;
    }
    if (image.channels() == 4) {
        // RGBA -> BGR (remove alpha channel)
        cv::cvtColor(image, file_bgr_image, cv::COLOR_RGBA2BGR);
    } else if (image.channels() == 3) {
        // Already BGR format, use as-is
        file_bgr_image = image;
    } else {
        LOG_F(ERROR, "Unsupported image format with %d channels", image.channels());
        return;
    }

    // Ensure image is continuous in memory
    if (!file_bgr_image.isContinuous()) {
        file_bgr_image = file_bgr_image.clone();
    }
    if (!online_bgr_image.isContinuous()) {
        online_bgr_image = online_bgr_image.clone();
    }

    // Validate image dimensions
    if (file_bgr_image.empty() || file_bgr_image.cols <= 0 || file_bgr_image.rows <= 0) {
        LOG_F(ERROR, "Invalid image dimensions: %dx%d", file_bgr_image.cols, file_bgr_image.rows);
        return;
    }
    if (online_bgr_image.empty() || online_bgr_image.cols <= 0 || online_bgr_image.rows <= 0) {
        LOG_F(ERROR, "Invalid image dimensions: %dx%d", online_bgr_image.cols,
              online_bgr_image.rows);
        return;
    }

    // Use OpenCV's JPEG encoding
    std::vector<uchar> jpeg_buffer_online;
    std::vector<uchar> jpeg_buffer_file;
    std::vector<int> compression_params_online;
    std::vector<int> compression_params_file;
    compression_params_online.push_back(cv::IMWRITE_JPEG_QUALITY);
    compression_params_online.push_back(jpeg_quality_online);
    compression_params_file.push_back(cv::IMWRITE_JPEG_QUALITY);
    compression_params_file.push_back(jpeg_quality_file);

    // Encode as JPEG
    bool success =
            cv::imencode(".jpg", online_bgr_image, jpeg_buffer_online, compression_params_online);

    if (!success || jpeg_buffer_online.empty()) {
        LOG_F(ERROR, "JPEG online encoding failed");
        return;
    }
    success = cv::imencode(".jpg", file_bgr_image, jpeg_buffer_file, compression_params_file);
    if (!success || jpeg_buffer_file.empty()) {
        LOG_F(ERROR, "JPEG file encoding failed");
        return;
    }

    // Create Rerun collection from JPEG buffer
    rerun::Collection<uint8_t> jpeg_data_online(jpeg_buffer_online);
    rerun::Collection<uint8_t> jpeg_data_file(jpeg_buffer_file);

    // Create EncodedImage with JPEG media type
    rerun::EncodedImage encoded_image_online = rerun::EncodedImage::from_bytes(
            jpeg_data_online, rerun::components::MediaType("image/jpeg"));
    rerun::EncodedImage encoded_image_file = rerun::EncodedImage::from_bytes(
            jpeg_data_file, rerun::components::MediaType("image/jpeg"));

    // Log to both streams
    rerun::RecordingStream& online_stream = logging_utils::rerun_online_stream();
    rerun::RecordingStream& file_stream = logging_utils::rerun_file_stream();

    online_stream.log(entity_path, encoded_image_online);
    file_stream.log(entity_path, encoded_image_file);
}

void log_encoded_image_jpeg_raw(std::string_view entity_path, uint8_t* bgra, int jpeg_quality_online,
                            int jpeg_quality_file, std::optional<float> online_scale_factor) {
    cv::Mat image(CAMERA_HEIGHT, CAMERA_WIDTH, CV_8UC4, bgra);
    log_encoded_image_jpeg(entity_path, image, jpeg_quality_online, jpeg_quality_file, online_scale_factor);
}
}  // namespace htwk
