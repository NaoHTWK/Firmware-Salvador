#pragma once
#include <cstdint>  // Include standard integer types as a fallback.
#include <loguru.hpp>
#include <opencv2/opencv.hpp>
#include <optional>
#include <rerun.hpp>
#include <string>

namespace htwk {

void log_image_vision_bgra(std::string_view entity_path, uint8_t* bgra_data, int width, int height);
//void log_image_vision(std::string_view entity_path, cv::Mat& image);
void log_vision(std::string_view entity_path);
void log_vision_lines(std::string_view entity_path,
                      const std::vector<rerun::LineStrip2D>& line_strips, float radius = 2.0f,
                      const rerun::Color& color = rerun::Color(255, 0, 0, 255),
                      const std::vector<std::string>& labels = {});

void log_vision_points(std::string_view entity_path, const std::vector<rerun::Position2D>& points,
                       float radius = 2.0f,
                       const rerun::Color& color = rerun::Color(255, 0, 0, 255),
                       const std::vector<std::string>& labels = {});

void log_vision_boxes_from_center_and_sizes(
        std::string_view entity_path, std::vector<std::array<float, 4>>& coordinates,
        float radius = 2.0f, const rerun::Color& color = rerun::Color(255, 0, 0, 255),
        const std::vector<std::string>& labels = {});

void log_vision_OpencvRect(std::string_view entity_path, std::vector<cv::Rect> rects,
                           float radius = 2.0f,
                           const rerun::Color& color = rerun::Color(255, 0, 0, 255),
                           const std::vector<std::string>& labels = {}, bool show_labels = true);

/// Log a JPEG-encoded image using libjpeg-turbo for efficient compression
/// @param entity_path The Rerun entity path to log the image to
/// @param image OpenCV Mat containing the image data (BGR or BGRA format)
/// @param jpeg_quality JPEG compression quality (1-100, higher = better quality)
/// @param scale_factor Scale factor to apply before encoding (default uses vision_scale_factor)
void log_encoded_image_jpeg(std::string_view entity_path, cv::Mat& image,
                            int jpeg_quality_online = 80, int jpeg_quality_file = 99,
                            std::optional<float> online_scale_factor = {});

void log_encoded_image_jpeg_raw(std::string_view entity_path, uint8_t* bgra, int jpeg_quality_online = 80,
                            int jpeg_quality_file = 99, std::optional<float> online_scale_factor = {});
}  // namespace htwk
