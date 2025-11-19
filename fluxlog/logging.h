#pragma once
#include <soccerfield.h>

#include <cstdint>  // Include standard integer types as a fallback.
#include <loguru.hpp>
#include <optional>
#include <rerun.hpp>
#include <string>

#include "order.h"

namespace loguru {
extern loguru::NamedVerbosity Verbosity_SPARSE_INFO;
extern loguru::NamedVerbosity Verbosity_SENSOR;
extern loguru::NamedVerbosity Verbosity_DEBUG;
extern loguru::NamedVerbosity Verbosity_TRACE;
extern loguru::NamedVerbosity Verbosity_OFFLINE;
}  // namespace loguru

namespace htwk::logging_utils {
rerun::RecordingStream& rerun_file_stream();
rerun::RecordingStream& rerun_online_stream();
}  // namespace htwk::logging_utils

namespace htwk {
constexpr float SENSOR_LOGGING_THRESHOLD = 0.3f;  // seconds
constexpr float SPARSE_INFO_LOGGING_THRESHOLD = 1.0f;
// how to use sensor logging:
//  LOG_S(SENSOR_LOGGING) << "{Sensor_data}: " << sensor_data;
//  now every string with that starts with {Sensor_data} has an SENSOR_LOGGING_THRESHOLD second
//  threshold bevor the next message is send to the rerun server. Messages will be discarded if they
//  are send in the threshold time. Dont use ":" in the string, it will be used as a separator

void init_logging(int argc, char** argv, const std::string& rerun_ip = "");

rerun::Quaternion euler_to_quaternion(float roll, float pitch, float yaw);

template <typename... Ts>
void log_rerun(std::string_view entity_path, const Ts&... as_components) {
    rerun::RecordingStream& online_stream = logging_utils::rerun_online_stream();
    online_stream.log(entity_path, as_components...);
    rerun::RecordingStream& file_stream = logging_utils::rerun_file_stream();
    file_stream.log(entity_path, as_components...);
}

template <typename... Ts>
void log_rerun_online(std::string_view entity_path, const Ts&... as_components) {
    rerun::RecordingStream& online_stream = logging_utils::rerun_online_stream();
    online_stream.log(entity_path, as_components...);
}
template <typename... Ts>
void log_rerun_file(std::string_view entity_path, const Ts&... as_components) {
    rerun::RecordingStream& file_stream = logging_utils::rerun_file_stream();
    file_stream.log(entity_path, as_components...);
}

template <typename... Ts>
void log_rerun_static(std::string_view entity_path, const Ts&... as_components) {
    rerun::RecordingStream& online_stream = logging_utils::rerun_online_stream();
    online_stream.log_static(entity_path, as_components...);
    rerun::RecordingStream& file_stream = logging_utils::rerun_file_stream();
    file_stream.log_static(entity_path, as_components...);
}

template <typename... Ts>
void log_rerun_static_online(std::string_view entity_path, const Ts&... as_components) {
    rerun::RecordingStream& online_stream = logging_utils::rerun_online_stream();
    online_stream.log_static(entity_path, as_components...);
}
template <typename... Ts>
void log_rerun_static_file(std::string_view entity_path, const Ts&... as_components) {
    rerun::RecordingStream& file_stream = logging_utils::rerun_file_stream();
    file_stream.log_static(entity_path, as_components...);
}

void log_soccer_field();

void log_order(const Order& order, std::string info, std::string file, int line);

void log_robot_position(std::string entity_path, const htwk::Position& position, float head_yaw,
                        const rerun::Color& color, std::string info = "");

void log_time(uint64_t frame_number);
}  // namespace htwk
