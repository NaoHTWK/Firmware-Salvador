#include "logging.h"

#include <cmath>
#include <filesystem>
#include <fstream>
#include <iostream>
#include <map>
#include <rerun.hpp>
#include <string>
#include <vector>

#include "robot_time.h"

loguru::NamedVerbosity loguru::Verbosity_DEBUG = loguru::Verbosity_1;
loguru::NamedVerbosity loguru::Verbosity_TRACE = loguru::Verbosity_2;
loguru::NamedVerbosity loguru::Verbosity_OFFLINE = loguru::Verbosity_3;
loguru::NamedVerbosity loguru::Verbosity_SENSOR = loguru::Verbosity_4;
loguru::NamedVerbosity loguru::Verbosity_SPARSE_INFO = loguru::Verbosity_5;

namespace htwk::logging_utils {

static std::string rerun_ip_address = "";

void set_rerun_ip(const std::string& ip) {
    rerun_ip_address = ip;
}

std::string loguru_format_mock_verbosity_5(const loguru::Message& m, int dropped_messages) {
    // Nutze einfach preamble + message, wie Loguru es tut:
    std::ostringstream oss;
    oss << m.preamble << "dropped:" << dropped_messages << "|" << m.message;
    return oss.str();
}

std::string init_logging_folder() {
    // Get current time and format as yyyy-mm-dd-hh-mm-ss
    auto now = std::chrono::system_clock::now();
    auto time_t = std::chrono::system_clock::to_time_t(now);
    auto tm = std::localtime(&time_t);
    char time_str[20];
    std::strftime(time_str, sizeof(time_str), "%Y-%m-%d-%H-%M-%S", tm);

#ifdef ROBOT_MODEL_T1
    static const std::string logging_folder =
            "/home/booster/Workspace/logs/" + std::string(time_str);
#elif ROBOT_MODEL_K1
    static const std::string logging_folder = "/home/booster/logs/" + std::string(time_str);
#else
    static const std::string logging_folder = "/tmp/logs/" + std::to_string(time_us());
#endif

    if (!std::filesystem::exists(logging_folder)) {
        std::filesystem::create_directories(logging_folder);
    }

    // set metadata for logging
    std::ofstream metadata_file(logging_folder + "/metadata.txt");
    if (!metadata_file.is_open()) {
        std::cerr << "Could not open metadata file for writing: "
                  << logging_folder + "/metadata.txt" << std::endl;
        exit(1);
    }

    auto timestamp_ns =
            std::chrono::duration_cast<std::chrono::nanoseconds>(now.time_since_epoch()).count();
    std::string metadata = "timestamp " + std::to_string(timestamp_ns) + "\n" + "rerun_version " +
                           std::string(RERUN_VERSION) + "\n";

    metadata_file << metadata;
    metadata_file.close();

    return logging_folder;
}

std::string& logging_path() {
    static std::string logging_path_ = init_logging_folder();
    return logging_path_;
}

rerun::RecordingStream& init_online_stream() {
    static rerun::RecordingStream rerun_online_stream_("fw_salvador_online");
    std::string path = logging_path() + "/online_stream.rrd";

    std::string server_ip = rerun_ip_address.empty() ? "127.0.0.1" : rerun_ip_address;
    std::string server_url = "rerun+http://" + server_ip + "/proxy";
    printf(server_url.c_str());
    rerun_online_stream_.connect_grpc(server_url).exit_on_failure();
    return rerun_online_stream_;
}

rerun::RecordingStream& rerun_online_stream() {
    static rerun::RecordingStream& rerun_online_stream_ = init_online_stream();
    return rerun_online_stream_;
}

rerun::RecordingStream& init_file_stream() {
    static rerun::RecordingStream rerun_file_stream_("fw_salvador_file");
    std::string path = logging_path() + "/file_stream.rrd";
    rerun::Error error = rerun_file_stream_.save(path);
    if (!error.is_ok()) {
        std::cerr << "Error saving rerun file stream: " << error.description << std::endl;
        exit(1);
    }
    return rerun_file_stream_;
}

rerun::RecordingStream& rerun_file_stream() {
    static rerun::RecordingStream& rerun_file_stream_ = init_file_stream();
    return rerun_file_stream_;
}

std::map<std::string, std::chrono::time_point<std::chrono::steady_clock>>
get_last_message_verbosity4() {
    static std::map<std::string, std::chrono::time_point<std::chrono::steady_clock>> map;
    return map;
}

std::map<std::string, std::chrono::time_point<std::chrono::steady_clock>>
get_last_message_verbosity5() {
    static std::map<std::string, std::chrono::time_point<std::chrono::steady_clock>> map;
    return map;
}

std::map<std::string, int>& get_dropped_messages_verbosity5() {
    static std::map<std::string, int> dropped_messages;
    return dropped_messages;
}

void loguru_to_rerun(void*, const loguru::Message& message) {
    bool should_log_online = true;
    bool should_log_file = true;
    rerun::TextLogLevel level;
    std::string entity_path = "default_log";

    switch (message.verbosity) {
        case loguru::Verbosity_FATAL:
            level = rerun::TextLogLevel::Critical;
            entity_path = "fatal";
            break;
        case loguru::Verbosity_ERROR:
            level = rerun::TextLogLevel::Error;
            entity_path = "error";
            break;
        case loguru::Verbosity_WARNING:
            level = rerun::TextLogLevel::Warning;
            entity_path = "warning";
            break;
        case loguru::Verbosity_INFO:
            level = rerun::TextLogLevel::Info;
            entity_path = "info";
            break;
        case loguru::Verbosity_1:
            level = rerun::TextLogLevel::Debug;
            entity_path = "debug";
            break;
        case loguru::Verbosity_2:
            level = rerun::TextLogLevel::Trace;
            entity_path = "trace";
            break;
        case loguru::Verbosity_3:
            // no online info
            entity_path = "offline";
            level = rerun::TextLogLevel::Info;
            should_log_online = false;
            break;
        case loguru::Verbosity_4: {
            entity_path = "sensor";
            // online only every .1 seconds
            level = rerun::TextLogLevel::Info;

            std::string prefix =
                    std::string(message.message).substr(0, std::string(message.message).find(':'));
            std::chrono::time_point now = std::chrono::steady_clock::now();
            // check for thread safety but is only logging
            static std::map<std::string, std::chrono::steady_clock::time_point> map =
                    htwk::logging_utils::get_last_message_verbosity4();
            // check if the prefix is already in the map
            if (map.find(prefix) != map.end()) {
                // check if the time since the last message is greater than the threshold
                auto it = map.find(prefix);
                auto last_time = it->second;
                float threshold_seconds = htwk::SENSOR_LOGGING_THRESHOLD;
                if (std::chrono::duration_cast<std::chrono::nanoseconds>(now - last_time).count() <
                    threshold_seconds * 1e9) {
                    should_log_online = false;  // do not log this message
                } else {
                    map[prefix] = now;  // update the time for this prefix
                }
            } else {
                map[prefix] = now;  // add the prefix to the map with the current time
            }
            break;
        }
        case loguru::Verbosity_5: {
            entity_path = "sparse_info";
            // online only every .1 seconds
            level = rerun::TextLogLevel::Info;

            std::string prefix =
                    std::string(message.message).substr(0, std::string(message.message).find(':'));
            std::chrono::time_point now = std::chrono::steady_clock::now();
            // check for thread safety but is only logging
            static std::map<std::string, std::chrono::steady_clock::time_point> map =
                    htwk::logging_utils::get_last_message_verbosity5();
            static std::map<std::string, int> dropped_messages =
                    htwk::logging_utils::get_dropped_messages_verbosity5();
            // check if the prefix is already in the map
            if (map.find(prefix) != map.end()) {
                // check if the time since the last message is greater than the threshold
                auto it = map.find(prefix);
                auto last_time = it->second;
                float threshold_seconds = htwk::SPARSE_INFO_LOGGING_THRESHOLD;
                if (std::chrono::duration_cast<std::chrono::nanoseconds>(now - last_time).count() <
                    threshold_seconds * 1e9) {
                    should_log_online = false;
                    dropped_messages[prefix]++;
                } else {
                    std::cout << loguru_format_mock_verbosity_5(message, dropped_messages[prefix])
                              << std::endl;
                    dropped_messages[prefix] = 0;
                    map[prefix] = now;
                }
            } else {
                map[prefix] = now;  // add the prefix to the map with the current time
                dropped_messages[prefix] = 0;
                std::cout << loguru_format_mock_verbosity_5(message, dropped_messages[prefix])
                          << std::endl;
            }
            break;
        }
        default:
            level = rerun::TextLogLevel::Info;
            break;
    }

    if (should_log_file) {
        rerun_file_stream().log(
                entity_path, rerun::TextLog(std::string() + message.preamble + message.indentation +
                                            message.prefix + message.message)

                                     .with_level(level));
    }
    if (should_log_online) {
        rerun_online_stream().log(
                entity_path, rerun::TextLog(std::string() + message.preamble + message.indentation +
                                            message.prefix + message.message)
                                     .with_level(level));
    }
}
}  // namespace htwk::logging_utils

namespace htwk {
void init_logging(int argc, char** argv, const std::string& rerun_ip) {
    logging_utils::set_rerun_ip(rerun_ip);
    loguru::add_callback("log_to_rerun", logging_utils::loguru_to_rerun, nullptr,
                         loguru::Verbosity_MAX);
    loguru::init(argc, argv);
}

std::vector<rerun::components::LineStrip2D> build_circle(float x, float y, float radius) {
    const int SEGMENTS = 64;
    std::vector<rerun::components::LineStrip2D> strips;
    float last_x_pos = x + radius;
    float last_y_pos = y;
    for (int i = 1; i <= SEGMENTS; ++i) {
        float a = 2.0f * float(M_PI) * i / SEGMENTS;
        float x_pos = x + std::cos(a) * radius;
        float y_pos = y + std::sin(a) * radius;
        strips.emplace_back(
                rerun::components::LineStrip2D({{last_x_pos, last_y_pos}, {x_pos, y_pos}}));
        last_x_pos = x_pos;
        last_y_pos = y_pos;
    }
    return strips;
}

std::vector<rerun::components::LineStrip2D> build_quarter_circle(float x, float y, float radius, float start_angle) {
    const int SEGMENTS = 16;
    std::vector<rerun::components::LineStrip2D> strips;
    float last_x_pos = x + radius * std::cos(start_angle);
    float last_y_pos = y + radius * std::sin(start_angle);
    for (int i = 1; i <= SEGMENTS; ++i) {
        float a = start_angle + (float(M_PI) / 2.0f) * i / SEGMENTS;
        float x_pos = x + std::cos(a) * radius;
        float y_pos = y + std::sin(a) * radius;
        strips.emplace_back(
                rerun::components::LineStrip2D({{last_x_pos, last_y_pos}, {x_pos, y_pos}}));
        last_x_pos = x_pos;
        last_y_pos = y_pos;
    }
    return strips;
}

void log_soccer_field() {
    htwk::log_rerun_static(
            "soccerfield",
            rerun::Transform3D{}
                    .with_scale({-1.0f, 1.0f, 1.0f})
                    .with_relation(rerun::components::TransformRelation::ChildFromParent));
    // log field circle
    auto lines = SoccerField::getFieldLines();
    std::vector<rerun::components::LineStrip2D> linesList =
            build_circle(0.0f, 0.0f, SoccerField::circleDiameter() / 2.0f);
    if(SoccerField::hasCornerCircles()) {
        //log corner circles
        auto corner_circle_a = build_quarter_circle(
                -SoccerField::fieldLength() / 2.0f, SoccerField::fieldWidth() / 2.0f,
                SoccerField::cornerCircleRadius(), -M_PI*0.5);
        auto corner_circle_b = build_quarter_circle(
            SoccerField::fieldLength() / 2.0f, SoccerField::fieldWidth() / 2.0f,
            SoccerField::cornerCircleRadius(),M_PI);
        auto corner_circle_c = build_quarter_circle(
            SoccerField::fieldLength() / 2.0f, -SoccerField::fieldWidth() / 2.0f,
            SoccerField::cornerCircleRadius(), M_PI * 0.5f);
        auto corner_circle_d = build_quarter_circle(
            -SoccerField::fieldLength() / 2.0f, -SoccerField::fieldWidth() / 2.0f,
            SoccerField::cornerCircleRadius(), 0);
        linesList.insert(linesList.end(), corner_circle_a.begin(), corner_circle_a.end());
        linesList.insert(linesList.end(), corner_circle_b.begin(), corner_circle_b.end());
        linesList.insert(linesList.end(), corner_circle_c.begin(), corner_circle_c.end());  
        linesList.insert(linesList.end(), corner_circle_d.begin(), corner_circle_d.end());
    }

    for (int i = 0; i < lines.size(); i++) {
        float line_px1_1 = lines[i].px1;
        float line_py1_1 = lines[i].py1;
        float line_px2_1 = lines[i].px2;
        float line_py2_1 = lines[i].py2;

        rerun::components::LineStrip2D line({{line_px1_1, line_py1_1}, {line_px2_1, line_py2_1}});
        linesList.push_back(line);
    }
    htwk::log_rerun("soccerfield/lines", rerun::LineStrips2D(linesList).with_colors(
                                                 rerun::components::Color(255, 255, 255, 255)));
    // log invisible field border lines
    auto fieldBorder = SoccerField::getBorderLines();
    std::vector<rerun::components::LineStrip2D> fieldBorderList;
    for (const auto& line : fieldBorder) {
        float line_px1_1 = line.px1;
        float line_py1_1 = line.py1;
        float line_px2_1 = line.px2;
        float line_py2_1 = line.py2;
        rerun::components::LineStrip2D rerunline(
                {{line_px1_1, line_py1_1}, {line_px2_1, line_py2_1}});
        fieldBorderList.push_back(rerunline);
    }
    htwk::log_rerun("soccerfield/border_lines",
                    rerun::LineStrips2D(fieldBorderList)
                            .with_colors(rerun::components::Color(34, 139, 36)));

    //log field features
    //penalty spot
    std::vector<point_2d> penaltySpots = SoccerField::penaltySpots();
    std::vector<rerun::datatypes::Vec2D> rerunPenaltySpots;
    for (const auto& spot : penaltySpots) {
        rerunPenaltySpots.emplace_back(spot.x, spot.y);
    }
    htwk::log_rerun("soccerfield/point_features/penaltypoint", 
                   rerun::archetypes::Points2D(rerunPenaltySpots)
                   .with_radii(rerun::Radius::ui_points(5.0f))
                   .with_colors(rerun::components::Color(255, 165, 0, 255))
                   .with_show_labels(true));

    //L Spot
    std::vector<point_2d> lSpots = SoccerField::get_L();
    std::vector<rerun::datatypes::Vec2D> rerunLSpots;
    for (const auto& spot : lSpots) {
        rerunLSpots.push_back({spot.x, spot.y});
    }
    htwk::log_rerun("soccerfield/point_features/lcross", 
                   rerun::archetypes::Points2D(rerunLSpots)
                   .with_colors(rerun::components::Color(0, 255, 0, 255))
                   .with_radii(rerun::Radius::ui_points(5.0f))
                   .with_show_labels(true));   
    
    //T spot
    std::vector<point_2d> tSpots = SoccerField::get_T();
    std::vector<rerun::datatypes::Vec2D> rerunTSpots;
    for (const auto& spot : tSpots) {
        rerunTSpots.push_back({spot.x, spot.y});
    }
    htwk::log_rerun("soccerfield/point_features/tcross", 
                   rerun::archetypes::Points2D(rerunTSpots)
                   .with_radii(rerun::Radius::ui_points(5.0f))
                   .with_colors(rerun::components::Color(0, 0, 255, 255))
                   .with_show_labels(true));
        
    //X spot
    std::vector<point_2d> xSpots = SoccerField::get_X();
    std::vector<rerun::datatypes::Vec2D> rerunXSpots;
    for (const auto& spot : xSpots) {
        rerunXSpots.push_back({spot.x, spot.y});
    }
    htwk::log_rerun("soccerfield/point_features/xcross", 
                   rerun::archetypes::Points2D(rerunXSpots)
                   .with_radii(rerun::Radius::ui_points(5.0f))
                   .with_colors(rerun::components::Color(255, 0, 0, 255))
                   .with_show_labels(true));
}

void log_order(const Order& order, std::string info, std::string file, int line) {
    std::stringstream ss;
    ss << "Creating order: " << order.getClassName() << " from " << file << ":" << line
       << " with info: " << info;
    htwk::log_rerun("behavior/order",
                    rerun::TextLog(rerun::Text(ss.str())).with_level(rerun::TextLogLevel::Debug));
}

void log_time(uint64_t frame_number) {
    rerun::RecordingStream& online_stream = logging_utils::rerun_online_stream();
    rerun::RecordingStream& file_stream = logging_utils::rerun_file_stream();
    online_stream.set_time_sequence("frame_number", frame_number);
    file_stream.set_time_sequence("frame_number", frame_number);
}

void log_robot_position(std::string entity_path, const htwk::Position& position, float head_yaw,
                        const rerun::Color& color, std::string info) {
    const float posRadius = 5.0f;

    Position p_visualize = position;

    // Create direction lines
    std::vector<rerun::LineStrip2D> directionLines;

    // Line for orientation
    point_2d orientationEnd = p_visualize.point() + point_2d(0.3f, 0).rotated(p_visualize.a);
    rerun::LineStrip2D orientationLine(
            {{p_visualize.point().x, p_visualize.point().y}, {orientationEnd.x, orientationEnd.y}});
    directionLines.push_back(orientationLine);

    // Line for head direction
    point_2d headEnd = p_visualize.point() + point_2d(0.5f, 0).rotated(p_visualize.a + head_yaw);
    rerun::LineStrip2D headLine(
            {{p_visualize.point().x, p_visualize.point().y}, {headEnd.x, headEnd.y}});
    directionLines.push_back(headLine);

    // Log to rerun
    htwk::log_rerun(entity_path + "position",
                    rerun::Points2D({{p_visualize.point().x, p_visualize.point().y}})
                            .with_radii(rerun::Radius::ui_points(posRadius))
                            .with_colors(color)
                            .with_labels(rerun::Text(info))
                            .with_show_labels(false));

    htwk::log_rerun(entity_path + "directions", rerun::LineStrips2D(directionLines)
                                                        .with_radii(rerun::Radius::ui_points(2.0f))
                                                        .with_colors({color, color}));
}

}  // namespace htwk
