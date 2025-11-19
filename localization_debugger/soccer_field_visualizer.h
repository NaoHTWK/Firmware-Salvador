#pragma once

#include <GLFW/glfw3.h>
#include <imgui.h>

#include <functional>
#include <optional>
#include <string>
#include <vector>

#include "localization.h"
#include "point_2d.h"
#include "position.h"

struct RobotDrawInfo {
    enum class RobotType { TEAMMATE, OPPONENT, UNKNOWN };
    int id;
    bool active;
    htwk::Position pos;
    std::optional<htwk::Position> walk_target;
    RobotType type = RobotType::TEAMMATE;
};

class SoccerFieldVisualizer {
public:
    SoccerFieldVisualizer(const std::string& window_title);
    ~SoccerFieldVisualizer();

    bool windowShouldClose();
    void beginFrame();
    void endFrame();

    void drawFieldAndRobots(const std::vector<RobotDrawInfo>& robots,
                            const std::optional<point_2d>& ball_position);

    void drawHypotheses(const std::vector<Hypothesis>& hypotheses, const htwk::Position& true_pos);
    void drawRelativeView(const std::vector<htwk::Line>& projected_lines,
                          const std::vector<Localization::ProjectedPointFeature>& point_features,
                          float fov);

    point_2d fromScreen(const ImVec2& screen_pos) const;
    ImVec2 toScreen(float field_x, float field_y) const;

    GLFWwindow* getWindow() const {
        return window;
    }
    float getScale() const {
        return scale;
    }

private:
    static void glfw_error_callback(int error, const char* description);
    void init(const std::string& window_title);
    void cleanup();

    GLFWwindow* window = nullptr;
    float scale = 1.0f;
};