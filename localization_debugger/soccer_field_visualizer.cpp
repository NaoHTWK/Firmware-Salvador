#include "soccer_field_visualizer.h"

#include <GLFW/glfw3.h>

#include <cmath>
#include <iostream>

#include "imgui_impl_glfw.h"
#include "imgui_impl_opengl3.h"
#include "line.h"
#include "soccerfield.h"
#include "soccerfielddefinitions.h"

SoccerFieldVisualizer::SoccerFieldVisualizer(const std::string& window_title) {
    init(window_title);
}

SoccerFieldVisualizer::~SoccerFieldVisualizer() {
    cleanup();
}

void SoccerFieldVisualizer::glfw_error_callback(int error, const char* description) {
    std::cerr << "Glfw Error " << error << ": " << description << std::endl;
}

void SoccerFieldVisualizer::init(const std::string& window_title) {
    glfwSetErrorCallback(glfw_error_callback);
    if (!glfwInit()) {
        throw std::runtime_error("Failed to initialize GLFW");
    }

    glfwWindowHint(GLFW_CONTEXT_VERSION_MAJOR, 3);
    glfwWindowHint(GLFW_CONTEXT_VERSION_MINOR, 3);
    glfwWindowHint(GLFW_OPENGL_PROFILE, GLFW_OPENGL_CORE_PROFILE);

    window = glfwCreateWindow(1280, 720, window_title.c_str(), nullptr, nullptr);
    if (window == nullptr) {
        throw std::runtime_error("Failed to create GLFW window");
    }
    glfwMakeContextCurrent(window);
    glfwSwapInterval(1);  // Enable vsync

    IMGUI_CHECKVERSION();
    ImGui::CreateContext();
    ImGuiIO& io = ImGui::GetIO();
    io.ConfigFlags |= ImGuiConfigFlags_NavEnableKeyboard;
    io.ConfigFlags |= ImGuiConfigFlags_NavEnableGamepad;

    ImGui::StyleColorsDark();

    ImGui_ImplGlfw_InitForOpenGL(window, true);
    ImGui_ImplOpenGL3_Init();
}

void SoccerFieldVisualizer::cleanup() {
    ImGui_ImplOpenGL3_Shutdown();
    ImGui_ImplGlfw_Shutdown();
    ImGui::DestroyContext();

    glfwDestroyWindow(window);
    glfwTerminate();
}

bool SoccerFieldVisualizer::windowShouldClose() {
    return glfwWindowShouldClose(window);
}

void SoccerFieldVisualizer::beginFrame() {
    glfwPollEvents();

    ImGui_ImplOpenGL3_NewFrame();
    ImGui_ImplGlfw_NewFrame();
    ImGui::NewFrame();
}

void SoccerFieldVisualizer::endFrame() {
    ImGui::Render();
    int display_w, display_h;
    glfwGetFramebufferSize(window, &display_w, &display_h);
    glViewport(0, 0, display_w, display_h);
    glClearColor(0.1f, 0.1f, 0.1f, 1.0f);
    glClear(GL_COLOR_BUFFER_BIT);
    ImGui_ImplOpenGL3_RenderDrawData(ImGui::GetDrawData());

    glfwSwapBuffers(window);
}

point_2d SoccerFieldVisualizer::fromScreen(const ImVec2& screen_pos) const {
    const ImVec2 window_size = ImGui::GetIO().DisplaySize;
    return point_2d{(screen_pos.x - window_size.x / 2.f) / scale,
                    (window_size.y / 2.f - screen_pos.y) / scale};
}

ImVec2 SoccerFieldVisualizer::toScreen(float field_x, float field_y) const {
    const ImVec2 window_size = ImGui::GetIO().DisplaySize;
    return ImVec2(window_size.x / 2.f + field_x * scale, window_size.y / 2.f - field_y * scale);
}

void SoccerFieldVisualizer::drawFieldAndRobots(const std::vector<RobotDrawInfo>& robots,
                                               const std::optional<point_2d>& ball_position) {
    ImDrawList* draw_list = ImGui::GetBackgroundDrawList();
    const float field_length = SoccerField::length();
    const float field_width = SoccerField::width();
    const float border_strip_width = 1.0f;  // 1m border

    const ImVec2 window_size = ImGui::GetIO().DisplaySize;
    const float view_margin = 100.0f;
    const float scale_x =
            (window_size.x - 2 * view_margin) / (field_length + 2 * border_strip_width);
    const float scale_y =
            (window_size.y - 2 * view_margin) / (field_width + 2 * border_strip_width);
    this->scale = std::min(scale_x, scale_y);

    ImGui::Begin("Debug Info");
    ImGui::Text("Scale: %.2f", scale);
    ImGui::Text("Window Size: %.0fx%.0f", window_size.x, window_size.y);
    ImGui::End();

    const ImVec2 field_top_left = toScreen(-field_length / 2.f, field_width / 2.f);
    const ImVec2 field_bottom_right = toScreen(field_length / 2.f, -field_width / 2.f);
    draw_list->AddRectFilled(field_top_left, field_bottom_right, IM_COL32(0, 128, 0, 255));

    for (const auto& line : SoccerField::getFieldLines()) {
        draw_list->AddLine(toScreen(line.p1().x, line.p1().y), toScreen(line.p2().x, line.p2().y),
                           IM_COL32(255, 255, 255, 255), 3.0f);
    }

    draw_list->AddCircle(toScreen(0, 0), SoccerField::circleDiameter() / 2.f * scale,
                         IM_COL32(255, 255, 255, 255), 0, 3.0f);

    // Draw ball
    if (ball_position) {
        ImVec2 ball_pos_screen = toScreen(ball_position->x, ball_position->y);
        draw_list->AddCircleFilled(ball_pos_screen, 0.15f * scale, IM_COL32(255, 255, 0, 255));
    }

    for (const auto& robot : robots) {
        if (robot.active) {
            ImU32 color;
            switch (robot.type) {
                case RobotDrawInfo::RobotType::TEAMMATE:
                    color = IM_COL32(255, 0, 0, 255);
                    break;
                case RobotDrawInfo::RobotType::OPPONENT:
                    color = IM_COL32(0, 0, 255, 255);
                    break;
                case RobotDrawInfo::RobotType::UNKNOWN:
                    color = IM_COL32(128, 128, 128, 255);
                    break;
            }

            ImVec2 pos = toScreen(robot.pos.x, robot.pos.y);
            ImVec2 dir_end = {pos.x + 0.2f * scale * cosf(robot.pos.a),
                              pos.y - 0.2f * scale * sinf(robot.pos.a)};
            draw_list->AddLine(pos, dir_end, IM_COL32(0, 0, 255, 255), 3.0f);
            draw_list->AddCircleFilled(pos, 0.15f * scale, color);

            if (robot.type == RobotDrawInfo::RobotType::TEAMMATE) {
                std::string jersey_str = std::to_string(robot.id + 1);
                ImVec2 text_size = ImGui::CalcTextSize(jersey_str.c_str());
                draw_list->AddText({pos.x - text_size.x / 2.f, pos.y - text_size.y / 2.f},
                                   IM_COL32(255, 255, 255, 255), jersey_str.c_str());
            }

            if (robot.walk_target) {
                ImVec2 target_pos = toScreen(robot.walk_target->x, robot.walk_target->y);
                draw_list->AddCircle(target_pos, 0.15f * scale, IM_COL32(255, 0, 0, 100), 0, 2.0f);
                ImVec2 target_dir_end = {target_pos.x + 0.2f * scale * cosf(robot.walk_target->a),
                                         target_pos.y - 0.2f * scale * sinf(robot.walk_target->a)};
                draw_list->AddLine(target_pos, target_dir_end, IM_COL32(0, 0, 255, 100), 3.0f);
                draw_list->AddLine(pos, target_pos, IM_COL32(255, 255, 255, 100), 1.0f);
            }
        }
    }
}

void SoccerFieldVisualizer::drawHypotheses(const std::vector<Hypothesis>& hypotheses,
                                           const htwk::Position& true_pos) {
    ImDrawList* draw_list = ImGui::GetBackgroundDrawList();
    // Draw true position in green
    ImVec2 pos = toScreen(true_pos.x, true_pos.y);
    ImVec2 dir_end = {pos.x + 0.3f * scale * cosf(true_pos.a),
                      pos.y - 0.3f * scale * sinf(true_pos.a)};
    draw_list->AddLine(pos, dir_end, IM_COL32(0, 255, 0, 255), 4.0f);
    draw_list->AddCircleFilled(pos, 0.2f * scale, IM_COL32(0, 255, 0, 255));
    // Draw hypotheses in yellow with alpha based on qual
    for (const auto& hyp : hypotheses) {
        ImU32 color = IM_COL32(255, 255, 0, static_cast<int>(hyp.qual * 255));
        ImVec2 hpos = toScreen(hyp.p.x, hyp.p.y);
        ImVec2 hdir_end = {hpos.x + 0.3f * scale * cosf(hyp.p.a),
                           hpos.y - 0.3f * scale * sinf(hyp.p.a)};
        draw_list->AddLine(hpos, hdir_end, color, 4.0f);
        draw_list->AddCircleFilled(hpos, 0.2f * scale, color);
    }
}

void SoccerFieldVisualizer::drawRelativeView(
        const std::vector<htwk::Line>& projected_lines,
        const std::vector<Localization::ProjectedPointFeature>& point_features, float fov) {
    ImGui::SetNextWindowSize(ImVec2(600, 400), ImGuiCond_FirstUseEver);
    ImGui::Begin("Relative View");
    ImVec2 pos = ImGui::GetCursorScreenPos();
    ImVec2 size = ImGui::GetContentRegionAvail();
    if (size.x <= 0.0f || size.y <= 0.0f) {
        ImGui::End();
        return;
    }
    ImDrawList* draw_list = ImGui::GetWindowDrawList();
    draw_list->AddRectFilled(pos, ImVec2(pos.x + size.x, pos.y + size.y), IM_COL32(0, 128, 0, 255));
    float max_distance = 10.0f;
    float half_fov = fov / 2.0f;
    float view_width = 2.0f * max_distance * tanf(half_fov);
    float rel_scale = std::min(size.x / view_width, size.y / max_distance) * 2.0f;
    ImVec2 robot_screen(pos.x + size.x / 2.0f, pos.y + size.y - 20.0f);
    draw_list->AddCircleFilled(robot_screen, 10.0f, IM_COL32(255, 0, 0, 255));
    auto to_rel_screen = [&](float rx, float ry) -> ImVec2 {
        return {robot_screen.x - ry * rel_scale, robot_screen.y - rx * rel_scale};
    };
    float ray_len = max_distance;
    ImVec2 end_left = to_rel_screen(ray_len * cosf(half_fov), ray_len * sinf(half_fov));
    ImVec2 end_right = to_rel_screen(ray_len * cosf(-half_fov), ray_len * sinf(-half_fov));
    draw_list->AddLine(robot_screen, end_left, IM_COL32(0, 255, 0, 128), 1.0f);
    draw_list->AddLine(robot_screen, end_right, IM_COL32(0, 255, 0, 128), 1.0f);
    for (const auto& l : projected_lines) {
        ImVec2 s = to_rel_screen(l.p1().x, l.p1().y);
        ImVec2 e = to_rel_screen(l.p2().x, l.p2().y);
        draw_list->AddLine(s, e, IM_COL32(255, 255, 255, 255), 2.0f);
    }
    for (const auto& pf : point_features) {
        ImVec2 p = to_rel_screen(pf.pos.x, pf.pos.y);
        draw_list->AddCircleFilled(p, 5.0f, IM_COL32(255, 255, 0, 255));
    }
    ImGui::End();
}