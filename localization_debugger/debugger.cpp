#include <GLFW/glfw3.h>
#include <imgui.h>
#include <imgui_impl_glfw.h>
#include <imgui_impl_opengl3.h>

#include <algorithm>
#include <cmath>
#include <iostream>
#include <optional>
#include <utility>
#include <vector>

#include "line.h"
#include "linegroup.h"
#include "localization.h"
#include "localization_utils.h"
#include "object_hypothesis.h"
#include "point_2d.h"
#include "position.h"
#include "soccer_field_visualizer.h"
#include "soccerfield.h"
#include "soccerfielddefinitions.h"

static void glfw_error_callback(int error, const char* description) {
    std::cerr << "Glfw Error " << error << ": " << description << std::endl;
}

int main() {
    SoccerFieldVisualizer visualizer("Localization Debugger");
    SoccerField::configureSoccerField(getSoccerFieldKidSize(), 0);
    Localization loc(0);
    float cam_fov = 110_deg;
    while (!visualizer.windowShouldClose()) {
        visualizer.beginFrame();
        static htwk::Position true_pos(0, 0, 0);
        ImGui::Begin("True Position");
        ImGui::SliderFloat("X", &true_pos.x, -4.5f, 4.5f);
        ImGui::SliderFloat("Y", &true_pos.y, -3.0f, 3.0f);
        ImGui::SliderFloat("A", &true_pos.a, -3.14f, 3.14f);
        ImGui::End();

        // Field features in absolute coordinates
        std::vector<std::pair<point_2d, htwk::ObjectType>> field_features;
        for (const auto& p : SoccerField::penaltySpots())
            field_features.emplace_back(p, htwk::ObjectType::PENALTY_SPOT);
        for (const auto& p : SoccerField::get_L())
            field_features.emplace_back(p, htwk::ObjectType::L_SPOT);
        for (const auto& p : SoccerField::get_T())
            field_features.emplace_back(p, htwk::ObjectType::T_SPOT);
        for (const auto& p : SoccerField::get_X())
            field_features.emplace_back(p, htwk::ObjectType::X_SPOT);
        // Add rotated versions
        auto original_features = field_features;
        for (const auto& f : original_features) {
            point_2d rotated(-f.first.x, -f.first.y);
            bool duplicate = false;
            for (const auto& existing : field_features) {
                if ((rotated - existing.first).norm() < 0.01f) {
                    duplicate = true;
                    break;
                }
            }
            if (!duplicate) {
                field_features.emplace_back(rotated, f.second);
            }
        }
        std::vector<Localization::ProjectedPointFeature> point_features;
        for (const auto& f : field_features) {
            point_2d rel = LocalizationUtils::absToRel(f.first, true_pos);
            point_features.emplace_back(rel, f.second);
        }
        std::vector<htwk::Line> projected;
        for (const auto& field_line : SoccerField::getFieldLines()) {
            point_2d rel_p1 = LocalizationUtils::absToRel(field_line.p1(), true_pos);
            point_2d rel_p2 = LocalizationUtils::absToRel(field_line.p2(), true_pos);
            projected.emplace_back(rel_p1, rel_p2);
        }
        // Filter points and clip lines to FOV and max distance
        float max_distance = 10.0f;
        float half_fov = cam_fov / 2.0f;
        std::vector<Localization::ProjectedPointFeature> filtered_point_features;
        for (const auto& pf : point_features) {
            float dist = sqrtf(pf.pos.x * pf.pos.x + pf.pos.y * pf.pos.y);
            if (dist < 1e-4f || dist > max_distance)
                continue;
            float angle = atan2f(pf.pos.y, pf.pos.x);
            if (std::abs(angle) > half_fov)
                continue;
            filtered_point_features.push_back(pf);
        }
        std::vector<htwk::Line> clipped_projected;
        struct HalfPlane {
            float a, b, c;
        };
        float tan_half = tanf(half_fov);
        std::vector<HalfPlane> planes = {
                {-1.f, 0.f, 0.f},       // x >= 0
                {-tan_half, 1.f, 0.f},  // y - tan*x <= 0
                {-tan_half, -1.f, 0.f}  // -y - tan*x <= 0
        };
        for (const auto& l : projected) {
            point_2d p1 = l.p1();
            point_2d p2 = l.p2();
            point_2d dir = p2 - p1;
            float t_min = 0.f;
            float t_max = 1.f;
            bool visible = true;
            for (const auto& plane : planes) {
                float pf = plane.a * p1.x + plane.b * p1.y + plane.c;
                float qf = plane.a * dir.x + plane.b * dir.y;
                if (std::abs(qf) < 1e-6f) {
                    if (pf > 0) {
                        visible = false;
                        break;
                    }
                } else {
                    float t = -pf / qf;
                    if (qf < 0) {
                        t_min = std::max(t_min, t);
                    } else {
                        t_max = std::min(t_max, t);
                    }
                }
            }
            if (!visible || t_min > t_max + 1e-6f)
                continue;
            point_2d c_start = p1 + t_min * dir;
            point_2d c_end = p1 + t_max * dir;
            point_2d c_dir = c_end - c_start;
            // Clip to circle
            float qa = c_dir.x * c_dir.x + c_dir.y * c_dir.y;
            if (qa < 1e-6f)
                continue;  // degenerate
            float qb = 2.f * (c_start.x * c_dir.x + c_start.y * c_dir.y);
            float qc = c_start.x * c_start.x + c_start.y * c_start.y - max_distance * max_distance;
            float disc = qb * qb - 4 * qa * qc;
            float ct_min = 0.f;
            float ct_max = 1.f;
            if (disc < 0) {
                // no intersection, check if inside
                if (c_start.norm() > max_distance + 1e-4f &&
                    (c_start + c_dir).norm() > max_distance + 1e-4f)
                    continue;
            } else {
                float sqrt_d = sqrtf(disc);
                float t1 = (-qb - sqrt_d) / (2 * qa);
                float t2 = (-qb + sqrt_d) / (2 * qa);
                if (t1 > t2)
                    std::swap(t1, t2);
                ct_min = std::max(t1, 0.f);
                ct_max = std::min(t2, 1.f);
                if (ct_min > ct_max + 1e-6f)
                    continue;
            }
            point_2d clip_start = c_start + ct_min * c_dir;
            point_2d clip_end = c_start + ct_max * c_dir;
            clipped_projected.emplace_back(clip_start, clip_end);
        }
        auto [hypotheses_lines, _] = loc.hypothesesLines(clipped_projected);
        std::vector<Hypothesis> hypotheses_point =
                loc.hypothesesPointFeatures(filtered_point_features, clipped_projected);
        std::vector<Hypothesis> all_hyps;
        all_hyps.insert(all_hyps.end(), hypotheses_lines.begin(), hypotheses_lines.end());
        all_hyps.insert(all_hyps.end(), hypotheses_point.begin(), hypotheses_point.end());
        std::vector<Hypothesis> merged = loc.merge(all_hyps);
        ImGui::Begin("Stats");
        std::map<htwk::ObjectType, int> point_counts;
        for (const auto& pf : filtered_point_features)
            point_counts[pf.type]++;
        ImGui::Text("Point Features in Image:");
        for (const auto& [type, count] : point_counts) {
            std::string type_str;
            switch (type) {
                case htwk::ObjectType::CENTER_SPOT:
                    type_str = "CENTER_SPOT";
                    break;
                case htwk::ObjectType::PENALTY_SPOT:
                    type_str = "PENALTY_SPOT";
                    break;
                case htwk::ObjectType::GOAL_POST:
                    type_str = "GOAL_POST";
                    break;
                case htwk::ObjectType::L_SPOT:
                    type_str = "L_SPOT";
                    break;
                case htwk::ObjectType::T_SPOT:
                    type_str = "T_SPOT";
                    break;
                case htwk::ObjectType::X_SPOT:
                    type_str = "X_SPOT";
                    break;
            }
            ImGui::Text("%s: %d", type_str.c_str(), count);
        }
        ImGui::Text("Total Points: %lu", filtered_point_features.size());
        ImGui::Text("Projected Lines: %lu", clipped_projected.size());
        ImGui::Text("Line Hypotheses: %lu", hypotheses_lines.size());
        ImGui::Text("Point Hypotheses: %lu", hypotheses_point.size());
        ImGui::Text("Merged Hypotheses: %lu", merged.size());
        ImGui::Separator();
        ImGui::Text("Point Feature Details:");
        for (const auto& pf : filtered_point_features) {
            std::string type_str;
            switch (pf.type) {
                case htwk::ObjectType::CENTER_SPOT:
                    type_str = "CENTER_SPOT";
                    break;
                case htwk::ObjectType::PENALTY_SPOT:
                    type_str = "PENALTY_SPOT";
                    break;
                case htwk::ObjectType::GOAL_POST:
                    type_str = "GOAL_POST";
                    break;
                case htwk::ObjectType::L_SPOT:
                    type_str = "L_SPOT";
                    break;
                case htwk::ObjectType::T_SPOT:
                    type_str = "T_SPOT";
                    break;
                case htwk::ObjectType::X_SPOT:
                    type_str = "X_SPOT";
                    break;
            }
            ImGui::Text("%s at (%.2f, %.2f)", type_str.c_str(), pf.pos.x, pf.pos.y);
        }
        ImGui::Separator();
        ImGui::Text("Projected Line Details:");
        for (const auto& pl : clipped_projected) {
            ImGui::Text("Line from (%.2f,%.2f) to (%.2f,%.2f), len=%.2f", pl.p1().x, pl.p1().y,
                        pl.p2().x, pl.p2().y, std::sqrt(pl.norm_sqr()));
        }
        ImGui::End();
        visualizer.drawFieldAndRobots({}, {});
        visualizer.drawHypotheses(merged, true_pos);
        visualizer.drawRelativeView(clipped_projected, filtered_point_features, cam_fov);
        visualizer.endFrame();
    }
    return 0;
}