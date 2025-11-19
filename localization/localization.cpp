#include <algorithm_ext.h>
#include <cam_constants.h>
#include <color.h>
#include <imu_joint_state.h>
#include <localization.h>
#include <localization_utils.h>
#include <logging.h>
#include <stl_ext.h>

#include <cmath>
#include <functional>
#include <rerun.hpp>
#include <set>

#include "cam_pose.h"
#include "localization_pub_sub.h"

using namespace htwk;
using namespace std;

namespace {

constexpr float max_dist = 0.2f;
constexpr float max_dist_long_lines = 0.5f;
constexpr float long_line_threshold = 2.5f;
constexpr float min_line_length = 0.2f;

point_2d transformPo(const point_2d& p, const float trans[][2], const float translation[]) {
    return {trans[0][0] * p.x + trans[1][0] * p.y + translation[0],
            trans[0][1] * p.x + trans[1][1] * p.y + translation[1]};
}

point_2d transformPoInv(const point_2d& p, const float trans[][2], const float translation[]) {
    return {(p.x - translation[0]) * trans[1][1] - (p.y - translation[1]) * trans[1][0],
            (p.x - translation[0]) * -trans[0][1] + (p.y - translation[1]) * trans[0][0]};
}

optional<Line> findChord(const Ellipse& ellipse, const vector<LineGroup>& line_groups) {
    for (const LineGroup& lg : line_groups) {
        Line line = lg.middle();
        point_2d t_ellipse(ellipse.ta, ellipse.tb);
        // Transform line into a space where the ellipse is a unit circle centered in (0,0).
        Line transformed(
                transformPo(line.p1(), ellipse.trans, ellipse.translation).div_elem(t_ellipse),
                transformPo(line.p2(), ellipse.trans, ellipse.translation).div_elem(t_ellipse));
        if (transformed.extendedDistance({0, 0}) >= 0.2f)
            continue;
        point_2d d = transformed.u().normalized();
        point_2d p = transformed.p1();
        // Intersection with circle, see:
        // https://math.stackexchange.com/questions/929193/general-solution-for-intersection-of-line-and-circle
        float sq = sqrt(p.dot(d) * p.dot(d) - p.norm_sqr() + 1);
        if (std::isnan(sq))
            continue;
        float t1 = -p.dot(d) + sq;
        float t2 = -p.dot(d) - sq;
        return Line{transformPoInv((p + t1 * d).mul_elem(t_ellipse), ellipse.trans,
                                   ellipse.translation),
                    transformPoInv((p + t2 * d).mul_elem(t_ellipse), ellipse.trans,
                                   ellipse.translation)};
    }
    return {};
}

bool matches(Line& model, const Line& perception) {
    // TODO: This should be based on distance of the perception and expected error at that
    // distance.
    if (perception.norm() >= long_line_threshold)
        return model.distance(perception.p1()) <= max_dist_long_lines &&
               model.distance(perception.p2()) <= max_dist_long_lines;
    return model.distance(perception.p1()) <= max_dist &&
           model.distance(perception.p2()) <= max_dist;
}

bool matches_any(vector<Line>& model, const Line& perception) {
    return any_of(model.begin(), model.end(),
                  [&](auto& mline) { return matches(mline, perception); });
}

int count_matches(vector<Line>& model, const std::vector<Line>& perceptions) {
    return count_if(perceptions, [&model](const Line& l) { return matches_any(model, l); });
}

void visualize(const Position& p, float head_yaw, const rerun::Color& color,
               std::string entity_path = "soccerfield/robot_own/", float quality = -1.0f) {
    std::stringstream ss;
    ss << "qual: " << quality;
    htwk::log_robot_position(entity_path, p, head_yaw, color, ss.str());
}

void visualize(const Hypothesis& h, float head_yaw, const rerun::Color& color,
               std::string entity_path) {
    visualize(h.p, head_yaw, color, entity_path);
}

void visualize(const vector<Hypothesis>& hypotheses, float head_yaw) {
    for (int i = 0; i < hypotheses.size(); i++) {
        const Hypothesis& h = hypotheses[i];
        rerun::Color color = rerun::Color(255, 255, 0, h.qual * 255);
        std::stringstream ss;
        ss << "qual: " << h.qual << " num_lines: " << h.line_ids.size()
           << " num_point_features: " << h.point_feature_ids.size();
        htwk::log_robot_position("soccerfield/localization/hypotheses/" + std::to_string(i), h.p,
                                 head_yaw, color, ss.str());
    }
}

bool in_own_half(const Hypothesis& hyp1) {
    static Position ownSide(-1, 0, 0);
    static Position oppSide(1, 0, 180_deg);
    return (hyp1.p - ownSide).weightedNorm(0.25f) < (hyp1.p - oppSide).weightedNorm(0.25f);
}

}  // namespace

optional<PitchRoll> Localization::angleFromEllipse(const Ellipse& e) {
    double data[24] = {e.a,
                       e.b,
                       e.c,
                       e.d,
                       e.e,
                       e.f,
                       e.ta,
                       e.tb,
                       e.brennpunkt,
                       e.translation[0],
                       e.translation[1],
                       acos(e.trans[0][0])};
    for (int i = 0; i < 12; i++) {
        data[12 + i] = data[i] * data[i];
    }
    PitchRoll angle;
    if (SoccerField::circleDiameter() == 1.5f) {
        angle = PitchRoll(
                0.23510986767097708 + (data[0] - (2.818417850420365E-6)) * -1617.9382858009312 +
                        (data[1] - (-4.168647006026829E-6)) * -1922.650168644176 +
                        (data[2] - (1.8337161815750592E-5)) * -1234.9195328585304 +
                        (data[3] - (-9.531940787141114E-4)) * -11.827933206197713 +
                        (data[4] - (-0.006587469848144885)) * -2.806259880875129 +
                        (data[5] - (0.9999726079215608)) * 9980.047966857683 +
                        (data[6] - (26.303558882665417)) * 0.010029262977545412 +
                        (data[7] - (79.1143877262635)) * 0.002097918099525817 +
                        (data[8] - (73.84748138064974)) * -0.0013456317908761126 +
                        (data[9] - (-256.4441294857918)) * 0.002927072351963823 +
                        (data[10] - (308.31047838584766)) * 3.0860525241464717E-4 +
                        (data[11] - (1.5707704675968777)) * -0.17279332211878576 +
                        (data[12] - (2.5301872349568308E-11)) * -1.0794252214678083E7 +
                        (data[13] - (4.0261765906330747E-10)) * 630013.8625616378 +
                        (data[14] - (1.7361652526124365E-9)) * -9496.785218097784 +
                        (data[15] - (2.6403368690162264E-6)) * -303.3701636643679 +
                        (data[16] - (5.213235908745658E-5)) * 924.9158913105039 +
                        (data[17] - (0.9999452251399475)) * -4179.8801341088465 +
                        (data[18] - (1052.2975966234717)) * -0.0013620385494453656 +
                        (data[19] - (6983.697726387217)) * 0.001327875651664331 +
                        (data[20] - (5931.413912122716)) * -0.0013423099925751899 +
                        (data[21] - (78816.35486248913)) * -2.676568181942322E-7 +
                        (data[22] - (132376.47901795557)) * -5.078897655581554E-7 +
                        (data[23] - (2.5392552404823894)) * -0.24868159928003966,
                -3.118589840607364E-4 + (data[0] - (2.818417850420365E-6)) * -44642.72330525053 +
                        (data[1] - (-4.168647006026829E-6)) * -14514.16740116423 +
                        (data[2] - (1.8337161815750592E-5)) * -5108.112109657765 +
                        (data[3] - (-9.531940787141114E-4)) * -128.09261048691945 +
                        (data[4] - (-0.006587469848144885)) * -49.22621234506471 +
                        (data[5] - (0.9999726079215608)) * 38363.226070422636 +
                        (data[6] - (26.303558882665417)) * -0.0032407702071365714 +
                        (data[7] - (79.1143877262635)) * 0.001738523220866748 +
                        (data[8] - (73.84748138064974)) * -4.7903009728824216E-4 +
                        (data[9] - (-256.4441294857918)) * -0.0015532223839493002 +
                        (data[10] - (308.31047838584766)) * 8.124490460826565E-4 +
                        (data[11] - (1.5707704675968777)) * -2.2478567294502505 +
                        (data[12] - (2.5301872349568308E-11)) * 4.445007926698641E7 +
                        (data[13] - (4.0261765906330747E-10)) * -2988810.383498364 +
                        (data[14] - (1.7361652526124365E-9)) * 555986.3510197543 +
                        (data[15] - (2.6403368690162264E-6)) * 4866.811165270825 +
                        (data[16] - (5.213235908745658E-5)) * 540.7782675557455 +
                        (data[17] - (0.9999452251399475)) * -18748.22644556813 +
                        (data[18] - (1052.2975966234717)) * -0.0033928283512448393 +
                        (data[19] - (6983.697726387217)) * 0.003400066082397966 +
                        (data[20] - (5931.413912122716)) * -0.003400521886568123 +
                        (data[21] - (78816.35486248913)) * -1.2995329514796369E-6 +
                        (data[22] - (132376.47901795557)) * -5.798668693837813E-8 +
                        (data[23] - (2.5392552404823894)) * 0.5118898685886135);
    } else {
        printf("Can't handle circle diameter of %f!\n", SoccerField::circleDiameter());
        exit(-1);
    }
    // Throw away unrealistic values.
    if (abs(angle.roll) > M_PIf / 4.f || angle.pitch < -M_PIf / 8.f || angle.pitch > M_PIf * .875f)
        return {};
    return angle;
}

void Localization::visualizeRelative(const vector<Line>& lines,
                                     const vector<ProjectedPointFeature>& projected_points,
                                     CamPose& cam_pose) {

    htwk::log_rerun_static(
            "relative",
            rerun::Transform3D{}
                    .with_scale({1.0f, -1.0f, 1.0f})
                    .with_relation(rerun::components::TransformRelation::ChildFromParent));

    // artifical fov
    std::vector<rerun::LineStrip2D> fov_lines;
    point_2d fov_left = (point_2d{1, 0}).rotated(-1.f + cam_pose.head_angles.yaw).normalized() * 10;
    point_2d fov_right = (point_2d{1, 0}).rotated(1.f + cam_pose.head_angles.yaw).normalized() * 10;
    fov_lines.push_back(rerun::LineStrip2D({{0, 0}, {fov_left.x, fov_left.y}}));
    fov_lines.push_back(rerun::LineStrip2D({{0, 0}, {fov_right.x, fov_right.y}}));
    htwk::log_rerun("relative/fov", rerun::LineStrips2D(fov_lines));

    // Visualize projected lines
    std::vector<rerun::LineStrip2D> rerun_lines;
    for (const auto& line : lines) {
        rerun_lines.push_back(
                rerun::LineStrip2D({{line.p1().x, line.p1().y}, {line.p2().x, line.p2().y}}));
    }

    htwk::log_rerun("relative/projections/lines",
                    rerun::LineStrips2D(rerun_lines)
                            .with_radii(rerun::Radius::ui_points(1.0f))
                            .with_colors(rerun::Color(255, 255, 255, 255)));

    std::vector<rerun::Position2D> positions;
    std::vector<rerun::Text> labels;
    std::vector<rerun::Color> colors;

    // Visualize point features
    for (const auto& proj : projected_points) {
        static unordered_map<ObjectType, rerun::Color> color_map{
                {ObjectType::CENTER_SPOT, rerun::Color(255, 0, 255, 255)},   // PINK
                {ObjectType::PENALTY_SPOT, rerun::Color(255, 165, 0, 255)},  // ORANGE
                {ObjectType::GOAL_POST, rerun::Color(128, 128, 128, 255)},   // GREY
                {ObjectType::L_SPOT, rerun::Color(0, 255, 0, 255)},          // GREEN
                {ObjectType::T_SPOT, rerun::Color(0, 0, 255, 255)},          // BLUE
                {ObjectType::X_SPOT, rerun::Color(255, 0, 0, 255)}           // RED
        };

        static unordered_map<ObjectType, rerun::Text> label_map{
                {ObjectType::CENTER_SPOT, rerun::Text("CENTER_SPOT")},
                {ObjectType::PENALTY_SPOT, rerun::Text("PENALTY_SPOT")},
                {ObjectType::GOAL_POST, rerun::Text("GOAL_POST")},
                {ObjectType::L_SPOT, rerun::Text("L_SPOT")},
                {ObjectType::T_SPOT, rerun::Text("T_SPOT")},
                {ObjectType::X_SPOT, rerun::Text("X_SPOT")}};

        positions.push_back(rerun::Position2D(proj.pos.x, proj.pos.y));
        labels.push_back(label_map[proj.type]);
        colors.push_back(color_map[proj.type]);
    }
    htwk::log_rerun("relative/projections/point_features",
                    rerun::Points2D(positions)
                            .with_radii(rerun::Radius::ui_points(2.0f))
                            .with_colors(colors)
                            .with_labels(labels)
                            .with_show_labels(false));
}

vector<Line> Localization::projectLines(const vector<LineGroup>& line_groups, CamPose& cam_pose) {
    vector<Line> projected;
    for (const auto& line : line_groups) {
        if (auto pline = CamUtils::project(line.middle(), cam_pose)) {
            // Throw out lines that are too short and would likely match anything in the model.
            if (pline->norm_sqr() < min_line_length * min_line_length)
                continue;
            projected.emplace_back(*pline);
        }
    }
    sort_desc(projected, norm_sqr());
    return projected;
}

void Localization::proceed(CamPose& cam_pose, int64_t timestamp_us) {
    // EASY_FUNCTION(profiler::colors::Cyan500);

    vector<Hypothesis> hypotheses;

    std::vector<htwk::LineGroup> line_groups =
            lines_subscriber.latestOr(std::vector<htwk::LineGroup>());

    vector<Line> projected_lines = projectLines(line_groups, cam_pose);

    auto [hypotheses_lines, max_line_matches] = hypothesesLines(projected_lines);

    std::vector<htwk::ObjectHypothesis> point_features =
            point_features_subscriber.latestOr(std::vector<htwk::ObjectHypothesis>());

    std::vector<ProjectedPointFeature> projected_points;
    for (const auto& feature : point_features) {
        if (auto p = CamUtils::project(feature.point(), cam_pose)) {
            projected_points.push_back({*p, feature.type});
        }
    }

    visualizeRelative(projected_lines, projected_points, cam_pose);

    vector<Hypothesis> hypotheses_point_features =
            hypothesesPointFeatures(projected_points, projected_lines);

    hypotheses.insert(hypotheses.end(), hypotheses_lines.begin(), hypotheses_lines.end());
    hypotheses.insert(hypotheses.end(), hypotheses_point_features.begin(),
                      hypotheses_point_features.end());
    hypotheses = merge(hypotheses);

    GCState gc_state = gc_state_sub.latest();
    if (gc_state.state == GameState::Initial || gc_state.state == GameState::Finished ||
        gc_state.state == GameState::Standby || gc_state.my_team.players[idx].is_penalized) {
        loc_quality = 0;
        initialized = false;
    }

    std::lock_guard<std::mutex> lock(mutex);
    Position odo = htwk::deltaOdoRelative(last_image_time, timestamp_us);
    last_image_time = timestamp_us;

    if (initialized) {
        position.add_relative(odo);
    }

    htwk::log_rerun("soccerfield/localization/hypotheses/", rerun::Clear().with_is_recursive(true));

    good_projection = false;  // cam_pose.cam_id == CamID::LOWER;
    if (!initialized) {
        std::optional<Hypothesis> hyp_for_init;
        for (const Hypothesis& hyp : hypotheses) {
            if (hyp.point_feature_ids.count(0) > 0 && in_own_half(hyp) && hyp.numFeatures() >= 3) {
                hyp_for_init = hyp;
                break;
            }
        }
        if (!hyp_for_init) {
            if ((hypotheses.size() == 2 ||
                 (hypotheses.size() > 2 &&
                  hypotheses[0].numFeatures() > hypotheses[2].numFeatures())) &&
                (hypotheses[0].point_feature_ids.size() >= 2 ||
                 hypotheses[0].point_feature_ids.count(0) > 0 ||
                 (hypotheses[0].point_feature_ids.size() == 1 &&
                  hypotheses[0].line_ids.size() >= 3) ||
                 hypotheses[0].line_ids.size() >= 4)) {
                hyp_for_init = in_own_half(hypotheses[0]) ? hypotheses[0] : hypotheses[1];
            }
        }
        if (hyp_for_init) {
            position = hyp_for_init->p;
            loc_quality = 0.7f;
            initialized = true;
            good_projection = true;
        }
        visualize(hypotheses, cam_pose.head_angles.yaw);
        // Visualizer::instance().commit(vis);
        loc_position_channel.publish(LocPosition{.position = position, .quality = loc_quality});
        return;
    }
    bool matched = false;
    erase_if(hypotheses, [this](const Hypothesis& h) {
        switch (h.trust) {
            case Hypothesis::Trust::HIGH:
                return false;
            case Hypothesis::Trust::MEDIUM:
                return loc_quality > 0 && (position - h.p).weightedNorm(4) > 4.f;
            case Hypothesis::Trust::LOW:
            default:
                return loc_quality > 0 && (position - h.p).weightedNorm(4) > 2.f;
        }
    });
    if (!hypotheses.empty()) {
        // TODO: We should adjust towards the most accurate somewhat close hypothesis instead of
        // the hypothesis closest to the current belief, which might be based on a line 4m away.
        auto closest = min_element_transformed(
                hypotheses, [&](const auto& a) { return (a.p - position).weightedNorm(4); });
        // Far away matches influence the position update less than close matches.
        float dist_fac = clamped_linear_interpolation(closest->match_dist, 1.f, 0.5f, 2.f, 8.f);
        position += {clamp(closest->p.x - position.x, -f, f) * fp * dist_fac,
                     clamp(closest->p.y - position.y, -f, f) * fp * dist_fac,
                     clamp(normalizeRotation(closest->p.a - position.a), -f_yaw, f_yaw) * fp_yaw *
                             dist_fac};
        if ((position - closest->p).weightedNorm(0.25f) < 0.75f) {
            matched = true;
        }
    }
    visualize(hypotheses, cam_pose.head_angles.yaw);
    visualize(position, cam_pose.head_angles.yaw, rerun::Color(255, 0, 0, 255),
              "soccerfield/robot_own/", loc_quality);
    if (matched) {
        loc_quality = limit01(loc_quality + 1.f / 3.5f);
        good_projection = true;
    } else {
        loc_quality *= limit01(0.999f - odo.weightedNorm(0.125f) / 128.f);
    }

    loc_position_channel.publish(LocPosition{.position = position, .quality = loc_quality});
}

std::vector<Hypothesis> Localization::merge(const std::vector<Hypothesis>& hyps) {
    std::vector<Hypothesis> hyps_sorted = hyps;
    sort(hyps_sorted, [](const Hypothesis& h1, const Hypothesis& h2) {
        return h1.line_ids.size() + h1.point_feature_ids.size() >
               h2.line_ids.size() + h2.point_feature_ids.size();
    });

    size_t max_lines_merged = 0;
    size_t max_points_merged = 0;

    for (size_t i = 0; i < hyps_sorted.size(); i++) {
        std::vector<Hypothesis> remaining;
        std::vector<Hypothesis> to_merge{hyps_sorted[i]};
        for (size_t j = i; j < hyps_sorted.size(); j++) {
            if ((hyps_sorted[i].p - hyps_sorted[j].p).weightedNorm(1) < 0.6f) {
                to_merge.push_back(hyps_sorted[j]);
            } else {
                remaining.push_back(hyps_sorted[j]);
            }
        }

        point_2d p;
        std::vector<float> angles;
        float qual = 0;
        float match_dist = std::numeric_limits<float>::infinity();
        Hypothesis::Trust trust = Hypothesis::Trust::LOW;
        std::set<size_t> line_ids;
        std::set<size_t> point_feature_ids;

        for (const Hypothesis& h : to_merge) {
            p += h.p.point();
            angles.push_back(h.p.a);
            qual = std::max(qual, h.qual);
            if (h.trust == Hypothesis::Trust::HIGH) {
                trust = h.trust;
            } else if (h.trust == Hypothesis::Trust::MEDIUM && trust != Hypothesis::Trust::HIGH) {
                trust = h.trust;
            }
            if (h.match_dist < match_dist) {
                match_dist = h.match_dist;
            }
            line_ids.insert(h.line_ids.begin(), h.line_ids.end());
            point_feature_ids.insert(h.point_feature_ids.begin(), h.point_feature_ids.end());
        }

        Hypothesis hyp{.p = {{p / to_merge.size()}, angleAvg(angles)},
                       .qual = qual,
                       .trust = trust,
                       .match_dist = match_dist,
                       .line_ids = line_ids,
                       .point_feature_ids = point_feature_ids};

        if (to_merge.size() > 1) {
            // visualize(hyp, cam_pose.head_angles.yaw, Color::BROWN);

            char buffer[32];
            snprintf(buffer, 32, "(%zu, %zu)", hyp.line_ids.size(), hyp.point_feature_ids.size());
            // vis->addShape(new Text2D(hyp.p.point() + point_2d{0, 0.15}, 12, buffer,
            // Color::BROWN, 100));

            if (hyp.line_ids.size() + hyp.point_feature_ids.size() >
                max_lines_merged + max_points_merged) {
                max_lines_merged = hyp.line_ids.size();
                max_points_merged = hyp.point_feature_ids.size();
            }
        }

        hyps_sorted.resize(i);
        hyps_sorted.push_back(hyp);
        hyps_sorted.insert(hyps_sorted.end(), remaining.begin(), remaining.end());
    }

    if (max_lines_merged + max_points_merged > 0) {
        char buffer[32];
        snprintf(buffer, 32, "%zu, %zu", max_lines_merged, max_points_merged);
        // vis->addShape(new Text2D(point_2d{- 0.05f, -SoccerField::width()/2 - 0.10f}, 12,
        // buffer, Color::BROWN, 100));
    }

    sort(hyps_sorted, [](const Hypothesis& h1, const Hypothesis& h2) {
        return h1.line_ids.size() + h1.point_feature_ids.size() >
               h2.line_ids.size() + h2.point_feature_ids.size();
    });
    std::erase_if(hyps_sorted, [&max_lines_merged, &max_points_merged](const Hypothesis& h) {
        return h.line_ids.size() + h.point_feature_ids.size() <
               max_lines_merged + max_points_merged - 1;
    });

    return hyps_sorted;
}

vector<Hypothesis> Localization::hypothesesCircle(const Ellipse& ellipse,
                                                  const vector<LineGroup>& line_groups,
                                                  CamPose& cam_pose) {
    // EASY_FUNCTION(profiler::colors::Cyan200);
    if (auto line = findChord(ellipse, line_groups)) {
        if (auto angle = angleFromEllipse(ellipse)) {
            cam_pose.setEllipseAngles(*angle);
            if (auto projected = CamUtils::project(*line, cam_pose)) {
                // const Color& color = Color::WHITE;
                // VisTransPtr vis =
                //         Visualizer::instance().startTransaction({}, "ProjectedLines",
                //         RELATIVE_BODY, NO_REPLACE);
                // vis->addShape(new Ellipse2D((projected->p1() + projected->p2()) / 2.f,
                // SoccerField::circleDiameter(),
                //                             SoccerField::circleDiameter(), color, 1000));
                // Visualizer::instance().commit(vis);
                float a = (projected->p2() - projected->p1()).angle_to({0, 1});
                point_2d pos = (-(projected->p1() + projected->p2()) / 2.f).rotated(a);
                // Circle-based hypotheses are more accurate due to angle correction.
                Hypothesis hyp{.p = {pos, a},
                               .qual = 1,
                               .trust = Hypothesis::Trust::HIGH,
                               .match_dist = pos.norm() / 2,
                               .line_ids = {0},
                               .point_feature_ids = {0}};
                return {hyp, hyp.mirrored()};
            } else {
                cam_pose.removeEllipseAngles();
            }
        }
    }
    return {};
}

pair<vector<Hypothesis>, float /*matched_lines*/> Localization::hypothesesLines(
        const vector<Line>& projected) {
    // EASY_FUNCTION(profiler::colors::Cyan100);
    too_many_lines = false;
    // Don't run loc on lower cam if we're close to the center circle since we detect the center
    // circle as lines.
    if (projected.size() < 2)
        return {};
    // Split lines into 2 groups that are orthogonal to each other. (Which makes lines in the
    // same group parallel to each other)
    vector<Line> group1;
    vector<Line> group2;
    for (const Line& line : projected) {
        float angle = projected.begin()->angle_to(line);
        if (abs(angle) < 20_deg || abs(angle) > 160_deg)
            group1.push_back(line);
        else if (abs(angle) > 70_deg && abs(angle) < 110_deg)
            group2.push_back(line);
    }

    const int max_line_count = 8;
    if (group1.size() > max_line_count || group2.size() > max_line_count) {
        too_many_lines = true;
        return {};
    }

    // We can't get a position from parallel lines only. (yet)
    if (group2.empty())
        return {};
    float max_matches = 0;
    vector<Hypothesis> hypotheses;
    for (const auto& line1 : group1) {
        for (const auto& line2 : group2) {
            if (auto intersection = line1.intersect(line2)) {
                zip(line_intersections, line_pairs,
                    [&](const point_2d& model_intersection, const auto& line_ids) {
                        float angle_offset =
                                (line1.u().normalized() + line2.u().normalized()).angle_to({1, 1});
                        for (float quarter : {0.f, 90_deg, 180_deg, 270_deg}) {
                            float angle = normalizeRotation(angle_offset + quarter);
                            point_2d pos = (-*intersection).rotated(angle) + model_intersection;
                            if (abs(pos.x) > SoccerField::length() / 2.f + 1.f ||
                                abs(pos.y) > SoccerField::width() / 2.f + 1.f)
                                continue;
                            Line global1 =
                                    (line1 - *intersection).rotated(angle) + model_intersection;
                            Line global2 =
                                    (line2 - *intersection).rotated(angle) + model_intersection;
                            if ((matches(field_lines[line_ids.first], global1) &&
                                 matches(field_lines[line_ids.second], global2)) ||
                                (matches(field_lines[line_ids.first], global2) &&
                                 matches(field_lines[line_ids.second], global1))) {
                                vector<Line> global;
                                global.reserve(group1.size() + group2.size());
                                transform(group1, std::back_inserter(global), [&](Line& l) {
                                    return (l - *intersection).rotated(angle) + model_intersection;
                                });
                                transform(group2, std::back_inserter(global), [&](Line& l) {
                                    return (l - *intersection).rotated(angle) + model_intersection;
                                });
                                float lines_matching =
                                        count_matches(field_lines, global) +
                                        (matches_any(field_lines, global[0]) ? 0.25f : 0);
                                std::set<size_t> matching_ids;
                                for (size_t i = 0; i < field_lines.size(); i++) {
                                    for (const Line& percept_line : global) {
                                        if (matches(field_lines[i], percept_line)) {
                                            matching_ids.insert(i);
                                        }
                                    }
                                }
                                Hypothesis hyp{.p = {pos, angle},
                                               .qual = lines_matching,
                                               .trust = lines_matching >= 3
                                                                ? Hypothesis::Trust::HIGH
                                                                : Hypothesis::Trust::MEDIUM,
                                               .match_dist = intersection->norm(),
                                               .line_ids = matching_ids};
                                if (lines_matching > max_matches) {
                                    max_matches = lines_matching;
                                    hypotheses = {hyp, hyp.mirrored()};
                                } else if (lines_matching == max_matches) {
                                    if (!any_of(hypotheses, [&hyp](const Hypothesis& old_hyp) {
                                            return (hyp.p - old_hyp.p).weightedNorm(1) < .6f;
                                        })) {
                                        hypotheses.push_back(hyp);
                                        hypotheses.push_back(hyp.mirrored());
                                    }
                                }
                            }
                        }
                    });
            }
        }
    }
    return {hypotheses, max_matches};
}

std::optional<Hypothesis> Localization::pointFeatureTriangulation(const point_2d& p1,
                                                                  const point_2d& p2, float d1,
                                                                  float d2) {
    // Calculate the distance between the two known points
    float q = (p2 - p1).norm();

    // Check if the problem has a solution
    if (q < 0.5 /* no feature is this near together */ || q > d1 + d2 || q < std::abs(d1 - d2)) {
        return std::nullopt;
    }

    // Calculate the coordinates of the unknown point
    float x = (d1 * d1 - d2 * d2 + q * q) / (2 * q);
    float y = -std::sqrt(d1 * d1 - x * x);

    // printf("q,x,y,d1,d2: %.2f, %.2f, %.2f, %.2f, %.2f\n", q, x, y, d1, d2);
    // fflush(stdout);

    // Rotate and translate the solution
    float angle = std::atan2(p2.y - p1.y, p2.x - p1.x);
    float xRotated = x * std::cos(angle) - y * std::sin(angle);
    float yRotated = x * std::sin(angle) + y * std::cos(angle);

    point_2d result;
    result.x = xRotated + p1.x;
    result.y = yRotated + p1.y;

    float length_half = SoccerField::length() / 2.f;
    float width_half = SoccerField::width() / 2.f;

    // outside of field is discarded
    if (result.x < -length_half || result.x > length_half)
        return std::nullopt;

    if (result.y < -width_half || result.y > width_half)
        return std::nullopt;

    // Calculate the angle of the position
    float positionAngle = std::atan2(result.y, result.x);

    // Ensure the angle is in the range [0, 2π)
    if (positionAngle < 0) {
        positionAngle += 2 * M_PI;
    }

    // printf("x,y,a: %.2f, %.2f, %.2f\n", result.x, result.y, positionAngle);
    Hypothesis hyp;
    hyp.p = Position(result, -positionAngle);

    return hyp;
}

vector<Hypothesis> Localization::hypothesesPointFeatures(
        const vector<ProjectedPointFeature>& projected_points, vector<Line>& projected) {
    // EASY_FUNCTION(profiler::colors::Cyan400);
    if (too_many_lines)
        return {};

    const auto& in_field = [](const point_2d& p) {
        return std::abs(p.x) < SoccerField::length() / 2 * 1.2f &&
               std::abs(p.y) < SoccerField::width() / 2 * 1.2f;
    };

    vector<Hypothesis> hypotheses;
    map<ObjectType, vector<Hypothesis>> vis_hypotheses;
    for (const auto& proj : projected_points) {
        point_2d p = proj.pos;
        if (proj.type == ObjectType::PENALTY_SPOT && initialized) {
            // TODO: Do we still need this? Do we need more of these?
            // Don't start processing the penalty point if it could be a misclassification
            // somewhere around the center circle.
            point_2d abs_penalty = LocalizationUtils::relToAbs(p, position);
            if (abs_penalty.norm() < SoccerField::circleDiameter())
                continue;
        }
        for (const PointFeature& model : point_feature_model[proj.type]) {
            for (Line& l : projected) {
                float dist = l.extendedDistance(p);

                for (const FeatureLine& f : model.lines) {
                    if (!within(dist, 0.75f * (f.dist - 0.15f), 1.25f * (f.dist + 0.15f)))
                        continue;

                    float a = normalizeRotation(f.angle - l.u().to_direction());
                    point_2d trans = model.pos - p.rotated(a);
                    if (in_field(trans) &&
                        field_lines[f.id].distance(l.p1().rotated(a) + trans) <
                                0.15f + l.p1().magnitude() * 0.25f &&
                        field_lines[f.id].distance(l.p2().rotated(a) + trans) <
                                0.15f + l.p2().magnitude() * 0.25f) {
                        Hypothesis hyp{.p = {trans, a},
                                       .qual = 1.f,
                                       .trust = Hypothesis::Trust::HIGH,
                                       .match_dist = p.norm(),
                                       .line_ids = {f.id},
                                       .point_feature_ids = {model.feature_id}};
                        hypotheses.push_back(hyp);
                        vis_hypotheses[proj.type].push_back(hyp);
                        hypotheses.push_back(hyp.mirrored());
                        vis_hypotheses[proj.type].push_back(hyp.mirrored());
                    }
                    a = normalizeRotation(a + M_PIf);
                    trans = model.pos - p.rotated(a);
                    if (in_field(trans) &&
                        field_lines[f.id].distance(l.p1().rotated(a) + trans) <
                                0.15f + l.p1().magnitude() * 0.25f &&
                        field_lines[f.id].distance(l.p2().rotated(a) + trans) <
                                0.15f + l.p2().magnitude() * 0.25f) {
                        Hypothesis hyp{.p = {trans, a},
                                       .qual = 1.f,
                                       .trust = Hypothesis::Trust::HIGH,
                                       .match_dist = p.norm(),
                                       .line_ids = {f.id},
                                       .point_feature_ids = {model.feature_id}};
                        hypotheses.push_back(hyp);
                        vis_hypotheses[proj.type].push_back(hyp);
                        hypotheses.push_back(hyp.mirrored());
                        vis_hypotheses[proj.type].push_back(hyp.mirrored());
                    }
                }
            }
        }
    }

    /******************** Here begins point to point feature hyps *********************/
    for (auto p1_it = projected_points.begin(); p1_it != projected_points.end(); ++p1_it) {
        const point_2d& rel1 = p1_it->pos;
        const ObjectType type1 = p1_it->type;

        for (auto p2_it = std::next(p1_it); p2_it != projected_points.end(); ++p2_it) {
            const point_2d& rel2 = p2_it->pos;
            const ObjectType type2 = p2_it->type;

            point_2d rel_vec = rel2 - rel1;

            if (rel_vec.norm() < 0.5f)
                continue;  // Too close, likely same feature

            for (const PointFeature& model1 : point_feature_model[type1]) {
                for (const PointFeature& model2 : point_feature_model[type2]) {
                    point_2d model_vec = model2.pos - model1.pos;
                    float model_len = model_vec.norm();
                    float rel_len = rel_vec.norm();

                    if (std::abs(model_len - rel_len) >
                        0.6f + 0.1f * (rel1.norm() + rel2.norm()) / 2.f)
                        continue;  // Length mismatch with tolerance

                    // Compute angle a such that rel_vec.rotated(a) == model_vec
                    float a = model_vec.to_direction() - rel_vec.to_direction();
                    a = normalizeRotation(a);

                    // Compute position using first feature
                    point_2d pos = model1.pos - rel1.rotated(a);

                    // Verify with second (should be close)
                    point_2d pos_from2 = model2.pos - rel2.rotated(a);
                    if ((pos - pos_from2).norm() > 0.2f)
                        continue;

                    if (!in_field(pos))
                        continue;

                    Hypothesis hyp{.p = {pos, a},
                                   .qual = 0.8f,  // Slightly less than line-based
                                   .trust = Hypothesis::Trust::MEDIUM,
                                   .match_dist = (rel1.norm() + rel2.norm()) / 2.f,
                                   .point_feature_ids = {model1.feature_id, model2.feature_id}};

                    hypotheses.push_back(hyp);
                    vis_hypotheses[type1].push_back(hyp);
                    hypotheses.push_back(hyp.mirrored());
                    vis_hypotheses[type1].push_back(hyp.mirrored());

                    // Try flipped orientation
                    float a_flipped = normalizeRotation(a + M_PIf);
                    pos = model1.pos - rel1.rotated(a_flipped);

                    pos_from2 = model2.pos - rel2.rotated(a_flipped);
                    if ((pos - pos_from2).norm() > 0.2f)
                        continue;

                    if (!in_field(pos))
                        continue;

                    Hypothesis hyp_flipped{
                            .p = {pos, a_flipped},
                            .qual = 1.f,
                            .trust = Hypothesis::Trust::MEDIUM,
                            .match_dist = (rel1.norm() + rel2.norm()) / 2.f,
                            .point_feature_ids = {model1.feature_id, model2.feature_id}};

                    hypotheses.push_back(hyp_flipped);
                    vis_hypotheses[type1].push_back(hyp_flipped);
                    hypotheses.push_back(hyp_flipped.mirrored());
                    vis_hypotheses[type1].push_back(hyp_flipped.mirrored());
                    std::cout << "Point hyp from " << (int)type1 << "+" << (int)type2 << " at ("
                              << hyp.p.x << "," << hyp.p.y << "," << hyp.p.a << ")" << std::endl;
                }
            }
        }
    }

    for (const auto& [type, hyps] : vis_hypotheses) {
        // Color color = colors[type];
        // color = color.brighter();

        // visualize(hyps, cam_pose.head_angles.yaw, color);
    }
    return hypotheses;
}
