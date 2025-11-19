#include <map>
#include <mutex>
#include <vector>

#include "localization_utils.h"
#include "point_2d.h"
#include "position.h"
#include "soccer_field_visualizer.h"
#include "soccerfield.h"
#include "soccerfielddefinitions.h"
#include "tc_network.h"
#include "tc_pub_sub.h"

int main(int, char**) {
    SoccerField::configureSoccerField(getSoccerFieldKidSize(), 0);
    SoccerFieldVisualizer visualizer("Team Strategy Visualizer");

    std::map<PlayerIdx, TeamComData> team_data;
    std::mutex team_data_mutex;
    std::cout << "Starting TeamComNetwork" << std::endl;

    TeamComNetwork tc_network(14, false);
    tc_internal::receive.registerCallback([&](const TeamComData& data) {
        std::lock_guard<std::mutex> lock(team_data_mutex);
        team_data[data.player_idx] = data;
        std::cout << "Received data for player " << data.player_idx << std::endl;
    });

    std::cout << "Starting main loop" << std::endl;
    while (!visualizer.windowShouldClose()) {
        visualizer.beginFrame();

        std::vector<RobotDrawInfo> robots;
        std::optional<point_2d> ball_position;

        {
            std::lock_guard<std::mutex> lock(team_data_mutex);
            TeamComData const* ball_provider = nullptr;
            for (auto const& [idx, data] : team_data) {
                robots.push_back(RobotDrawInfo{.id = data.player_idx,
                                               .active = true,
                                               .pos = data.pos,
                                               .walk_target = std::nullopt,
                                               .type = RobotDrawInfo::RobotType::TEAMMATE});
                if (data.ball) {
                    if (!ball_provider ||
                        data.ball->ball_age_us < ball_provider->ball->ball_age_us) {
                        ball_provider = &data;
                    }
                }
            }

            for (auto const& [idx, data] : team_data) {
                for (const auto& detected_robot : data.robots) {
                    RobotDrawInfo::RobotType type = RobotDrawInfo::RobotType::UNKNOWN;
                    if (detected_robot.ownTeamProb > 0.8) {
                        type = RobotDrawInfo::RobotType::TEAMMATE;
                    } else if (detected_robot.ownTeamProb < 0.2) {
                        type = RobotDrawInfo::RobotType::OPPONENT;
                    }

                    point_2d abs_pos_2d =
                            LocalizationUtils::relToAbs(detected_robot.pos_rel, data.pos);
                    htwk::Position abs_pos = {abs_pos_2d.x, abs_pos_2d.y, 0.f};

                    robots.push_back(RobotDrawInfo{.id = -1,  // No ID for detected robots
                                                   .active = true,
                                                   .pos = abs_pos,
                                                   .walk_target = std::nullopt,
                                                   .type = type});
                }
            }
            if (ball_provider) {
                ball_position = LocalizationUtils::relToAbs(ball_provider->ball->pos_rel,
                                                            ball_provider->pos);
            }
        }

        ImGui::Begin("Visualizer");
        ImGui::Text("Visualizing TeamCom data.");
        ImGui::End();

        visualizer.drawFieldAndRobots(robots, ball_position);

        visualizer.endFrame();
    }

    return 0;
}