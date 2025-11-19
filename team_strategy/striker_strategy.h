#pragma once

#include <map>
#include <memory>
#include <optional>
#include <vector>

#include "order.h"
#include "point_2d.h"
#include "soccerfield.h"

class StrikerStrategy {
public:
    StrikerStrategy(PlayerIdx player_idx) : player_idx(player_idx) {}
    std::shared_ptr<Order> proceed(const TeamBall& teamball,
                                   const std::map<PlayerIdx, Robot>& alive_robots,
                                   const std::multimap<float, ObstacleObservation>& obstacles_by_x);
    std::shared_ptr<Order> proceed_kickoff(const TeamBall& teamball,
                                           const std::map<PlayerIdx, Robot>& alive_robots,
                                           const std::vector<ObstacleObservation>& obstacles);
    void reset();

private:
    static constexpr point_2d middle_point = point_2d(0.f, 0.f);
    static constexpr point_2d opp_goal = point_2d(4.5f, 0.f);
    const PlayerIdx player_idx;
    TeamBall teamball;
    std::map<PlayerIdx, Robot> alive_robots;
    std::optional<PlayerIdx> passCandidate;

    void selectPassCandidate(int y_indicator);
};
