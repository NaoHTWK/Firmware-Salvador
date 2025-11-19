#include "position_util.h"

#include <map>
#include <tuple>

#include "algorithm_ext.h"

using namespace std;

#ifdef TS_DEBUG
constexpr bool saveScores = true;
#else
constexpr bool saveScores = false;
#endif

static map<tuple<float, float>, float> generatedPositionScores;

std::map<std::tuple<float, float>, float> getGeneratedPositionScores() {
    return generatedPositionScores;
}

void clearGeneratedPositionScores() {
    generatedPositionScores.clear();
}

point_2d generatePosition(const PositionArea& posArea,
                          const function<float(const point_2d&)>& scoreFunc) {
    if constexpr (saveScores)
        generatedPositionScores.clear();

    float min_score = numeric_limits<float>::infinity();
    point_2d optimized_pos(0, 0);
    const float temp_endPoint_y =
            posArea.endPoint.y + posArea.stepSize.y;  // can not check for equal (float)
    const float temp_endPoint_x =
            posArea.endPoint.x + posArea.stepSize.x;  // can not check for equal (float)
    for (float y = posArea.startPoint.y; y < temp_endPoint_y; y += posArea.stepSize.y) {
        for (float x = posArea.startPoint.x; x < temp_endPoint_x; x += posArea.stepSize.x) {
            float score = scoreFunc({x, y});
            if constexpr (saveScores)
                generatedPositionScores[make_tuple<>(x, y)] = score;

            if (score < min_score) {
                min_score = score;
                optimized_pos = {x, y};
            }
        }
    }
    return optimized_pos;
}

float distToClosestNeighbor(const point_2d& pos, const map<PlayerIdx, TeamComData>& alive_robots,
                            PlayerIdx player_idx) {
    return pos.dist(min_element_transformed(alive_robots, [&player_idx, &pos](const auto& r) {
                        return r.first == player_idx ? numeric_limits<float>::infinity()
                                                     : pos.dist(r.second.pos.point());
                    })->second.pos.point());
}

optional<float> distToClosestNeighborGreaterX(const point_2d& pos,
                                              const map<PlayerIdx, TeamComData>& alive_robots,
                                              PlayerIdx player_idx) {
    auto it = min_element_transformed(alive_robots, [&player_idx, &pos](const auto& r) {
        return r.first == player_idx || pos.x > r.second.pos.x ? numeric_limits<float>::infinity()
                                                               : abs(pos.x - r.second.pos.x);
    });
    float dist_x = abs(pos.x - it->second.pos.x);
    float min_val = it->first == player_idx || pos.x > it->second.pos.x
                            ? numeric_limits<float>::infinity()
                            : dist_x;
    return std::isinf(min_val) ? std::nullopt : optional<float>(dist_x);
}

optional<float> distToClosestNeighborLesserX(const point_2d& pos,
                                             const map<PlayerIdx, TeamComData>& alive_robots,
                                             PlayerIdx player_idx) {
    auto it = min_element_transformed(alive_robots, [&player_idx, &pos](const auto& r) {
        return r.first == player_idx || pos.x < r.second.pos.x ? numeric_limits<float>::infinity()
                                                               : abs(pos.x - r.second.pos.x);
    });
    float dist_x = abs(pos.x - it->second.pos.x);
    float min_val = it->first == player_idx || pos.x < it->second.pos.x
                            ? numeric_limits<float>::infinity()
                            : dist_x;
    return std::isinf(min_val) ? std::nullopt : optional<float>(dist_x);
}

PlayerIdx nearestPlayerToPoint(const point_2d& point,
                               const map<PlayerIdx, TeamComData>& alive_robots) {
    return min_element(
                   alive_robots,
                   [point](const auto& a, const auto& b) {
                       return (point.dist(a.second.pos.point()) < point.dist(b.second.pos.point()));
                   })
            ->first;
}

PlayerIdx nearestPlayerToLine(htwk::Line& line, const map<PlayerIdx, TeamComData>& alive_robots) {
    return min_element(alive_robots,
                       [&line](const auto& a, const auto& b) {
                           if (line.innerDistance(a.second.pos.point()))
                               return false;
                           if (line.innerDistance(b.second.pos.point()))
                               return true;
                           return (line.innerDistance(a.second.pos.point()) <
                                   line.innerDistance(b.second.pos.point()));
                       })
            ->first;
}

PlayerIdx nearestFieldPlayerToPoint(const point_2d& point, const set<PlayerIdx>& except,
                                    const map<PlayerIdx, TeamComData>& alive_robots) {
    map<PlayerIdx, TeamComData> field_robots;
    insert_if(alive_robots, field_robots,
              [&except](const auto& alive_robot) { return !contains(except, alive_robot.first); });
    return min_element(
                   field_robots,
                   [point](const auto& a, const auto& b) {
                       return (point.dist(a.second.pos.point()) < point.dist(b.second.pos.point()));
                   })
            ->first;
}

PlayerIdx nearestPlayerToPlayer(const PlayerIdx player_idx,
                                const std::map<PlayerIdx, TeamComData>& alive_robots) {
    return min_element(
                   alive_robots,
                   [&alive_robots, player_idx](const auto& a, const auto& b) {
                       if (a.second.player_idx == player_idx)
                           return false;
                       return (alive_robots.at(player_idx).pos.point().dist(a.second.pos.point()) <
                               alive_robots.at(player_idx).pos.point().dist(b.second.pos.point()));
                   })
            ->first;
}

int playerWithinArea(const map<PlayerIdx, TeamComData>& alive_robots, PlayerIdx player_idx,
                     const PositionArea& posArea) {
    int counter = 0;
    for (auto& bot : alive_robots) {
        if (bot.first != player_idx && posArea.isWithinArea(bot.second.pos.point()))
            counter++;
    }
    return counter;
}

point_2d smoothPosition(std::deque<point_2d>& history, size_t max_size) {
    while (history.size() > max_size) {
        history.pop_front();
    }

    point_2d avg_pos = accumulate(history, point_2d(0.f, 0.f));
    return avg_pos / history.size();
}

point_2d maxPosition(std::deque<point_2d>& history, size_t max_size) {
    while (history.size() > max_size) {
        history.pop_front();
    }

    float maxY = 0;
    for (const point_2d& pos : history) {
        if (abs(pos.y) > abs(maxY)) {
            maxY = pos.y;
        }
    }

    point_2d avg_pos = accumulate(history, point_2d(0.f, 0.f));
    return point_2d{avg_pos.x, maxY};
}
