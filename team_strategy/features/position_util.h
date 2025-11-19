#pragma once

#include <map>
#include <optional>
#include <set>
#include <tuple>

#include "line.h"
#include "position_area.h"
#include "tc_data.h"

point_2d generatePosition(const PositionArea& posArea,
                          const std::function<float(const point_2d&)>& scoreFunc);

/**
 * @brief getGeneratedPositionScores Return all scores and positions which where generated via
 * generatePosition
 * @return key Position(x, y), value is the score
 * @note This is only interesting for debugging or team strategy debugger purposes. (if you call
 * this you are stupid :-P)
 */
std::map<std::tuple<float, float>, float> getGeneratedPositionScores();
void clearGeneratedPositionScores();

float distToClosestNeighbor(const point_2d& pos,
                            const std::map<PlayerIdx, TeamComData>& alive_robots,
                            PlayerIdx player_idx);
std::optional<float> distToClosestNeighborGreaterX(
        const point_2d& pos, const std::map<PlayerIdx, TeamComData>& alive_robots,
        PlayerIdx player_idx);
std::optional<float> distToClosestNeighborLesserX(
        const point_2d& pos, const std::map<PlayerIdx, TeamComData>& alive_robots,
        PlayerIdx player_idx);
PlayerIdx nearestPlayerToPoint(const point_2d& point,
                               const std::map<PlayerIdx, TeamComData>& alive_robots);
PlayerIdx nearestPlayerToLine(htwk::Line& line,
                              const std::map<PlayerIdx, TeamComData>& alive_robots);
PlayerIdx nearestFieldPlayerToPoint(const point_2d& point, const std::set<PlayerIdx>& except,
                                    const std::map<PlayerIdx, TeamComData>& alive_robots);
PlayerIdx nearestPlayerToPlayer(const PlayerIdx playerIdx,
                                const std::map<PlayerIdx, TeamComData>& alive_robots);
int playerWithinArea(const std::map<PlayerIdx, TeamComData>& alive_robots, PlayerIdx player_idx,
                     const PositionArea& posArea);
point_2d smoothPosition(std::deque<point_2d>& history, size_t max_size);
point_2d maxPosition(std::deque<point_2d>& history, size_t max_size);
