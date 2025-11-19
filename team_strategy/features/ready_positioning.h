#pragma once

#include <map>
#include <string>
#include <vector>

#include "position.h"
#include "tc_data.h"

std::map<size_t, std::vector<htwk::Position>> readTacticsFile(const std::string& file);
std::vector<size_t> permutePositions_play(const std::map<PlayerIdx, TeamComData>& robots,
                                          const std::vector<htwk::Position>& positions,
                                          bool have_goalie, std::optional<PlayerIdx> cur_striker);
std::vector<size_t> permutePositions_goaly0(const std::map<PlayerIdx, TeamComData>& robots,
                                            const std::vector<htwk::Position>& positions,
                                            bool have_goalie);
std::optional<htwk::Position> robocupReadyPositioning(
        const std::vector<htwk::Position>& positions_tmp,
        const std::map<PlayerIdx, TeamComData>& robots, PlayerIdx player_idx);
