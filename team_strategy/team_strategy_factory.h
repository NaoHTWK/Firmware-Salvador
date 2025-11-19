#pragma once

#include <memory>
#include <string>

#include "tc_data.h"
#include "team_strategy.h"

class TeamStrategyFactory {
public:
    static std::unique_ptr<TeamStrategy> create(const std::string& strategy_name, uint8_t team_nr,
                                                PlayerIdx player_idx);
};
