#include "team_strategy_factory.h"

#include "moveball_strategy.h"
#include "penalty_strategy.h"
#include "robocup_strategy.h"
#include "stay_strategy.h"
#include "tutorial_strategy.h"

std::unique_ptr<TeamStrategy> TeamStrategyFactory::create(const std::string& strategy_name,
                                                          uint8_t team_nr, PlayerIdx player_idx) {
    if (strategy_name == "penalty") {
        return std::make_unique<PenaltyStrategy>(team_nr, player_idx);
    } else if (strategy_name == "robocup") {
        return std::make_unique<RobocupStrategy>(team_nr, player_idx);
    } else if (strategy_name == "tutorial") {
        return std::make_unique<TutorialStrategy>(team_nr, player_idx);
    } else if (strategy_name == "stay") {
        return std::make_unique<StayStrategy>(team_nr, player_idx);
    } else if (strategy_name == "moveball") {
        return std::make_unique<MoveBallStrategy>(team_nr, player_idx);
    }
    throw std::invalid_argument("Unknown teamstrategy of name:" + strategy_name);
}
