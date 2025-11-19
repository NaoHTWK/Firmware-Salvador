#include "gc_state.h"

bool GCState::operator==(const GCState& other) const {
    return player_idx == other.player_idx && my_team.team_nr == other.my_team.team_nr &&
           opp_team.team_nr == other.opp_team.team_nr && state == other.state &&
           secondary_state == other.secondary_state && kicking_team == other.kicking_team &&
           secs_remaining == other.secs_remaining && secondary_time == other.secondary_time &&
           my_team.players.size() == other.my_team.players.size() &&
           opp_team.players.size() == other.opp_team.players.size() &&
           std::equal(my_team.players.begin(), my_team.players.end(), other.my_team.players.begin(),
                      [](const Player& a, const Player& b) {
                          return a.is_penalized == b.is_penalized;
                      }) &&
           std::equal(opp_team.players.begin(), opp_team.players.end(),
                      other.opp_team.players.begin(),
                      [](const Player& a, const Player& b) {
                          return a.is_penalized == b.is_penalized;
                      }) &&
           game_phase == other.game_phase &&
           remaining_message_budget == other.remaining_message_budget;
}

bool GCState::operator!=(const GCState& other) const {
    return !(*this == other);
}
