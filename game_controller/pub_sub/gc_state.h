#pragma once

#include <cstdint>
#include <vector>

using PlayerIdx = uint8_t;
using TeamIdx = uint8_t;

struct Player {
    bool is_penalized;
};

struct Team {
    TeamIdx team_nr;
    std::vector<Player> players;
};

// Keep in sync with RoboCupGameControlData.h
enum class GameState : uint8_t {
    Initial = 0,
    Ready = 1,
    Set = 2,
    Playing = 3,
    Finished = 4,
    Standby = 5
};

// Keep in sync with RoboCupGameControlData.h
enum class SecondaryState : uint8_t {
    Normal = 0,
    PenaltyShoot = 1,
    Overtime = 2,
    Timeout = 3,
    DirectFreeKick = 4,
    IndirectFreeKick = 5,
    PenaltyKick = 6,
    CornerKick = 7,
    GoalKick = 8,
    ThrowIn = 9
};

enum class KickingTeam : uint8_t { Unknown = 0, MyTeam, OppTeam, Both };

// In Humanoid, set plays work like this:
// 1. In the "initial" phase (when the penalty is announced), game_state will be playing,
// secondary_state has the set play type, setPlay_kicking_team is set, and setPlay_game_state is set
// to initial.
// 2. In the "ready" phase (after the ball is placed and the referee announces to prepare),
// game_state will be playing, secondary_state has the set play type, setPlay_kicking_team is set,
// and setPlay_game_state is set to ready.
// 3. In the "set" phase (after the referee announces the end of prepare),
// game_state will be playing, secondary_state has the set play type, setPlay_kicking_team is set,
// and setPlay_game_state is set to set.
// 4. At the end of the set play (after the referee announces to play),
// game_state will be playing, secondary_state is none(!), setPlay_kicking_team is set,
// and setPlay_game_state is still set to set(!).
struct GCState {
    enum class GamePhase { FIRST_HALF, SECOND_HALF, PENALTY_SHOOT };

    PlayerIdx player_idx;
    Team my_team;
    Team opp_team;

    GamePhase game_phase;
    GameState state = GameState::Initial;
    SecondaryState secondary_state = SecondaryState::Normal;
    KickingTeam kicking_team;
    KickingTeam setPlay_kicking_team = KickingTeam::Unknown;
    GameState setPlay_state = GameState::Initial;
    uint16_t secs_remaining;  // seconds remaining in the half
    uint16_t secondary_time;  // seconds remaining for secondary time (e.g., ready phase)
    uint16_t remaining_message_budget;
    bool is_fake_gc = false;

    bool operator==(const GCState& other) const;
    bool operator!=(const GCState& other) const;
};

class GCInterface {
public:
    virtual ~GCInterface() = default;
};