#pragma once

#include <map>
#include <memory>
#include <optional>
#include <vector>

#include "field_defender.h"
#include "gc_state.h"
#include "line_defender.h"
#include "penalty_kick.h"
#include "position.h"
#include "soccerfield.h"
#include "team_strategy.h"

class RobocupStrategy : public TeamStrategy {
public:
    RobocupStrategy(uint8_t team_nr, PlayerIdx player_idx);

    enum class Positioning { STRIKER = 0, GOALIE, COVER_MID, SHADOW, COVER, COVER2, SHADOW2 };
    enum class PlayState {
        WAITING_FOR_OWN_KICKOFF = 0,
        WAITING_FOR_OPP_KICKOFF,
        WAITING_FOR_OPP_SETPLAY,
        PLAYING
    };

    const std::map<PlayerIdx, Positioning>& getRoleByPlayer() const {
        return role_by_player;
    }

    const std::map<Positioning, point_2d>& getPositionByRole() const {
        return position_by_role;
    }

protected:
    std::shared_ptr<Order> ready() override;
    std::shared_ptr<Order> set() override;
    std::shared_ptr<Order> play() override;
    std::shared_ptr<Order> setPlay_initial() override;
    std::shared_ptr<Order> setPlay_ready() override;
    std::shared_ptr<Order> setPlay_set() override;

private:
    static constexpr PlayerIdx goaly_idx = 0;
    static constexpr float defensive_line_thres = -0.75f;
    static constexpr int no_switch_ready_state_after = 10;
    static constexpr point_2d middle_point = point_2d(0.f, 0.f);
    const float ball_moved_by_enemy = SoccerField::circleDiameter() / 2.f;
    const point_2d own_goal = point_2d(-SoccerField::length() * 0.5f, 0);
    const point_2d opp_goal = point_2d(SoccerField::length() * 0.5f, 0);
    const float fieldwidth_offset = SoccerField::width() * .5f * .9f;
    const float penaltyarea_border =
            -SoccerField::length() * 0.5 + SoccerField::penaltyAreaWidth() + 0.5f;

    std::shared_ptr<Order> handlePenaltyReturn();
    point_2d globalBall(int64_t min_time = 0);
    bool isBallFreeInKickoff(const TeamComData& robot);
    bool isBallFreeInKickoff(const Myself& robot);
    bool isBallFreeInSetPlay(const Myself& robot);
    bool isBallFreeInSetPlay(const TeamComData& robot);
    [[nodiscard]] std::shared_ptr<Order> setPlay_offensive(const point_2d global_ball);
    [[nodiscard]] std::shared_ptr<Order> setPlay_defensive(const point_2d global_ball);

    void simulate_positions(int num_robots, const point_2d& ball);
    point_2d generatePosition(Positioning pb, const point_2d& ball);
    void distributePositions();
    point_2d manipulate_position(const point_2d& ball);

    point_2d positioning_goalie(const point_2d& ball);
    point_2d positioning_shadow(const point_2d& ball);
    point_2d positioning_shadow_mirror(const point_2d& ball, const int side);
    point_2d positioning_cover(const point_2d& ball, const float min_ball_distance,
                               const float max_ball_distance, const float min_goal_distance);
    point_2d positioning_cover_mid(const point_2d& ball, const float min_ball_distance,
                                   const float max_ball_distance, const float min_goal_distance);
    point_2d positioning_cover_duo_une(const point_2d& ball, const float min_ball_distance,
                                       const float max_ball_distance, const float min_goal_distance,
                                       const float lo, const float hi);
    point_2d positioning_cover_duo_deux(const point_2d& ball, const float min_ball_distance,
                                        const float max_ball_distance,
                                        const float min_goal_distance, const float lo,
                                        const float hi);
    void selectMarkTarget();
    point_2d positioning_mark();
    void determineStriker(std::optional<point_2d> striker_pos = std::nullopt,
                          bool high_quality = true);
    bool strikerQualityBall(const TeamComData& robot, bool high_quality = true);
    bool isRogueStriker();
    bool isStupidStriker(point_2d global_ball);

    PenaltyKick setPlay_penalty_kick;
    LineDefender setPlay_line_def;
    FieldDefender setPlay_field_def;
    point_2d setPlay_ball_pos = {0, 0};
    std::map<size_t /*num_active*/, std::vector<htwk::Position>> offensive_kickoff_positions;
    std::map<size_t /*num_active*/, std::vector<htwk::Position>> defensive_kickoff_positions;
    std::optional<PlayerIdx> cur_striker;
    std::map<PlayerIdx, Positioning> role_by_player;
    std::map<Positioning, point_2d> position_by_role;
    std::optional<point_2d> last_seen_ball;
    int64_t last_seen_ball_time = 0;
    PlayState play_state = PlayState::WAITING_FOR_OWN_KICKOFF;
    std::optional<Positioning> cur_role;

    std::optional<htwk::Position> fixed_ready_pos;

    // playing
    bool marking_enabled = false;
    point_2d position_filter{0, 0};
    bool sent_striker = false;
    bool short_kick = false;
    point_2d shoot_target{4.5f, 0.f};
};
