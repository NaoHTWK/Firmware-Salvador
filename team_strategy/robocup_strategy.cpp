#include "robocup_strategy.h"

#include <cmath>
#include <iomanip>
#include <loguru.hpp>

#include "dribbling_direction.h"
#include "gc_state.h"
#include "head_focus.h"
#include "keepgoalorder.h"
#include "localization_utils.h"
#include "moveballgoalorder.h"
#include "noorder.h"
#include "ready_positioning.h"
#include "soccerfield.h"
#include "stl_ext.h"
#include "team_strategy_pub_sub.h"
#include "vector_field.h"
#include "walking_time.h"
#include "walkrelativeorder.h"
#include "walktopositionorder.h"

using namespace htwk;
using namespace std;

namespace {

float dribbleWalkingTime(const htwk::Position& robot, const point_2d& abs_ball) {
    htwk::Position target = htwk::Position(abs_ball, dribblingDirection(abs_ball).to_direction());
    return walkingTime(robot, target, true);
}

float dribbleWalkingTime(const htwk::Position& robot, const point_2d& to_point,
                         const point_2d& abs_ball) {
    htwk::Position target = htwk::Position(to_point, dribblingDirection(abs_ball).to_direction());
    return walkingTime(robot, target, true);
}

float dribbleWalkingTime(const TeamComData& robot) {
    if (!robot.ball)
        return std::numeric_limits<float>::infinity();
    return dribbleWalkingTime(robot.pos,
                              LocalizationUtils::relToAbs(robot.ball->pos_rel, robot.pos));
}

float dribbleWalkingTime(const Myself& myself) {
    if (!myself.ball)
        return std::numeric_limits<float>::infinity();
    return dribbleWalkingTime(
            myself.loc.position,
            LocalizationUtils::relToAbs(myself.ball->pos_rel, myself.loc.position));
}

}  // namespace

RobocupStrategy::RobocupStrategy(uint8_t team_nr, PlayerIdx player_idx)
    : TeamStrategy(team_nr, player_idx),
      offensive_kickoff_positions(readTacticsFile("kickoff_1.conf")),
      defensive_kickoff_positions(readTacticsFile("kickoff_0.conf")),
      setPlay_penalty_kick(player_idx) {}

std::shared_ptr<Order> RobocupStrategy::handlePenaltyReturn() {
    if (penalized) {
        tc_manager.resetStrikerRequest();
        cur_role = std::nullopt;
        return NoOrder::create();
    }
    if (!returning_from_penalty) {
        return NoOrder::create();
    }
    if (penalty_return_us > time - 3_s && myself.loc.quality < 0.7f) {
        return WalkRelativeOrder::create(Position{0.6f, 0, 0});
    } else if (myself.loc.quality < 0.7f) {
        return NoOrder::create();
    }
    returning_from_penalty = false;
    return NoOrder::create();
}

shared_ptr<Order> RobocupStrategy::ready() {
    last_seen_ball = std::nullopt;
    last_seen_ball_time = myself.last_update_us;
    striker_channel.publish(std::nullopt);
    tc_manager.resetStrikerRequest();
    cur_striker = std::nullopt;
    cur_role = std::nullopt;
    play_state = gc_data.kicking_team == KickingTeam::MyTeam ? PlayState::WAITING_FOR_OWN_KICKOFF
                                                             : PlayState::WAITING_FOR_OPP_KICKOFF;
    std::shared_ptr<Order> order = handlePenaltyReturn();
    if (penalized || returning_from_penalty) {
        return order;
    }
    // Normal position distribution.
    if (gc_data.secondary_time > no_switch_ready_state_after) {
        auto& positions_tmp = (gc_data.kicking_team == KickingTeam::MyTeam
                                       ? offensive_kickoff_positions
                                       : defensive_kickoff_positions)[robots.size()];
        if (positions_tmp.empty()) {
            LOG_S(ERROR) << "No kickoff position for " << robots.size() << " players!!!";
            // TODO: Do something more useful in case that really happens in a game.
            return NoOrder::create();
        }
        fixed_ready_pos = robocupReadyPositioning(positions_tmp, robots, myself.idx);
        fixed_ready_pos->a = point_2d(1, 0).angle_to(point_2d{0.f, 0.f} - fixed_ready_pos->point());
    } else if (!fixed_ready_pos) {
        // The robot came back from penalty too late for the others to adjust their positions, let's
        // just have him stay where he is.
        fixed_ready_pos = myself.loc.position;
        fixed_ready_pos->a = point_2d(1, 0).angle_to(point_2d{0.f, 0.f} - fixed_ready_pos->point());
    }

    // TODO: Implement getting to a safe position if the robot is returning too slowly from the
    // opponent half.

    return WalkToPositionOrder::create(*fixed_ready_pos, WalkToPositionOrder::Mode::USE_A, true,
                                       HeadFocus::LOC);
}

shared_ptr<Order> RobocupStrategy::set() {
    last_seen_ball_time = myself.last_update_us;
    striker_channel.publish(std::nullopt);
    float min_time = numeric_limits<float>::infinity();
    PlayerIdx min_idx = 255;
    for (const auto& [idx, bot] : robots) {
        float t = dribbleWalkingTime(bot.pos, {0, 0});
        if (t < min_time) {
            min_time = t;
            min_idx = idx;
        }
    }
    if (min_idx == 255)
        cur_striker = std::nullopt;
    else {
        cur_striker = min_idx;
    }
    if (cur_striker && *cur_striker == myself.idx) {
        tc_manager.strikerRequest();
    } else {
        tc_manager.resetStrikerRequest();
    }
    switch (gc_data.kicking_team) {
        case KickingTeam::MyTeam:
            play_state = PlayState::WAITING_FOR_OWN_KICKOFF;
            break;
        case KickingTeam::OppTeam:
        case KickingTeam::Unknown:
            play_state = PlayState::WAITING_FOR_OPP_KICKOFF;
            break;
        case KickingTeam::Both:
            play_state = PlayState::PLAYING;
            break;
    }
    cur_role = std::nullopt;
    return NoOrder::create();
}

point_2d RobocupStrategy::globalBall(int64_t min_time) {
    point_2d global_ball{0, 0};
    if (myself.ball && !myself.penalized && time_us() - myself.ball->ball_age_us >= min_time) {
        global_ball = LocalizationUtils::relToAbs(myself.ball->pos_rel, myself.loc.position);
        last_seen_ball = global_ball;
        last_seen_ball_time = myself.last_update_us;
    } else if (cur_striker && robots[*cur_striker].ball &&
               time_us() - robots[*cur_striker].ball->ball_age_us >= min_time) {
        global_ball = LocalizationUtils::relToAbs(robots[*cur_striker].ball->pos_rel,
                                                  robots[*cur_striker].pos);
        last_seen_ball = global_ball;
        last_seen_ball_time = robots[*cur_striker].sent_time_us;
    } else {
        float ball_dist = numeric_limits<float>::infinity();
        PlayerIdx ball_idx = 255;
        for (const auto& [idx, bot] : robots) {
            if (bot.ball && time_us() - bot.ball->ball_age_us > min_time &&
                bot.ball->pos_rel.norm() < ball_dist) {
                ball_dist = bot.ball->pos_rel.norm();
                ball_idx = idx;
            }
        }
        if (ball_idx != 255) {
            global_ball = LocalizationUtils::relToAbs(robots[ball_idx].ball->pos_rel,
                                                      robots[ball_idx].pos);
            last_seen_ball = global_ball;
            last_seen_ball_time = robots[ball_idx].sent_time_us;
        } else if (last_seen_ball && last_seen_ball_time >= min_time) {
            global_ball = *last_seen_ball;
        } else {
            global_ball = middle_point;
        }
    }
    return global_ball;
}

void RobocupStrategy::simulate_positions(int num_robots, const point_2d& ball) {
    position_by_role.clear();
    for (int n = 0; n < num_robots; n++) {
        position_by_role.emplace(static_cast<Positioning>(n),
                                 generatePosition(static_cast<Positioning>(n), ball));
    }
}

void RobocupStrategy::determineStriker(std::optional<point_2d> striker_pos, bool high_quality) {
    int64_t max_striker_request_us = 0;
    PlayerIdx max_striker_request_idx = 255;
    for (const auto& [idx, bot] : robots) {
        if (bot.striker_request_time_us > max_striker_request_us) {
            max_striker_request_us = bot.striker_request_time_us;
            max_striker_request_idx = idx;
        }
    }
    if (max_striker_request_idx == 255)
        cur_striker = std::nullopt;
    else
        cur_striker = max_striker_request_idx;
    if (cur_striker && !strikerQualityBall(robots[*cur_striker], high_quality)) {
        cur_striker = std::nullopt;
    }
    if (cur_striker && *cur_striker == myself.idx)
        return;
    // Our clock is a bit behind, so we wouldn't be chosen even if we tried.
    if (myself.last_update_us <= max_striker_request_us)
        return;
    PlayerIdx best_striker_idx = 255;
    float best_striker_time = numeric_limits<float>::infinity();
    std::map<PlayerIdx, float> striker_times;
    for (const auto& [idx, bot] : robots) {
        if (!strikerQualityBall(bot, high_quality))
            continue;
        float t = striker_pos ? dribbleWalkingTime(bot.pos, *striker_pos, globalBall())
                              : dribbleWalkingTime(bot);
        striker_times[idx] = t;
        if (t < best_striker_time) {
            best_striker_time = t;
            best_striker_idx = idx;
        }
    }
    if (!striker_times.contains(myself.idx))
        return;
    float my_striker_time = striker_times[myself.idx];
    float cur_striker_time =
            cur_striker ? striker_times[*cur_striker] : numeric_limits<float>::infinity();
    bool better_than_cur_striker =
            !cur_striker ||
            cur_striker_time > std::max(my_striker_time * 1.1f, my_striker_time + 0.75f);
    bool reasonable_option =
            my_striker_time < std::max(best_striker_time * 1.1f, best_striker_time + 0.75f);
    // To avoid all robots sending a striker request if the previous striker loses the ball, we
    // check whether we're at least reasonably close compared to what the other robots are saying.
    // We then might end up with multiple striker requests, but they'll filter themselves based on
    // the request timestamp.
    if (better_than_cur_striker && reasonable_option) {
        tc_manager.strikerRequest();
        cur_striker = myself.idx;
    }
}

bool RobocupStrategy::isRogueStriker() {
    if (!cur_striker || *cur_striker == myself.idx || !strikerQualityBall(robots[*cur_striker]))
        return false;
    float cur_striker_ball_dist_diff =
            robots[*cur_striker].pos.point().dist(
                    LocalizationUtils::relToAbs(myself.ball->pos_rel, myself.loc.position)) -
            robots[*cur_striker].ball->pos_rel.norm();
    if (cur_striker_ball_dist_diff < 0)
        return false;
    float my_striker_time = dribbleWalkingTime(myself);
    float cur_striker_time = dribbleWalkingTime(robots[*cur_striker]);
    float striker_detour_time = walkingTimeStraight(cur_striker_ball_dist_diff);
    // We think that the striker's ball location is off (e.g. because the ball just moved past it
    // and it didn't see it, so it still thinks it's close), so we behave like a striker even though
    // we technically aren't (yet).
    return std::max(my_striker_time * 1.2f, my_striker_time + 1.5f) <
           cur_striker_time + striker_detour_time;
}

bool RobocupStrategy::isStupidStriker(point_2d global_ball) {
    if (!cur_striker) {
        float best_ball_dist = numeric_limits<float>::infinity();
        PlayerIdx best_ball_idx = 255;
        for (const auto& [idx, bot] : robots) {
            float t = dribbleWalkingTime(bot.pos, global_ball);
            if (t < best_ball_dist) {
                best_ball_dist = t;
                best_ball_idx = idx;
            }
        }
        if (best_ball_idx == myself.idx) {
            return true;
        }
    }
    if (!cur_striker || *cur_striker == myself.idx || strikerQualityBall(robots[*cur_striker]))
        return false;
    float stupid_ball_dist = dribbleWalkingTime(
            myself.loc.position, LocalizationUtils::relToAbs(robots[*cur_striker].ball->pos_rel,
                                                             robots[*cur_striker].pos));
    float striker_ball_dist = dribbleWalkingTime(robots[*cur_striker]) -
                              (myself.last_update_us - robots[*cur_striker].sent_time_us) / 1._s;
    // The striker, who is further away, tells us that the ball is right next to us.
    return std::max(stupid_ball_dist * 1.2f, stupid_ball_dist + 1.5f) < striker_ball_dist;
}

bool RobocupStrategy::strikerQualityBall(const TeamComData& robot, bool high_quality) {
    return robot.ball && (robot.ball->ball_age_us <= 2_s || !high_quality) && !robot.is_fallen;
}

// TODO: add field search
shared_ptr<Order> RobocupStrategy::play() {
    std::shared_ptr<Order> order = handlePenaltyReturn();
    std::optional<PlayerIdx> prev_striker = cur_striker;
    determineStriker();
    // There is a better striker already, so remove our outdated striker request so that we don't
    // accidentally fall back to it (e.g. if the striker gets penalized).
    if (cur_striker && *cur_striker != myself.idx)
        tc_manager.resetStrikerRequest();
    bool rogue_striker = isRogueStriker();
    point_2d global_ball = globalBall();
    bool stupid_striker = isStupidStriker(global_ball);

    if (play_state == PlayState::WAITING_FOR_OWN_KICKOFF) {
        if (time_us_inPlay >= 10_s || (cur_striker && isBallFreeInKickoff(robots[*cur_striker]))) {
            LOG_S(INFO) << "Kickoff done: " << time_us_inPlay;
            play_state = PlayState::PLAYING;
        }
    } else if (play_state == PlayState::WAITING_FOR_OPP_KICKOFF) {
        if (time_us_inPlay >= 10_s || (cur_striker && isBallFreeInKickoff(robots[*cur_striker]))) {
            LOG_S(INFO) << "Kickoff done: " << time_us_inPlay;
            play_state = PlayState::PLAYING;
        }
    } else if (play_state == PlayState::WAITING_FOR_OPP_SETPLAY) {
        if (time_us_inSetPlayPlay >= 10_s ||
            (cur_striker && isBallFreeInSetPlay(robots[*cur_striker]))) {
            play_state = PlayState::PLAYING;
        }
    }

    if (penalized || returning_from_penalty)
        return order;

    striker_channel.publish(cur_striker ? std::make_optional(robots[*cur_striker]) : std::nullopt);

    if (play_state == PlayState::WAITING_FOR_OPP_KICKOFF ||
        play_state == PlayState::WAITING_FOR_OPP_SETPLAY) {
        if (myself.idx == 0 && (!cur_striker || *cur_striker != myself.idx))
            return KeepGoalOrder::create();
        if (fixed_ready_pos)
            return WalkToPositionOrder::create(*fixed_ready_pos,
                                               WalkToPositionOrder::Mode::SUPPORTER,
                                               myself.idx == 0 ? true : false, HeadFocus::NOTHING);
        return NoOrder::create();
    }
    if (play_state == PlayState::WAITING_FOR_OWN_KICKOFF) {
        if (cur_striker && *cur_striker == myself.idx)
            return MoveBallGoalOrder::create();
        if (myself.idx == 0)
            return KeepGoalOrder::create();
        if (fixed_ready_pos)
            return WalkToPositionOrder::create(*fixed_ready_pos,
                                               WalkToPositionOrder::Mode::SUPPORTER,
                                               myself.idx == 0 ? true : false, HeadFocus::NOTHING);
        return NoOrder::create();
    }

    simulate_positions(robots.size(), global_ball);
    distributePositions();
    bool reset_position_filter = false;
    if (!cur_role || role_by_player[myself.idx] != cur_role) {
        cur_role = role_by_player[myself.idx];
        LOG_S(INFO) << "Idx: " << (int)player_idx << " new Role: " << (int)*cur_role;
        reset_position_filter = true;
    }

    if (reset_position_filter)
        position_filter = global_ball;

    if ((cur_striker && *cur_striker == myself.idx) || rogue_striker) {
        return MoveBallGoalOrder::create();
    } else if (stupid_striker) {
        return WalkToPositionOrder::create(Position(global_ball, 0.f),
                                           WalkToPositionOrder::Mode::STRIKER);
    } else if (myself.idx == goaly_idx) {
        // TODO: We need to make sure that the goalie can still dive even if it just became the
        // striker.
        return KeepGoalOrder::create();
    } else {
        point_2d pos = manipulate_position(global_ball);

        std::vector<vectorfield::Influencer> internal_obstacles = {vectorfield::Influencer{
                .position = global_ball, .radius = 1.f, .deflection = 1.f, .flee = true}};
        if (cur_striker)
            internal_obstacles.push_back(
                    vectorfield::Influencer{.position = robots[*cur_striker].pos.point(),
                                            .radius = .75f,
                                            .deflection = 1.f,
                                            .flee = true});

        if (pos.dist(position_filter) > SoccerField::fieldLength() * 0.2f || reset_position_filter)
            position_filter = pos;
        else
            position_filter = .9f * position_filter + .1f * pos;
        return WalkToPositionOrder::create(Position(position_filter, 0.f),
                                           WalkToPositionOrder::Mode::SUPPORTER, false,
                                           std::nullopt, internal_obstacles);
    }
}

point_2d RobocupStrategy::manipulate_position(const point_2d& ball) {
    point_2d pos = position_by_role[role_by_player[myself.idx]];
    return pos;
}

bool RobocupStrategy::isBallFreeInKickoff(const TeamComData& robot) {
    if (robot.ball && robot.loc_quality >= 0.7f && robot.ball->ball_age_us < 1_s) {
        LOG_S(INFO) << "ball at: " << std::fixed << std::setprecision(2)
                    << LocalizationUtils::relToAbs(robot.ball->pos_rel, robot.pos).x << ", "
                    << LocalizationUtils::relToAbs(robot.ball->pos_rel, robot.pos).y;
        if (LocalizationUtils::relToAbs(robot.ball->pos_rel, robot.pos).dist(middle_point) >
            ball_moved_by_enemy)
            return true;
    }
    return false;
}

bool RobocupStrategy::isBallFreeInSetPlay(const TeamComData& robot) {
    if (robot.ball && robot.loc_quality >= 0.7f && robot.ball->ball_age_us < 1_s) {
        LOG_S(INFO) << "ball at: " << std::fixed << std::setprecision(2)
                    << LocalizationUtils::relToAbs(robot.ball->pos_rel, robot.pos).x << ", "
                    << LocalizationUtils::relToAbs(robot.ball->pos_rel, robot.pos).y;
        if (LocalizationUtils::relToAbs(robot.ball->pos_rel, robot.pos).dist(setPlay_ball_pos) >
            ball_moved_by_enemy)
            return true;
    }
    return false;
}

bool RobocupStrategy::isBallFreeInKickoff(const Myself& robot) {
    if (robot.ball && robot.loc.quality >= 0.7f && robot.ball->ball_age_us < 1_s) {
        LOG_S(INFO) << "ball at: " << std::fixed << std::setprecision(2)
                    << LocalizationUtils::relToAbs(robot.ball->pos_rel, robot.loc.position).x
                    << ", "
                    << LocalizationUtils::relToAbs(robot.ball->pos_rel, robot.loc.position).y;
        if (LocalizationUtils::relToAbs(robot.ball->pos_rel, robot.loc.position)
                    .dist(middle_point) > ball_moved_by_enemy)
            return true;
    }
    return false;
}

bool RobocupStrategy::isBallFreeInSetPlay(const Myself& robot) {
    if (robot.ball && robot.loc.quality >= 0.7f && robot.ball->ball_age_us < 1_s) {
        LOG_S(INFO) << "ball at: " << std::fixed << std::setprecision(2)
                    << LocalizationUtils::relToAbs(robot.ball->pos_rel, robot.loc.position).x
                    << ", "
                    << LocalizationUtils::relToAbs(robot.ball->pos_rel, robot.loc.position).y;
        if (LocalizationUtils::relToAbs(robot.ball->pos_rel, robot.loc.position)
                    .dist(setPlay_ball_pos) > ball_moved_by_enemy)
            return true;
    }
    return false;
}

void RobocupStrategy::distributePositions() {
    bool have_goalie = robots.contains(0);
    vector<Position> positions;
    vector<Positioning> roles;

    for (const auto& p : position_by_role) {
        roles.push_back(p.first);
        positions.push_back(Position{p.second, 0.f});
    }

    auto best_position_ids = permutePositions_play(robots, positions, have_goalie, cur_striker);
    size_t i = 0;
    role_by_player.clear();
    for (auto& [idx, robot] : robots) {
        role_by_player.emplace(idx, roles[best_position_ids[i]]);
        i++;
    }
}

// TODO: something is wrong with mark in def corner situation without a player to mark
point_2d RobocupStrategy::generatePosition(Positioning pb, const point_2d& ball) {
    float ball_dist;
    switch (pb) {
        case Positioning::GOALIE:
            return positioning_goalie(ball);
        case Positioning::SHADOW:
            return positioning_shadow_mirror(ball, 1);
        case Positioning::COVER:
            if (robots.size() > 5)
                return positioning_cover_duo_une(ball, 1.25f, 3.f, SoccerField::penaltySpot2Goal(),
                                                 -SoccerField::width() * .5f + .5f, -.5f);
            return positioning_cover(ball, 1.25f, SoccerField::penaltySpot2Goal(), 0.f);
        case Positioning::STRIKER:
            return ball;
        case Positioning::COVER_MID:
            ball_dist = clamped_linear_interpolation(ball.x, 1.75f / 9.f * SoccerField::length(),
                                                     2.25f / 9.f * SoccerField::length(),
                                                     -4.5f / 9.f * SoccerField::length(),
                                                     2.f / 9.f * SoccerField::length());
            return positioning_cover_mid(ball, ball_dist, SoccerField::length() * 0.5f,
                                         SoccerField::goalBoxWidth());
        case Positioning::COVER2:
            return positioning_cover_duo_deux(ball, 1.25f, SoccerField::penaltySpot2Goal(), 0.f,
                                              .5f, SoccerField::width() * .5f - .5f);
        case Positioning::SHADOW2:
            return positioning_shadow_mirror(ball, -1);
    }
    return {0.f, 0.f};
}

point_2d RobocupStrategy::positioning_goalie(const point_2d& ball) {
    Line goal_ball_line_direct(own_goal, ball);
    return goal_ball_line_direct.p1() +
           clamp(goal_ball_line_direct.p1().dist(ball) - 0.5f, 0.f, 0.5f) *
                   goal_ball_line_direct.u().normalized();
}

point_2d RobocupStrategy::positioning_shadow(const point_2d& ball) {
    Line mid_ball_line(middle_point, ball);
    return mid_ball_line.p2() - 1.5f * mid_ball_line.u().normalized();
}

point_2d RobocupStrategy::positioning_shadow_mirror(const point_2d& ball, const int side) {
    float sideY = clamp(ball.y + 1.5f / 9.f * SoccerField::length() * side,
                        -SoccerField::width() * .5f, SoccerField::width() * .5f);
    Line fixpoint_ball_line({0.f, sideY}, ball);
    float dist2ballX =
            (side < 0 && ball.x > 3.5f / 9.f * SoccerField::length())
                    ? clamped_linear_interpolation(ball.x, 1.25f / 9.f * SoccerField::length(),
                                                   2.f / 9.f * SoccerField::length(),
                                                   3.5f / 9.f * SoccerField::length(),
                                                   4.5f / 9.f * SoccerField::length())
                    : 1.25f / 9.f * SoccerField::length();
    point_2d tmp = fixpoint_ball_line.p2() - dist2ballX * fixpoint_ball_line.u().normalized();

    const float ball_goal_dist = ball.dist(point_2d{-SoccerField::length() * .5f, ball.y});
    float dist2borderY = 0.4f;
    if (side < 0 && ball.y > 1.5f)
        dist2borderY = clamped_linear_interpolation(ball.y, .4f, 1.25f, 1.5f, 3.f);
    if (side > 0 && ball.y < -1.5f)
        dist2borderY = clamped_linear_interpolation(ball.y, 1.25f, .4f, -3.f, -1.5f);
    float clampY = SoccerField::width() * .5f - dist2borderY;
    if (ball_goal_dist < 3.5f && ((side < 0.f && ball.y > side) || (side > 0.f && ball.y < side)))
        clampY = clamp(-.5f + ball_goal_dist, 0.f, clampY);
    if (ball_goal_dist < 2.5f && abs(ball.y) < 1.f) {
        clampY = clamp(1.f - abs(ball.y), 0.f, SoccerField::width() * .5f - .5f);
        if (side > 0.f) {
            return {clamp(tmp.x, -1.5f, SoccerField::length() * .5f),
                    clamp(tmp.y, clampY, SoccerField::width() * .5f + .5f)};
        } else {
            return {clamp(tmp.x, -1.5f, SoccerField::length() * .5f),
                    clamp(tmp.y, -SoccerField::width() * .5f + .5f, -clampY)};
        }
    }
    float clampX_max =
            (play_state != PlayState::PLAYING && play_state != PlayState::WAITING_FOR_OPP_SETPLAY)
                    ? -0.5f
                    : SoccerField::length() * .5f;
    return {clamp(tmp.x, -1.5f, clampX_max), clamp(tmp.y, -clampY, clampY)};
}

point_2d RobocupStrategy::positioning_cover(const point_2d& ball, const float min_ball_distance,
                                            const float max_ball_distance,
                                            const float min_goal_distance) {
    Line goal_ball_line_parallel(point_2d{-SoccerField::length() * .5f, 0.f}, ball);
    return goal_ball_line_parallel.p1() + 3.f * goal_ball_line_parallel.u().normalized();
}

point_2d RobocupStrategy::positioning_cover_mid(const point_2d& ball, const float min_ball_distance,
                                                const float max_ball_distance,
                                                const float min_goal_distance) {
    Line goal_ball_line_parallel(
            point_2d{-SoccerField::length() * .5f, clamp(ball.y, -0.75f, 0.75f)}, ball);
    point_2d tmp = goal_ball_line_parallel.p1() +
                   clamp(goal_ball_line_parallel.p1().dist(ball) - min_ball_distance,
                         min_goal_distance, max_ball_distance) *
                           goal_ball_line_parallel.u().normalized();

    return {tmp.x, clamp(tmp.y, -2.f, 2.f)};
}

point_2d RobocupStrategy::positioning_cover_duo_une(const point_2d& ball,
                                                    const float min_ball_distance,
                                                    const float max_ball_distance,
                                                    const float min_goal_distance, const float lo,
                                                    const float hi) {
    float distance = clamped_linear_interpolation(ball.x, 1.f, 2.5f, -1.0f, 2.5f);
    float mod_distance = clamped_linear_interpolation(ball.y, 1.f, distance,
                                                      -SoccerField::width() / 5 + .5f, 1.f);
    Line goal_ball_line_parallel(point_2d{-SoccerField::length() * .5f, 0.f}, ball);
    point_2d tmp =
            goal_ball_line_parallel.p1() + mod_distance * goal_ball_line_parallel.u().normalized();
    return {clamp(tmp.x, -4.f, 0.f), clamp(tmp.y, lo, hi)};
}

point_2d RobocupStrategy::positioning_cover_duo_deux(const point_2d& ball,
                                                     const float min_ball_distance,
                                                     const float max_ball_distance,
                                                     const float min_goal_distance, const float lo,
                                                     const float hi) {
    float distance = clamped_linear_interpolation(ball.x, 1.f, 2.5f, -1.0f, 2.5f);
    float mod_distance = clamped_linear_interpolation(ball.y, distance, 1.f, -1.f,
                                                      SoccerField::width() / 5 - .5f);
    Line goal_ball_line_parallel(point_2d{-SoccerField::length() * .5f, 0.f}, ball);
    point_2d tmp =
            goal_ball_line_parallel.p1() + mod_distance * goal_ball_line_parallel.u().normalized();
    return {clamp(tmp.x, -4.f, 0.f), clamp(tmp.y, lo, hi)};
}

shared_ptr<Order> RobocupStrategy::setPlay_initial() {
    striker_channel.publish(std::nullopt);
    tc_manager.resetStrikerRequest();
    cur_striker = std::nullopt;
    cur_role = std::nullopt;
    // For now, we just play normally if we get a setplay.
    play_state = gc_data.setPlay_kicking_team == KickingTeam::MyTeam
                         ? PlayState::PLAYING
                         : PlayState::WAITING_FOR_OPP_SETPLAY;

    return NoOrder::create();
}

shared_ptr<Order> RobocupStrategy::setPlay_ready() {
    std::shared_ptr<Order> order = handlePenaltyReturn();
    std::optional<PlayerIdx> prev_striker = cur_striker;
    point_2d global_ball = globalBall(time_us_lastSetPlayPlacement);
    setPlay_ball_pos = global_ball;
    point_2d striker_pos;
    std::vector<vectorfield::Influencer> internal_obstacles;
    if (gc_data.setPlay_kicking_team == KickingTeam::MyTeam) {
        if (gc_data.secondary_state == SecondaryState::ThrowIn ||
            gc_data.secondary_state == SecondaryState::GoalKick) {
            htwk::Position p0{global_ball - point_2d{0.75f, 0}, 0};
            htwk::Position p1{global_ball + point_2d{0.75f, 0}, M_PI};
            if (walkingTime(myself.loc.position, p0) < walkingTime(myself.loc.position, p1))
                striker_pos = p0.point();
            else
                striker_pos = p1.point();
        } else if (gc_data.secondary_state == SecondaryState::CornerKick) {
            htwk::Position p0{global_ball + point_2d{0, -0.75f * sgn(global_ball.y)}, 0};
            htwk::Position p1{global_ball + point_2d{-0.75f, 0}, M_PI};
            if (walkingTime(myself.loc.position, p0) < walkingTime(myself.loc.position, p1))
                striker_pos = p0.point();
            else
                striker_pos = p1.point();
        } else {
            striker_pos = global_ball - dribblingDirection(global_ball).normalized() * 0.75f;
            internal_obstacles.push_back(vectorfield::Influencer{
                    .position = global_ball, .radius = .75f, .deflection = .75f, .flee = false});
        }
    } else {
        striker_pos = global_ball - (global_ball - SoccerField::ownGoal()).normalized() * 2.f;
        internal_obstacles.push_back(vectorfield::Influencer{
                .position = global_ball, .radius = 2.f, .deflection = 2.f, .flee = true});
        if (striker_pos.y > SoccerField::width() / 2)
            striker_pos.y = SoccerField::width() / 2;
        if (striker_pos.y < -SoccerField::width() / 2)
            striker_pos.y = -SoccerField::width() / 2;
    }
    determineStriker(striker_pos, false);
    bool stupid_striker = isStupidStriker(striker_pos);

    // There is a better striker already, so remove our outdated striker request so that we don't
    // accidentally fall back to it (e.g. if the striker gets penalized).
    if (cur_striker && *cur_striker != myself.idx)
        tc_manager.resetStrikerRequest();
    if (penalized || returning_from_penalty)
        return order;

    striker_channel.publish(cur_striker ? std::make_optional(robots[*cur_striker]) : std::nullopt);

    if (global_ball.dist({0, 0}) == 0 && gc_data.secondary_state == SecondaryState::GoalKick) {
        switch (myself.idx) {
            case 0:
                return KeepGoalOrder::create();
            case 1:
                return WalkToPositionOrder::create({-2.f, SoccerField::fieldWidth() / 2 - 2.f},
                                                   WalkToPositionOrder::Mode::STRIKER);
            case 2:
                return WalkToPositionOrder::create({2.f, SoccerField::fieldWidth() / 2 - 2.f},
                                                   WalkToPositionOrder::Mode::STRIKER);
            case 3:
                return WalkToPositionOrder::create({-2.f, -SoccerField::fieldWidth() / 2 + 2.f},
                                                   WalkToPositionOrder::Mode::STRIKER);
            case 4:
                return WalkToPositionOrder::create({2.f, -SoccerField::fieldWidth() / 2 + 2.f},
                                                   WalkToPositionOrder::Mode::STRIKER);
            default:
                return WalkToPositionOrder::create({0, 0}, WalkToPositionOrder::Mode::STRIKER);
        }
    }

    simulate_positions(robots.size(), global_ball);
    for (int i = 0; i < robots.size(); i++) {
        point_2d& pos = position_by_role[(Positioning)i];
        if (pos.dist(striker_pos) < 0.75f) {
            if (striker_pos.y > 0)
                pos.y -= 1;
            else
                pos.y += 1;
        }
        if (pos.dist(striker_pos) < 0.75f) {
            if (pos.x > -SoccerField::length() * .25f)
                pos.x -= 1;
        }
        if (pos.dist(global_ball) < 1.f) {
            if (global_ball.y > 0)
                pos.y = global_ball.y - 1.f;
            else
                pos.y = global_ball.y + 1.f;
        }
    }
    distributePositions();
    bool reset_position_filter = false;
    if (!cur_role || role_by_player[myself.idx] != cur_role) {
        cur_role = role_by_player[myself.idx];
        reset_position_filter = true;
    }

    if (reset_position_filter)
        position_filter = global_ball;

    if (cur_striker && *cur_striker == myself.idx) {
        *fixed_ready_pos = {striker_pos, 0};
        return WalkToPositionOrder::create(striker_pos, WalkToPositionOrder::Mode::SUPPORTER, true,
                                           HeadFocus::BALL, internal_obstacles);
    } else if (stupid_striker) {
        return WalkToPositionOrder::create(striker_pos, WalkToPositionOrder::Mode::STRIKER);
    } else if (myself.idx == goaly_idx) {
        return KeepGoalOrder::create();
    } else {
        point_2d pos = manipulate_position(global_ball);
        if (cur_striker)
            internal_obstacles.push_back(
                    vectorfield::Influencer{.position = robots[*cur_striker].pos.point(),
                                            .radius = 1.f,
                                            .deflection = 1.f,
                                            .flee = true});

        if (pos.dist(position_filter) > SoccerField::fieldLength() * 0.2f || reset_position_filter)
            position_filter = pos;
        else
            position_filter = .9f * position_filter + .1f * pos;
        *fixed_ready_pos = {position_filter, 0.f};
        return WalkToPositionOrder::create(Position(position_filter, 0.f),
                                           WalkToPositionOrder::Mode::SUPPORTER, false,
                                           std::nullopt, internal_obstacles);
    }
}

shared_ptr<Order> RobocupStrategy::setPlay_set() {
    point_2d global_ball = globalBall();
    setPlay_ball_pos = global_ball;
    return NoOrder::create();
}

std::shared_ptr<Order> RobocupStrategy::setPlay_offensive(const point_2d global_ball) {
    return NoOrder::create();
}

std::shared_ptr<Order> RobocupStrategy::setPlay_defensive(const point_2d global_ball) {
    return NoOrder::create();
}
