#pragma once

#include <optional>
#include <sstream>

#include "channel.h"
#include "gc_pub_sub.h"
#include "localization_pub_sub.h"
#include "multi_target_tracker_pub_sub.h"
#include "point_2d.h"
#include "position.h"
#include "rel_ball.h"
#include "robot_time.h"
#include "sensor_pub_sub.h"

// TODO: They should be somewhere else
using PlayerIdx = uint8_t;
constexpr int kMaxNumPlayers = 6;

// Team strategy and team com need a consistent "snapshot" of the robot's state, so that
// information doesn't change while processing through the strategy and the information sent out
// to other robots is consistent with the information used to determine the local strategy.
// The local version is Myself, while the reduced network version is TeamComData.
struct Myself {
    PlayerIdx idx;
    bool penalized;
    htwk::FallDownState fallen;
    int64_t last_update_us;
    LocPosition loc;
    std::optional<RelBall> ball;
    std::vector<RobotDetection> robots;
};

class MyselfProvider {
public:
    MyselfProvider(PlayerIdx idx)
        : myself(idx),
          gc_state_sub(gc_state.create_subscriber()),
          loc_sub(loc_position_channel.create_subscriber()),
          rel_ball_sub(rel_ball_channel.create_subscriber()),
          fallen_sub(htwk::fallen_channel.create_subscriber()),
          rel_robots_sub(rel_robots_channel.create_subscriber()) {
        updateMyself();
    }

    Myself getMyself(bool update = false) {
        if (update) {
            updateMyself();
        }
        return myself;
    }

private:
    void updateMyself() {
        auto gc_state = gc_state_sub.latestIfExists();
        myself.penalized = gc_state ? gc_state->my_team.players[myself.idx].is_penalized : true;
        myself.loc = loc_sub.latest();
        myself.ball = rel_ball_sub.latest();
        myself.last_update_us = time_us();
        myself.fallen = fallen_sub.latest();
        myself.robots = rel_robots_sub.latest();
    }

    Myself myself;
    htwk::ChannelSubscriber<GCState> gc_state_sub;
    htwk::ChannelSubscriber<LocPosition> loc_sub;
    htwk::ChannelSubscriber<std::optional<RelBall>> rel_ball_sub;
    htwk::ChannelSubscriber<htwk::FallDownState> fallen_sub;
    htwk::ChannelSubscriber<std::vector<RobotDetection>> rel_robots_sub;
};

struct TeamComBall {
    point_2d pos_rel;
    int64_t ball_age_us = 0;

    template <class Archive>
    void serialize(Archive& ar, const unsigned int version) {
        ar & pos_rel;
        ar & ball_age_us;
    }
};

struct TeamComData {
    int64_t striker_request_time_us = 0;
    PlayerIdx player_idx;
    bool is_fallen = false;
    htwk::Position pos;
    float loc_quality = 0;
    std::optional<TeamComBall> ball;
    int64_t sent_time_us = 0;
    std::vector<RobotDetection> robots;

    template <class Archive>
    void serialize(Archive& ar, const unsigned int version) {
        ar & striker_request_time_us;
        ar & player_idx;
        ar & is_fallen;
        ar & pos;
        ar & loc_quality;
        ar & ball;
        ar & sent_time_us;
        ar & robots;
    }
    std::string to_string() const {
        std::string ball_str = "none";
        if (ball) {
            ball_str = std::to_string(ball->pos_rel.x) + ", " + std::to_string(ball->pos_rel.y);
        }
        std::stringstream ss;
        ss << "TeamComData(player_idx=" << int(player_idx) << ", is_fallen=" << is_fallen
           << ", pos=" << pos.x << ", " << pos.y << ", " << pos.a << ", loc_quality=" << loc_quality
           << ", ball="
           << (ball ? "none"
                    : (std::to_string(ball->pos_rel.x) + ", " + std::to_string(ball->pos_rel.y) +
                       ", ball_age_us=" + std::to_string(ball->ball_age_us)))
           << ", sent_time_us=" << sent_time_us
           << ", striker_request_time_us=" << striker_request_time_us << ")";
        return ss.str();
    }
};
