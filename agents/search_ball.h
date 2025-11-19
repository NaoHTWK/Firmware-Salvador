#include <cstdint>
#include <optional>

#include "agent_base.h"
#include "localization_pub_sub.h"
#include "multi_target_tracker_pub_sub.h"
#include "rel_ball.h"
#include "robot_time.h"
#include "stl_ext.h"
#include "team_strategy_pub_sub.h"

class BallSearchAgent : public AgentBase {
public:
    BallSearchAgent() : AgentBase("BallSearch") {}
    MotionCommand proceed(std::shared_ptr<Order> order) override;

    enum class Side { LEFT, RIGHT };

private:
    void updateBallDirectionMemory(const std::optional<RelBall>& ball) {
        if (ball) {
            if (ball->last_seen_time != ballLastSeen) {
                lastBallDist = ball->pos_rel.norm();
                ballLastSide = ball->pos_rel.y > 0 ? Side::LEFT : Side::RIGHT;
                ballLastSeen = ball->last_seen_time;
            }
            // TODO: Add ball hypothesis tracking
            // hypoLastSeen = ownRobot.last_ball_hypothesis.value_or(RelativeHypo{}).lastSeenTime;
        }
    }

    htwk::ChannelSubscriber<std::optional<RelBall>> ball_sub = rel_ball_channel.create_subscriber();
    htwk::ChannelSubscriber<std::optional<TeamComData>> striker_sub =
            striker_channel.create_subscriber();
    htwk::ChannelSubscriber<LocPosition> pos_sub = loc_position_channel.create_subscriber();
    Side ballLastSide = Side::LEFT;
    int64_t ballLastSeen = 0;
    int64_t strikerBallLastSeen = 0;
    int64_t hypoLastSeen = 0;

    float lastBallDist = std::numeric_limits<float>::infinity();
    const int64_t standTime = 0.5_s;
    const int64_t turnTime = 3.0_s;
    const float maxBallDist = 2.f;
};
