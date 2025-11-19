#include "vision_pub_sub.h"

htwk::Channel<std::optional<htwk::ObjectHypothesis>> ball_hypothesis_channel;

htwk::Channel<std::vector<htwk::ObjectHypothesis>> point_features_channel;

htwk::Channel<std::vector<htwk::ObjectHypothesis>> opponent_hypotheses_channel;

htwk::Channel<std::vector<htwk::LineGroup>> lines_channel;

htwk::Channel<std::shared_ptr<std::vector<point_2d>>> near_obstacles_channel;
htwk::Channel<std::shared_ptr<std::vector<point_2d>>> no_near_obstacles_channel;
htwk::Channel<std::shared_ptr<std::vector<point_2d>>> ball_obstacles_channel;

Callback<void(point_2d cam_ball, YawPitch head_angles, int64_t timestamp_us, int cx, int cy, int fx,
              int fy)>
        cam_ball_callback;
