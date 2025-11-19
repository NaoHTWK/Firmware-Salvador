#include "image.h"
#include "localization_pub_sub.h"
#include "multi_target_tracker.h"
#include "object_hypothesis.h"
#include "tracked_object.h"

class BallTracker {
public:
    BallTracker();
    ~BallTracker();

    void proceed(const Image& img);

private:
#ifdef ROBOT_MODEL_T1
    static constexpr float ball_radius = 0.11f;
#else
    static constexpr float ball_radius = 0.075f;
#endif

    htwk::ChannelSubscriber<std::optional<htwk::ObjectHypothesis>> ball_sub;
    htwk::ChannelSubscriber<LocPosition> loc_position_sub;

    static void publishBallCallback(const std::vector<htwk::TrackedObject>& tracked_objects);

    htwk::MTtracker ball_tracker{1, 0.5f, 0.2f, publishBallCallback};

    bool isValidBall(const Image& img, point_2d ball, point_2d ball_pos);
};