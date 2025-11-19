#include <cstdint>

#include <agent_base.h>
#include <stl_ext.h>
#include <localization_pub_sub.h>
#include <robot_time.h>

class LocalizeAgent : public AgentBase {
public:
    LocalizeAgent() : AgentBase("Localize") {}
    MotionCommand proceed(std::shared_ptr<Order> order) override;

private:


    htwk::ChannelSubscriber<LocPosition> loc_position_sub =
            loc_position_channel.create_subscriber();


    float bestQual = 0;
    const float minQual = 0.30f;
    const float minQualHysteresis = 0.70f;

    int64_t lastLocTime = time_us();
    const int64_t standTime = 2._s;
    const int64_t turnTime = 2._s;
};

