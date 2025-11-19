#include "localize.h"

MotionCommand LocalizeAgent::proceed(std::shared_ptr<Order> /* order */) {
    std::optional<LocPosition> loc_position = loc_position_sub.latestIfExists();
    float locQual = loc_position ? loc_position->quality : 0.0f;

    if (locQual < minQual || bestQual < minQualHysteresis) {
        int64_t lastLocAge = time_us() - lastLocTime;
        int64_t lastLocTimeCycle = lastLocAge % (standTime + turnTime);
        int cycle_number = lastLocAge / (standTime + turnTime);

        // Just try to find the position by looking around.
        MotionCommand mc = MotionCommand::Stand(HeadFocus::LOC);

        if (cycle_number != 0 && cycle_number % 4 == 0 && lastLocTimeCycle < 3_s) {
            mc = MotionCommand::Walk({.dx = 0.35, .dy = 0, .da = 0}, HeadFocus::LOC);
        }

        // If we stood there and haven't seen anything rotate slowly around ourself.
        if (lastLocTimeCycle > standTime) {
            mc = MotionCommand::Walk({.dx = 0, .dy = 0, .da = 1.f}, HeadFocus::LOC);
        }

        bestQual = locQual;
        return mc;
    }

    lastLocTime = time_us();
    return MotionCommand::Nothing;
}
