#pragma once

#include <memory>

#include "channel.h"
#include "position.h"

struct LocPosition {
    htwk::Position position{0.0f, 0.0f, 0.0f};
    float quality = 0.0f;
};

extern htwk::Channel<LocPosition> loc_position_channel;
