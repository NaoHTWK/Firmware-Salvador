#pragma once

#include <memory>

#include "channel.h"
#include "image.h"

extern htwk::Channel<std::shared_ptr<Image>> images;
