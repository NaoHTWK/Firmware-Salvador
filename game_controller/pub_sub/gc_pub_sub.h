#pragma once

#include "callback.h"
#include "channel.h"
#include "gc_state.h"

extern htwk::Channel<GCState> gc_state;
extern Callback<void(const GCState& /*old_state*/, const GCState& /*new_state*/)>
        on_gc_state_change;
