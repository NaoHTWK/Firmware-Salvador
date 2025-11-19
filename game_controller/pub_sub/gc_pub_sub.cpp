#include "gc_pub_sub.h"

htwk::Channel<GCState> gc_state;
Callback<void(const GCState& /*old_state*/, const GCState& /*new_state*/)> on_gc_state_change;
