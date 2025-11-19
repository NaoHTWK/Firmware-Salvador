#include <walk_relative.h>

#include "walkrelativeorder.h"

using namespace htwk;
using namespace std;

MotionCommand WalkRelativeAgent::proceed(std::shared_ptr<Order> order) {
    if (!isOrder<WalkRelativeOrder>(order))
        return MotionCommand::Nothing;

    auto* wtp_order = dynamic_cast<WalkRelativeOrder*>(order.get());

    htwk::Position dest_rel = wtp_order->pos;
    // TODO: Avoid near obstacles.

    return MotionCommand::Walk({.dx = dest_rel.x, .dy = dest_rel.y, .da = dest_rel.a},
                               HeadFocus::LOC);
}
