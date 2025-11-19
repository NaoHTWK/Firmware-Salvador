#pragma once

#include <vector>

#include "point_2d.h"
#include "position.h"

namespace htwk::vectorfield {

/// @brief The Influencer struct contains the arguments to shape the vector field according to the
/// datafields.
struct Influencer {
    point_2d position;  //!< The center of the influencer field.
    float radius;  //!< The influencer only modifies vectors that are bounded by this radius around
                   //!< the position.
    float deflection;   //!< Strength of the influence. If the deflection is negative and flee is
                        //!< true, we get closer to the center of the influencer field.
    bool flee = false;  //!< Specify if we want to increase the distance to the obstacle
};

/// @brief Calculates the direction the robot should walk towards to avoid running into obstacles;
///     All coordinates must be of the same kind (either relative or absolute).
/// @param[in] source The robots position.
/// @param[in] target The position where the robot is supposed to go.
/// @param[in] obstacles A list of positions of the obstacles we know of.
/// @param[in] radius_of_influence The radius in which the obstacles have an influence
/// @param[in] deflection Some kind of measure of influence. The deflection strength is calculated
/// by deviding deflection by the distance to the obstacle.
/// @return The direction the robot should walk towards next to avoid obstacles.
///
///  The current shape of the influencer field if the source is at infinit distance to the left and
///  the target is at infinit distance to the right. The Influencer position is marked with `i` and
///  the area of effect is in front of the influencer position marked with either `/` or `\`,
///  depending on the deflection direction. `/` is a deflection to the north, while `\` is a
///  deflection to the south. Note that the actual field looks a bit different (check out the
///  VectorFieldDebugger!). Indeed, the influencer, if `flee` is not set, causes the object entering
///  the area of effect to keep it's distance from the influencer position. Because the influencer
///  usually describes an object in the path of the current position (`source`) and the target, the
///  field is cut as soon as the `source` object passes the influencer object. After passing the
///  center of the influencer, the direct path will be resumed.
///
///  - - - - - - - - - - - - - - -
///  - - - - - - - - - - - - - - -
///  - - - - - - - - - - - - - - -
///  - - - - - - - - - - - - - - -
///  - - - - - - - - - / / - - - -
///  - - - - - - - / / / / - - - -
///  - - - - - - / / / / / - - - -
///  - - - - - - / / / / / i - - -
///  - - - - - - \ \ \ \ \ - - - -
///  - - - - - - - \ \ \ \ - - - -
///  - - - - - - - - - \ \ - - - -
///  - - - - - - - - - - - - - - -
///  - - - - - - - - - - - - - - -
///  - - - - - - - - - - - - - - -
///  - - - - - - - - - - - - - - -
///  - - - - - - - - - - - - - - -
point_2d calcDirection(const Position& source, const Position& target,
                       const std::vector<Influencer>& influencer);

}  // namespace htwk::vectorfield
