#include <memory>
#include <optional>
#include <vector>

#include "bounding_box.h"
#include "image.h"
#include "point_3d.h"

namespace htwk {

class ObstacleDetectorLegacy {
public:
    ObstacleDetectorLegacy();
    ~ObstacleDetectorLegacy();
    void proceed(const std::shared_ptr<Image>& img, std::vector<float>& plane,
                 std::vector<BoundingBox> ball_boxes);
};

}  // namespace htwk
