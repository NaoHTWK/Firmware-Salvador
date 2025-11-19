#include <cstdint>
#include "imu_joint_state.h"

namespace htwk {

enum FallDownStateType : uint32_t {
  READY,
  FALLEN,
  GETTING_UP
};

enum FallDownStateResult : uint32_t {
  SUCCEEDED,
  FAILED
};

enum FallDownSide : uint32_t {
  FRONT,
  BACK
};

struct FallDownState {
  FallDownStateType type = READY;
  FallDownStateResult result = SUCCEEDED;
  FallDownSide side = FRONT;

  void to_failed(FallDownSide side) {
    type = FALLEN;
    result = FAILED;
    this->side = side;
  }

  void to_succeeded() {
    type = READY;
    result = SUCCEEDED;
  }
};

}  // namespace htwk