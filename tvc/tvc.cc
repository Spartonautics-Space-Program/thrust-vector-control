#include "tvc/tvc.h"

#include "gflags/gflags.h"
#include "glog/logging.h"

namespace spartonautics::tvc {

// TODO(milind): fill out code
linalg::Vector4 ThrustController::Iterate(
    linalg::Vector3 accel, linalg::Vector3 gyro,
    chrono::system_clock::time_point now) {
  linalg::Vector6 u = linalg::Vector6::Data(
      {accel.x(), accel.y(), accel.z(), gyro.x(), gyro.y(), gyro.z()});
  (void)u;

  return linalg::Vector4();
}

}  // namespace spartonautics::tvc
