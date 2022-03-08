#include "tvc/tvc.h"

#include "gflags/gflags.h"
#include "glog/logging.h"

namespace spartonautics::tvc {

// TODO(milind): fill out code
linalg::Vector<ThrustController::kNumOutputs> ThrustController::Iterate(
    linalg::Vector3 accel, linalg::Vector3 gyro,
    chrono::system_clock::time_point now) {
  linalg::Vector<kNumInputs> z = linalg::Vector<kNumInputs>::Data(
      {accel.x(), accel.y(), accel.z(), gyro.x(), gyro.y(), gyro.z()});
  (void)z;

  return linalg::Vector<kNumOutputs>();
}

}  // namespace spartonautics::tvc
