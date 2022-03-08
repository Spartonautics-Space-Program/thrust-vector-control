#ifndef TVC_TVC_H_
#define TVC_TVC_H_

#include <chrono>
#include <cstddef>

#include "linalg/matrix.h"

namespace {
namespace chrono = std::chrono;
}

namespace spartonautics::tvc {

class ThrustController {
 public:
  // Kalman filter for estimating state and computing optimal thrust vector.
  // Inputs: 6 dimensions, acceleration and angular velocity x, y, z.
  // Also takes in current time, but not as an input.
  // State: 6 dimensions, position and velocity x, y, z.
  // Outputs: 4 dimensions, thrust roll, pitch, yaw, and magnitude.
  linalg::Vector4 Iterate(linalg::Vector3 accel, linalg::Vector3 gyro,
                          chrono::system_clock::time_point now);

 private:
  static constexpr linalg::Matrix6 kQ = linalg::Matrix6();
  static constexpr linalg::Matrix6 kR = linalg::Matrix6();
  static constexpr linalg::Matrix<double, 6, 4> kH =
      linalg::Matrix<double, 6, 4>();

  linalg::Vector6 x_hat_;
  linalg::Matrix6 P_;
  chrono::system_clock::time_point last_now_ =
      chrono::system_clock::time_point::min();
};

}  // namespace spartonautics::tvc

#endif  // TVC_TVC_H_
