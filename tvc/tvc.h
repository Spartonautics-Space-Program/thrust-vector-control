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
  static constexpr size_t kNumStates = 9;
  static constexpr size_t kNumInputs = 6;
  static constexpr size_t kNumOutputs = 4;

  // Kalman filter for estimating state and computing optimal thrust vector.
  // Inputs: acceleration and angular velocity x, y, z.
  // Also takes in current time, but not as an input.
  // State: position and velocity x, y, z, rotation roll, pitch,
  // yaw.
  // Outputs: thrust roll, pitch, yaw, and magnitude.
  linalg::Vector<kNumOutputs> Iterate(linalg::Vector3 accel,
                                      linalg::Vector3 gyro,
                                      chrono::system_clock::time_point now);

 private:
  static constexpr linalg::Matrix<kNumStates, kNumStates> kQ =
      linalg::Matrix<kNumStates, kNumStates>();
  static constexpr linalg::Matrix<kNumInputs, kNumInputs> kR =
      linalg::Matrix<kNumInputs, kNumInputs>();
  static constexpr linalg::Matrix<kNumInputs, kNumStates> kH =
      linalg::Matrix<kNumInputs, kNumStates>();

  linalg::Vector<kNumStates> x_hat_;
  linalg::Matrix<kNumStates, kNumStates> P_;
  chrono::system_clock::time_point last_now_ =
      chrono::system_clock::time_point::min();
};

}  // namespace spartonautics::tvc

#endif  // TVC_TVC_H_
