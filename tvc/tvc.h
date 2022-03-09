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
  static constexpr size_t kNumStates = 15;
  static constexpr size_t kNumInputs = 6;
  static constexpr size_t kNumOutputs = 4;

  ThrustController();
  ThrustController(linalg::Vector<kNumStates> x_hat);

  // Kalman filter for estimating state and computing optimal thrust vector.
  // Inputs: acceleration and angular veclocity x, y, z
  // State: position, velocity, acceleration, angle, angular velocity x, y, z
  // Outputs: optimal thrust euler angles and magnitude
  linalg::Vector<kNumOutputs> Iterate(linalg::Vector3 accel,
                                      linalg::Vector3 gyro,
                                      chrono::system_clock::time_point now);

  // Predicts what the state is before using the measurement
  void Predict(double dt);
  // Corrects the prediction using the measurements
  void Correct(linalg::Vector3 accel, linalg::Vector3 gyro, double dt);
  // After predicting and correcting the state, computes the thrust vector
  linalg::Vector<kNumOutputs> ComputeThrust() const;

  inline linalg::Vector<kNumStates> x_hat() const { return x_hat_; }

 private:
  // Process noise uncertainty
  static constexpr linalg::Matrix<kNumStates, kNumStates> kQ =
      linalg::Matrix<kNumStates, kNumStates>();
  // Measurement uncertainty
  static constexpr linalg::Matrix<kNumInputs, kNumInputs> kR =
      linalg::Matrix<kNumInputs, kNumInputs>();
  // Observation matrix
  static constexpr linalg::Matrix<kNumInputs, kNumStates> kH =
      linalg::Matrix<kNumInputs, kNumStates>();

  // Estimated state
  linalg::Vector<kNumStates> x_hat_;
  // Estimate uncertainty
  linalg::Matrix<kNumStates, kNumStates> P_;
  chrono::system_clock::time_point now_;
};

}  // namespace spartonautics::tvc

#endif  // TVC_TVC_H_
