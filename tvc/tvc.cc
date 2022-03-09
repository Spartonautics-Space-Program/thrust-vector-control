#include "tvc/tvc.h"

#include "gflags/gflags.h"
#include "glog/logging.h"

namespace spartonautics::tvc {

ThrustController::ThrustController()
    : ThrustController(linalg::Vector<kNumStates>()) {}

ThrustController::ThrustController(linalg::Vector<kNumStates> x_hat)
    : x_hat_(x_hat), P_(), now_(chrono::system_clock::time_point::min()) {}

// TODO(milind): fill out code
linalg::Vector<ThrustController::kNumOutputs> ThrustController::Iterate(
    linalg::Vector3 accel, linalg::Vector3 gyro,
    chrono::system_clock::time_point now) {
  const double dt = std::chrono::duration<double>(now - now_).count();
  now_ = now;

  Predict(dt);
  Correct(accel, gyro, dt);

  return ComputeThrust();
}

void ThrustController::Predict(double dt) {
  const linalg::Matrix<kNumStates, kNumStates> F =
      linalg::Matrix<kNumStates, kNumStates>::Data{{
          // For position x, y, z, use x = x_0 + v_0t + 0.5at^2
          {1.0, 0.0, 0.0, dt, 0.0, 0.0, 0.5 * std::pow(dt, 2), 0.0, 0.0, 0.0,
           0.0, 0.0, 0.0, 0.0, 0.0},
          {0.0, 1.0, 0.0, 0.0, dt, 0.0, 0.0, 0.5 * std::pow(dt, 2), 0.0, 0.0,
           0.0, 0.0, 0.0, 0.0, 0.0},
          {0.0, 0.0, 1.0, 0.0, 0.0, dt, 0.0, 0.0, 0.5 * std::pow(dt, 2), 0.0,
           0.0, 0.0, 0.0, 0.0, 0.0},
          // For velocity x, y, z, use v = v_0 + at
          {0.0, 0.0, 0.0, 1.0, 0.0, 0.0, dt, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
           0.0},
          {0.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, dt, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
           0.0},
          {0.0, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, dt, 0.0, 0.0, 0.0, 0.0, 0.0,
           0.0},
          // Keep acceleration x, y, z constant until we get measurements
          {0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
           0.0},
          {0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
           0.0},
          {0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 0.0, 0.0,
           0.0},
          // For angles roll, pitch, yaw, use theta = theta_0 + omega * t
          {0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, dt, 0.0,
           0.0},
          {0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, dt,
           0.0},
          {0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0,
           dt},
          // Keep angular velocity roll, pitch, yaw constants until we get
          // measurements
          {0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0,
           0.0},
          {0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0,
           0.0},
          {0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
           1.0},
      }};
  x_hat_ = F * x_hat_;

  // TODO(milind): predict uncertainty
}

// TODO(milind): write code
void ThrustController::Correct(linalg::Vector3 accel, linalg::Vector3 gyro,
                               double dt) {
  linalg::Vector<kNumInputs> z = linalg::Vector<kNumInputs>::Data(
      {accel.x(), accel.y(), accel.z(), gyro.x(), gyro.y(), gyro.z()});
  (void)z;
}

// TODO(milind): write code
linalg::Vector<ThrustController::kNumOutputs> ThrustController::ComputeThrust()
    const {
  return linalg::Vector<kNumOutputs>();
}

}  // namespace spartonautics::tvc
