#include "tvc/tvc.h"

#include "gflags/gflags.h"
#include "glog/logging.h"

namespace spartonautics::tvc {

ThrustController::ThrustController()
    : ThrustController(linalg::Vector<kNumStates>(),
                       linalg::Matrix<kNumStates, kNumStates>()) {}

ThrustController::ThrustController(linalg::Vector<kNumStates> x_hat,
                                   linalg::Matrix<kNumStates, kNumStates> P)
    : x_hat_(x_hat), P_(P), now_(chrono::system_clock::time_point::min()) {}

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

  // Process noise uncertainty
  // Assume that the noise of measurements on separate axes isn't correlated,
  // and states are only correlated with their derivitives and antiderivitives.
  // Noise for position
  const double p_noise = std::pow(dt, 4.0) / 4.0;
  // Noise for 1st derivitive
  const double p_dot_noise = std::pow(dt, 3.0) / 2.0;
  // Noise for 2nd derivitive
  const double p_dot_dot_noise = std::pow(dt, 2.0) / 2.0;

  // Noise for velocity
  const double v_noise = std::pow(dt, 2.0);
  // Noise for 1st derivitive
  const double v_dot_noise = dt;

  // Noise for acceleration
  const double a_noise = 1.0;

  const auto Q =
      std::pow(kAccelerationStdDev, 2) *
      linalg::Matrix<kNumStates, kNumStates>(
          linalg::Matrix<kNumStates, kNumStates>::Data{
              {// Position noise
               {p_noise, 0.0, 0.0, p_dot_noise, 0.0, 0.0, p_dot_dot_noise, 0.0,
                0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0},
               {0.0, p_noise, 0.0, 0.0, p_dot_noise, 0.0, 0.0, p_dot_dot_noise,
                0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0},
               {0.0, 0.0, p_noise, 0.0, 0.0, p_dot_noise, 0.0, 0.0,
                p_dot_dot_noise, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0},
               // Velocity noise
               {p_dot_noise, 0.0, 0.0, v_noise, 0.0, 0.0, 0.0, 0.0, v_dot_noise,
                0.0, 0.0, 0.0, 0.0, 0.0, 0.0},
               {0.0, p_dot_noise, 0.0, 0.0, v_noise, 0.0, 0.0, v_dot_noise, 0.0,
                0.0, 0.0, 0.0, 0.0, 0.0, 0.0},
               {0.0, 0.0, p_dot_noise, 0.0, 0.0, v_noise, 0.0, 0.0, v_dot_noise,
                0.0, 0.0, 0.0, 0.0, 0.0, 0.0},
               // Acceleration noise
               {p_dot_dot_noise, 0.0, 0.0, v_dot_noise, 0.0, 0.0, a_noise, 0.0,
                0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0},
               {0.0, p_dot_dot_noise, 0.0, 0.0, v_dot_noise, 0.0, 0.0, a_noise,
                0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0},
               {0.0, 0.0, p_dot_dot_noise, 0.0, 0.0, v_dot_noise, 0.0, 0.0,
                a_noise, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0},
               // Angle noise
               {0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, p_noise, 0.0, 0.0,
                p_dot_noise, 0.0, 0.0},
               {0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, p_noise, 0.0,
                0.0, p_dot_noise, 0.0},
               {0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, p_noise,
                0.0, 0.0, p_dot_noise},
               // Angular velocity noise
               {0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, p_dot_noise, 0.0,
                0.0, v_noise, 0.0, 0.0},
               {0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, p_dot_noise,
                0.0, 0.0, v_noise, 0.0},
               {0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
                p_dot_noise, 0.0, 0.0, v_noise}}});

  // Update estimate uncertainty
  P_ = (F * P_ * F.Transpose()) + Q;
}

// TODO(milind): write code
void ThrustController::Correct(linalg::Vector3 pos, linalg::Vector3 omega,
                               double dt) {
  linalg::Vector<kNumInputs> z = linalg::Vector<kNumInputs>::Data(
      {pos.x(), pos.y(), pos.z(), omega.x(), omega.y(), omega.z()});
  (void)z;
}

// TODO(milind): write code
linalg::Vector<ThrustController::kNumOutputs> ThrustController::ComputeThrust()
    const {
  return linalg::Vector<kNumOutputs>();
}

}  // namespace spartonautics::tvc
