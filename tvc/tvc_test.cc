#include "tvc/tvc.h"

#include <chrono>

#include "gtest/gtest.h"

namespace spartonautics::tvc::testing {

double ComputePosition(linalg::Vector3 x_0, linalg::Vector3 v_0,
                       linalg::Vector3 a, size_t axis, double t) {
  return x_0(axis) + (v_0(axis) * t) + (0.5 * a(axis) * std::pow(t, 2));
}

double ComputeVelocity(linalg::Vector3 v_0, linalg::Vector3 a, size_t axis,
                       double t) {
  return v_0(axis) + (a(axis) * t);
}

double ComputeAngle(linalg::Vector3 theta_0, linalg::Vector3 omega, size_t axis,
                    double t) {
  return theta_0(axis) + (omega(axis) * t);
}

// Test the ability to predict the state of the system without measurements
TEST(ThrustControllerTest, Predict) {
  constexpr linalg::Vector3 kP = linalg::Vector3::Data({1.0, 2.0, 3.0});
  constexpr linalg::Vector3 kV = linalg::Vector3::Data({4.0, 5.0, 6.0});
  constexpr linalg::Vector3 kA = linalg::Vector3::Data({7.0, 8.0, 9.0});

  constexpr linalg::Vector3 kTheta = linalg::Vector3::Data({10.0, 11.0, 12.0});
  constexpr linalg::Vector3 kOmega = linalg::Vector3::Data({13.0, 14.0, 15.0});

  auto controller =
      ThrustController(linalg::Vector<ThrustController::kNumStates>::Data(
          {kP.x(), kP.y(), kP.z(), kV.x(), kV.y(), kV.z(), kA.x(), kA.y(),
           kA.z(), kTheta.x(), kTheta.y(), kTheta.z(), kOmega.x(), kOmega.y(),
           kOmega.z()}));
  constexpr double kT = 0.5;
  controller.Predict(kT);
  EXPECT_EQ(
      controller.x_hat(),
      linalg::Vector<ThrustController::kNumStates>(
          {ComputePosition(kP, kV, kA, 0, kT),
           ComputePosition(kP, kV, kA, 1, kT),
           ComputePosition(kP, kV, kA, 2, kT), ComputeVelocity(kV, kA, 0, kT),
           ComputeVelocity(kV, kA, 1, kT), ComputeVelocity(kV, kA, 2, kT),
           kA.x(), kA.y(), kA.z(), ComputeAngle(kTheta, kOmega, 0, kT),
           ComputeAngle(kTheta, kOmega, 1, kT),
           ComputeAngle(kTheta, kOmega, 2, kT), kOmega.x(), kOmega.y(),
           kOmega.z()}));
}

}  // namespace spartonautics::tvc::testing
