#include "tvc/tvc.h"

#include <chrono>

#include "gtest/gtest.h"

namespace spartonautics::tvc::testing {

TEST(ThrustControllerTest, HelloWorld) {
  ThrustController controller;
  controller.Iterate(linalg::Vector3(), linalg::Vector3(),
                     std::chrono::system_clock::now());
}

}  // namespace spartonautics::tvc::testing
