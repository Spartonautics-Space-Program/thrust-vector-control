#include "tvc/tvc.h"

#include "gtest/gtest.h"

namespace spartonautics::tvc::testing {

TEST(ThrustControllerTest, HelloWorld) {
  ThrustController controller;
  EXPECT_EQ(controller.HelloWorld(), 1ul);
}

}  // namespace spartonautics::tvc::testing
