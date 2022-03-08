#include "tvc/tvc.h"

#include "gflags/gflags.h"
#include "glog/logging.h"

DEFINE_bool(twice, false, "If true, print twice");

namespace spartonautics::tvc {

size_t ThrustController::HelloWorld() const {
  const size_t iterations = (FLAGS_twice ? 2ul : 1ul);
  for (size_t i = 0; i < iterations; i++) {
    LOG(INFO) << "Hello, World!";
  }
  return iterations;
}

}  // namespace spartonautics::tvc
