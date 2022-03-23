#ifndef UTIL_UTIL_H_
#define UTIL_UTIL_H_

#ifndef PICO
#include "gflags/gflags.h"
#include "glog/logging.h"
#include "gtest/gtest.h"
#endif  // PICO

namespace spartonautics::util {

// TODO(milind): replace this with file out or something
#ifdef PICO
#define LOG(level) std::cout;

// TODO(milind): add gflags replacements
#endif  // PICO

// Initialize glog and gflags
void InitGoogle(int *argc, char ***argv);

#ifndef PICO
class TestEnvironment : public ::testing::Environment {
 public:
  void SetUp() override {
    int argc = 1;
    std::string argv = " ";
    char *argv_ptr = argv.data();
    char **argv_ptr_2 = &argv_ptr;
    InitGoogle(&argc, &argv_ptr_2);
  }
};
#endif  // PICO

}  // namespace spartonautics::util

#endif  // UTIL_UTIL_H_
