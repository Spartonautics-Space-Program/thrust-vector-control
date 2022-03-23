#include "util/util.h"

namespace spartonautics::util {

void InitGoogle(int *argc, char ***argv) {
#ifndef PICO
  google::InitGoogleLogging((*argv)[0]);
  gflags::ParseCommandLineFlags(argc, argv, true);
  google::InstallFailureSignalHandler();

  FLAGS_logtostderr = true;
#endif  // PICO
}

}  // namespace spartonautics::util
