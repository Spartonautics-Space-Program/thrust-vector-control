#include "gflags/gflags.h"
#include "glog/logging.h"
#include "tvc/tvc.h"

int main(int argc, char **argv) {
  google::InitGoogleLogging(argv[0]);
  gflags::ParseCommandLineFlags(&argc, &argv, true);
  google::InstallFailureSignalHandler();

  FLAGS_logtostderr = true;

  // TODO(milind): add phased loop
  spartonautics::tvc::ThrustController thrust_controller;
}
