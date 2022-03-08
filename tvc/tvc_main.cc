#include "gflags/gflags.h"
#include "glog/logging.h"
#include "tvc/tvc.h"

int main(int argc, char **argv) {
  google::InitGoogleLogging(argv[0]);
  gflags::ParseCommandLineFlags(&argc, &argv, true);
  google::InstallFailureSignalHandler();

  FLAGS_logtostderr = true;

  spartonautics::tvc::ThrustController thrust_controller;
  thrust_controller.HelloWorld();
}
