#include "tvc/tvc.h"
#include "util/util.h"

#define PICO

int main(int argc, char **argv) {
  spartonautics::util::InitGoogle(&argc, &argv);

  // TODO(milind): add phased loop
  spartonautics::tvc::ThrustController thrust_controller;
}
