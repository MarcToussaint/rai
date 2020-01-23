#pragma once

#include "../Core/thread.h"
#include <memory>

namespace rai{
  struct Simulation;
  struct Configuration;
}

namespace ry {

struct RySimulation {
  std::shared_ptr<rai::Simulation> sim;
  std::shared_ptr<Var<rai::Configuration>> config;
};

};
