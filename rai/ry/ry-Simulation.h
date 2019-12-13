#pragma once

#include <memory>

namespace rai{
  struct Simulation;
}

namespace ry {

struct RySimulation { std::shared_ptr<rai::Simulation> sim; };

};
