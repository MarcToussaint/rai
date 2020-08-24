/*  ------------------------------------------------------------------
    Copyright (c) 2011-2020 Marc Toussaint
    email: toussaint@tu-berlin.de

    This code is distributed under the MIT License.
    Please see <root-path>/LICENSE for details.
    --------------------------------------------------------------  */

#pragma once

#include "../Core/thread.h"
#include <memory>

namespace rai {
struct Simulation;
struct Configuration;
}

namespace ry {

struct RySimulation {
  std::shared_ptr<rai::Simulation> sim;
  std::shared_ptr<Var<rai::Configuration>> config;
};

};
