/*  ------------------------------------------------------------------
    Copyright (c) 2011-2024 Marc Toussaint
    email: toussaint@tu-berlin.de

    This code is distributed under the MIT License.
    Please see <root-path>/LICENSE for details.
    --------------------------------------------------------------  */

#pragma once

#include "m_Newton.h"
#include "m_Gradient.h"

namespace rai {

struct RestartNewton {
  ScalarFunction f;
  shared_ptr<rai::OptOptions> opt;
  arr bounds;

  struct LocalMinimum { arr x; double fx; uint hits; };
  rai::Array<LocalMinimum> localMinima;
  LocalMinimum* best=0;

  RestartNewton(ScalarFunction f, const arr& bounds, std::shared_ptr<OptOptions> opt=make_shared<OptOptions>());
  ~RestartNewton();

  void step();
  void run(uint maxIt=10);
  void report();

  void reOptimizeLocalMinima();
private:
  void addRunFrom(arr& x);
};

} //namespace
