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

struct GlobalIterativeNewton {
  arr x;
  OptNewton newton;
  OptGrad grad;
  arr bounds;

  struct LocalMinimum { arr x; double fx; uint hits; };
  rai::Array<LocalMinimum> localMinima;
  LocalMinimum* best;

  GlobalIterativeNewton(ScalarFunction f, const arr& bounds, std::shared_ptr<OptOptions> opt=make_shared<OptOptions>());
  ~GlobalIterativeNewton();

  void step();
  void run(uint maxIt=10);
  void report();

  void reOptimizeAllPoints();
};

} //namespace
