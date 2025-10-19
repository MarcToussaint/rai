/*  ------------------------------------------------------------------
    Copyright (c) 2011-2024 Marc Toussaint
    email: toussaint@tu-berlin.de

    This code is distributed under the MIT License.
    Please see <root-path>/LICENSE for details.
    --------------------------------------------------------------  */

#pragma once

#include "options.h"
#include "NLP.h"
#include "../Core/util.h"

//===========================================================================

namespace rai {

struct SlackGaussNewton {
  OptOptions opt;
  std::shared_ptr<NLP> nlp;

  SlackGaussNewton(const shared_ptr<NLP>& _nlp, const arr& x_init={});
  SlackGaussNewton& setOptions(const OptOptions& _opt) { opt = _opt; return *this; }
  std::shared_ptr<SolverReturn> solve();

private:
  arr x;
  uint evals=0;
  uint iters=0;
  double step();

  struct Eval {
    arr x;
    arr phi, J;
    arr err;
    arr s, Js;
    void eval(const arr& _x, SlackGaussNewton& walker);
  } ev;
};

//===========================================================================

}
