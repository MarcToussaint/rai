/*  ------------------------------------------------------------------
    Copyright (c) 2011-2024 Marc Toussaint
    email: toussaint@tu-berlin.de

    This code is distributed under the MIT License.
    Please see <root-path>/LICENSE for details.
    --------------------------------------------------------------  */

#pragma once

#include "../Optim/NLP.h"
#include "../Core/util.h"

//===========================================================================

namespace rai {

struct SlackGaussNewton_Options {
  RAI_PARAM("sam/", double, tolerance, .01)
  RAI_PARAM("sam/", double, margin, .0)
  RAI_PARAM("sam/", int, verbose, 2)

  RAI_PARAM("sam/", int, maxEvals, 50)
  RAI_PARAM("sam/", double, maxStep, .1)
  RAI_PARAM("sam/", double, damping, 1e-2)
};

struct SlackGaussNewton {
  SlackGaussNewton_Options opt;
  std::shared_ptr<NLP> nlp;

  SlackGaussNewton(const shared_ptr<NLP>& _nlp) : nlp(_nlp) {}
  SlackGaussNewton& setOptions(const SlackGaussNewton_Options& _opt) { opt = _opt; return *this; }
  std::shared_ptr<SolverReturn> solve();

private:
  arr x;
  uint evals=0;
  uint iters=0;
  void step();

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
