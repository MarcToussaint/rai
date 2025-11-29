/*  ------------------------------------------------------------------
    Copyright (c) 2011-2024 Marc Toussaint
    email: toussaint@tu-berlin.de

    This code is distributed under the MIT License.
    Please see <root-path>/LICENSE for details.
    --------------------------------------------------------------  */

#pragma once

#include "GenericBO.h"
#include "m_RestartNewton.h"
#include "../Core/array.h"

struct KernelRidgeRegression;
struct DefaultKernelFunction;

namespace rai {

struct BayesOpt : GenericBO {
  arr data_X, data_y;

  shared_ptr<KernelRidgeRegression> gp_model;
  shared_ptr<DefaultKernelFunction> kernel;
  bool thompsonMethod=false;
  RestartNewton alphaMinima;
  arr thompsonSampleX, thompsonSampleF;
  struct LocalMinimum { arr x; double fx; } thompsonMinima;

  bool leastSquaresCase=false;
  double lengthScale;
  double lambda = 1e-6;

  //lengthScale is always relative to hi-lo
  BayesOpt(shared_ptr<NLP> P, shared_ptr<OptOptions> opt, double init_lengthScale=1., double prior_var=1.);

  virtual arr generateSamples();
  virtual void update(arr& X, arr& Phi, arr& F);

  void run(uint maxIt=10);
  void report();

private:
  void addDataPoint(const arr& x, const arr& y); //and update the regressions
  void updateAlphaMinima();
  void updateThompsonMinima();
};

} //namespace
