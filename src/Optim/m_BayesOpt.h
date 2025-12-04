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
  RestartNewton alphaMinima;
  arr thompsonSampleX, thompsonSampleF;
  struct LocalMinimum { arr x; double fx; } thompsonMinima;

  RAI_PARAM("GPOpt/", double, lengthScale, 1.)
  RAI_PARAM("GPOpt/", double, sigmaObsSqr, 1e-6) //same as lambda regularization in KernelRidgeRegression
  RAI_PARAM("GPOpt/", double, priorVar, 1.)
  RAI_PARAM("GPOpt/", bool, leastSquaresCase, false)
  RAI_PARAM("GPOpt/", bool, thompsonMethod, false)

  //lengthScale is always relative to hi-lo
  BayesOpt(shared_ptr<NLP> P, shared_ptr<OptOptions> opt=make_shared<OptOptions>());

  virtual arr generateSamples();
  virtual void update(arr& X, arr& Phi, arr& F);

  void run(uint maxIt=10);
  void report();

private:
  void ensure_kernel();
  void addDataPoint(const arr& x, const arr& y); //and update the regressions
  void updateAlphaMinima();
  void updateThompsonMinima();
};

} //namespace
