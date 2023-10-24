/*  ------------------------------------------------------------------
    Copyright (c) 2011-2020 Marc Toussaint
    email: toussaint@tu-berlin.de

    This code is distributed under the MIT License.
    Please see <root-path>/LICENSE for details.
    --------------------------------------------------------------  */

#include "blackbox.h"
#include <iomanip>

//===========================================================================

// void BayesOpt::query(const arr& x){
//   X.append(x);
//   y.append( f(NoArr, NoArr, x) );

//   double mu = sum(y)/double(y.N);
// //  double var = var(y);

//   if(model) delete model;
//   model = new KernelRidgeRegression(X, y, *kernel, -1., mu);
// }

//===========================================================================

extern "C" {
#include "CMA/cmaes_interface.h" //by Nikolaus Hansen
}

struct sSearchCMA {
  cmaes_t evo;
};

SearchCMA::SearchCMA() {
  self = make_unique<sSearchCMA>();
}

SearchCMA::~SearchCMA() {
}

void SearchCMA::init(uint D, int mu, int lambda, const arr& startPoint, const arr& startDev) {
  cmaes_init(&self->evo, nullptr, D, startPoint.p, startDev.p, 1, lambda, mu, nullptr);
}

void SearchCMA::init(uint D, int mu, int lambda, const arr& startPoint, double _startDev) {
  CHECK_EQ(startPoint.N, D, "");
  arr startDev(D);
  startDev=_startDev;
  cmaes_init(&self->evo, nullptr, D, startPoint.p, startDev.p, 1, lambda, mu, nullptr);
}

void SearchCMA::init(uint D, int mu, int lambda, double lo, double hi) {
  arr startPoint(D);
  rndUniform(startPoint, lo, hi, false);
  init(D, mu, lambda, startPoint, hi-lo);
}

void SearchCMA::step(arr& samples, arr& costs) {
  if(costs.N) {
    cmaes_ReestimateDistribution(&self->evo, costs.p);
  } else { //first iteration: initialize arrays:
    samples.resize(self->evo.sp.lambda, self->evo.sp.N);
    costs.resize(self->evo.sp.lambda).setZero();
  }

  //generate samples
  double* const* rgx = cmaes_SampleDistribution(&self->evo, nullptr);
  for(uint i=0; i<samples.d0; i++) samples[i].setCarray(rgx[i], samples.d1);
}

void SearchCMA::getBestSample(arr& sample) {
  sample.resize(self->evo.sp.N);
  sample.setCarray(self->evo.rgxbestever, sample.N);
}

void SearchCMA::getMean(arr& mean) {
  mean.resize(self->evo.sp.N);
  mean.setCarray(self->evo.rgxmean, mean.N);
}

