#pragma once

#include "NLP.h"
#include "options.h"
#include "../Core/util.h"

/*

  Implement:
 HillClimbing (with fixed exploration; with adaptive exploration)
 DifferentialHillClimbing (with adaptive exploration distribution on delta)
 classical model-based optim

 Greedy local search (6:5) Stochastic local search (6:6) Simulated annealing (6:7)
Random restarts (6:10) Iterated local search (6:11) Variable neighborhood search
(6:13) Coordinate search (6:14) Pattern search (6:15) Nelder-Mead simplex method
(6:16) General stochastic search (6:20) Evolutionary algorithms (6:23) Covariance Matrix
Adaptation (CMAES) (6:24) Estimation of Distribution Algorithms (EDAs) (6:28)
Model-based optimization (6:31) Implicit filtering (6:34)

Improvement (5:24) Maximal Probability of Improvement
(5:24) GP-UCB (5:24)

Generic globalization: Iterated Local Optim: check when converged multiply to same local opt

Require bound constraints!

*
Twiddle
 */

namespace rai {

//===========================================================================

struct EvolutionStrategy {
  ScalarFunction f;
  shared_ptr<OptOptions> opt;
  arr x;
  double f_x=1e10;
  int evals=0, steps=0, rejectedSteps=0, tinySteps=0;

  EvolutionStrategy(ScalarFunction _f, const arr& x_init, shared_ptr<OptOptions> _opt): f(_f), opt(_opt), x(x_init) {}

  //virtuals that define a method
  virtual arr generateNewSamples() = 0;
  virtual void update(arr& samples, const arr& values) = 0;

  //generic stepping & looping
  bool step();
  shared_ptr<SolverReturn> solve();

  //helper
  arr select(const arr& samples, const arr& values, uint mu);

};

//===========================================================================

struct CMAES : EvolutionStrategy {
  unique_ptr<struct CMA_self> self;
  RAI_PARAM("CMA/", int, lambda, 20)
  RAI_PARAM("CMA/", double, sigmaInit, .1)

  CMAES(ScalarFunction f, const arr& x_init, shared_ptr<OptOptions> opt = make_shared<OptOptions>());
  ~CMAES();

  virtual arr generateNewSamples();
  virtual void update(arr& samples, const arr& values);

  arr getBestEver();
  arr getCurrentMean();
};

//===========================================================================

struct ES_mu_plus_lambda : EvolutionStrategy {
  arr mean;
  arr elite;
  RAI_PARAM("ES/", double, sigma, .1)
  RAI_PARAM("ES/", double, sigmaDecay, .001)
  RAI_PARAM("ES/", uint, lambda, 20)
  RAI_PARAM("ES/", uint, mu, 5)

  ES_mu_plus_lambda(ScalarFunction f, const arr& x_init, shared_ptr<OptOptions> opt = make_shared<OptOptions>())
      : EvolutionStrategy(f, x_init, opt) { mean = x_init; }

  virtual arr generateNewSamples();

  virtual void update(arr& X, const arr& y);
};

//===========================================================================

struct GaussEDA : EvolutionStrategy {
  arr mean;
  arr cov;
  arr elite;
  RAI_PARAM("GaussEDA/", double, sigmaInit, .1)
  RAI_PARAM("GaussEDA/", double, sigma2Min, .001)
  RAI_PARAM("GaussEDA/", double, momentum, 1.)
  RAI_PARAM("GaussEDA/", double, beta, .1)
  RAI_PARAM("ES/", double, sigmaDecay, .001)
  RAI_PARAM("ES/", uint, lambda, 20)
  RAI_PARAM("ES/", uint, mu, 5)

  GaussEDA(ScalarFunction f, const arr& x_init, shared_ptr<OptOptions> opt = make_shared<OptOptions>());

  virtual arr generateNewSamples();

  virtual void update(arr& X, const arr& y);
};

} //namespace
