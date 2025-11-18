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

std::tuple<arr, arr> select_best_mu(const arr& samples, const arr& values, uint mu);
uintA pick_best_mu(const arr& samples, const arr& values, uint mu);

//===========================================================================

struct EvolutionStrategy {
  ScalarFunction f;
  shared_ptr<OptOptions> opt;
  str method;

  arr best_x;
  double best_f=1e10;
  int evals=0, steps=0, rejectedSteps=0, tinySteps=0;

  EvolutionStrategy(str method, ScalarFunction _f, const arr& x_init, shared_ptr<OptOptions> _opt): f(_f), opt(_opt), method(method), best_x(x_init) {}

  //virtuals that define a method
  virtual arr generateSamples() = 0;
  virtual void update(arr& samples, arr& values) = 0;

  //generic stepping & looping
  bool step();
  shared_ptr<SolverReturn> solve();
  arr evaluateSamples(const arr& X);
  void update_best(const arr& X, const arr& F);
};

//===========================================================================

struct ES_mu_plus_lambda : EvolutionStrategy {
  arr mean;
  arr elite_X, elite_y;
  RAI_PARAM("ES/", double, sigma, .1)
  RAI_PARAM("ES/", double, sigmaDecay, .001)
  RAI_PARAM("ES/", uint, lambda, 20)
  RAI_PARAM("ES/", uint, mu, 5)

  ES_mu_plus_lambda(ScalarFunction f, const arr& x_init, shared_ptr<OptOptions> opt = make_shared<OptOptions>())
      : EvolutionStrategy("es_mu_lam", f, x_init, opt) { mean = x_init; }

  virtual arr generateSamples();

  virtual void update(arr& X, arr& y);
};

//===========================================================================

struct GaussEDA : EvolutionStrategy {
  arr mean;
  arr cov;
  arr elite_X, elite_y;
  RAI_PARAM("GaussEDA/", double, sigmaInit, .1)
  RAI_PARAM("GaussEDA/", double, momentum, .5)
  RAI_PARAM("GaussEDA/", double, beta, .1)
  RAI_PARAM("ES/", double, sigmaDecay, .001)
  RAI_PARAM("ES/", uint, lambda, 20)
  RAI_PARAM("ES/", uint, mu, 5)

  GaussEDA(ScalarFunction f, const arr& x_init, shared_ptr<OptOptions> opt = make_shared<OptOptions>());

  virtual arr generateSamples();

  virtual void update(arr& X, arr& y);
};

} //namespace
