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

  EvolutionStrategy(ScalarFunction _f): f(_f) {}

  virtual arr generateNewSamples() = 0;
  virtual void update(const arr& samples, const arr& values) = 0;

  bool step();

  shared_ptr<SolverReturn> solve();

  arr select(const arr& samples, const arr& y, uint mu);

};

//===========================================================================

struct CMAES : EvolutionStrategy {
  unique_ptr<struct CMA_self> self;
  RAI_PARAM("CMA/", int, lambda, 20)
  RAI_PARAM("CMA/", double, sigmaInit, .1)

  CMAES(ScalarFunction f, const arr& x_init={}, shared_ptr<OptOptions> _opt = make_shared<OptOptions>());
  ~CMAES();

  virtual arr generateNewSamples();
  virtual void update(const arr& samples, const arr& values);

  arr getBestEver();
  arr getCurrentMean();
};

//===========================================================================

struct ES_mu_plus_lambda : EvolutionStrategy {
  arr mean;
  RAI_PARAM("ES/", double, sigma, .1)
  RAI_PARAM("ES/", uint, lambda, 20)
  RAI_PARAM("ES/", uint, mu, 5)

  ES_mu_plus_lambda(ScalarFunction f, const arr& x_init={}) : EvolutionStrategy(f) {}

  virtual arr generateNewSamples(){
    arr X = replicate(mean, lambda);
//    for(uint i=0;i<X.d0;i++) X[i] = X[i] % (0.5 + 1.*rand(X.d1));
    rndGauss(X, sigma, true);
    return X;
  }

  virtual void update(const arr& X, const arr& y){
    arr Y = select(X, y, mu);
    mean = ::mean(Y);
  }
};

} //namespace
