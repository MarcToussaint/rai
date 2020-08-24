/*  ------------------------------------------------------------------
    Copyright (c) 2011-2020 Marc Toussaint
    email: toussaint@tu-berlin.de

    This code is distributed under the MIT License.
    Please see <root-path>/LICENSE for details.
    --------------------------------------------------------------  */

#pragma once

#include "optimization.h"
#include "../Algo/MLcourse.h"

/*

  Implement:
 HillClimbing (with fixed exploration; with adaptive exploration)
 DifferentialHillClimbing (with adaptive exploration distribution on delta)
 classical model-based optim

 Greedy local search (6:5) Stochastic local search (6:6) Simulated annealing (6:7)
Random restarts (6:10) Iterated local search (6:11) Variable neighborhood search
(6:13) Coordinate search (6:14) Pattern search (6:15) Nelder-Mead simplex method
(6:16) General stochastic search (6:20) Evolutionary algorithms (6:23) Covariance Matrix
Adaptation (CMA) (6:24) Estimation of Distribution Algorithms (EDAs) (6:28)
Model-based optimization (6:31) Implicit filtering (6:34)

Improvement (5:24) Maximal Probability of Improvement
(5:24) GP-UCB (5:24)

Generic globalization: Iterated Local Optim: check when converged multiply to same local opt

Require bound constraints!

*
Twiddle
 */

struct BayesOpt {
  ScalarFunction f;
  KernelFunction* kernel;
  KernelRidgeRegression* model;
  arr X, y;

  BayesOpt(arr& x, const ScalarFunction& f, OptOptions o=NOOPT);
  ~BayesOpt();

  void step();
  void run(uint maxIt = 1000);

 private:
  void query(const arr& x);
};

//===========================================================================

/// Wrapper for CMA stochastic optimization
struct SearchCMA {
  unique_ptr<struct sSearchCMA> self;

  SearchCMA();
  ~SearchCMA();

  /// D=problem dimension, lambda=#samples taken in each iteration, mu \approx lambda/3 specifies the selection size, lo and hi specify an initialization range
  void init(uint D, int mu=-1, int lambda=-1, double lo=-1., double hi=1.);

  /// instead of lo and hi, explicitly give a start point and standard deviation around this point
  void init(uint D, int mu, int lambda, const arr& startPoint, double _startDev);

  void init(uint D, int mu, int lambda, const arr& startPoint, const arr& startDev);

  /// first call: generate initial random samples
  /// further calls: costs needs to contain the cost function values for all elements in samples; returns a new set of samples
  void step(arr& samples, arr& costs);

  void getBestSample(arr& sample);
  void getMean(arr& mean);
};
