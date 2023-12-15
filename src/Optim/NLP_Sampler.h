/*  ------------------------------------------------------------------
    Copyright (c) 2011-2020 Marc Toussaint
    email: toussaint@tu-berlin.de

    This code is distributed under the MIT License.
    Please see <root-path>/LICENSE for details.
    --------------------------------------------------------------  */

#pragma once

#include <Optim/NLP.h>

//===========================================================================

struct NLP_Walker{
  NLP& nlp;

  //evaluation data
  arr x;
  struct Eval{
    arr x;
    arr phi, J;
    arr g, Jg;
    arr h, Jh, Ph;
    arr s, Js;
    arr r, Jr;
    arr gpos;
    double err;
    void eval(const arr& _x, NLP_Walker& walker);
  } ev;

  //h-threshold
  double eps = .05;

  //slack step rate
  double alpha = .1;
  double maxStep = .5;

  double temperature;

  //counters
  uint samples=0;
  uint evals=0;

  NLP_Walker(NLP& _nlp, double _temperature=0.) : nlp(_nlp), temperature(_temperature) {}

  void initialize(const arr& _x){ x=_x; ev.phi.clear(); ev.x.clear(); }

  bool step();
  bool step_delta();
  bool step_slack();
  bool step_hit_and_run(double maxStep);

protected:
  void clipBeta(const arr& d, const arr& xbar, double& beta_lo, double& beta_up);
  arr get_delta();
  arr get_rnd_direction();
  void get_beta_mean(double& beta_mean, double& beta_sdv, const arr& dir, const arr& xbar);
};

//===========================================================================

struct LineSampler{
  arr b, s;
  double beta_lo=-1e6, beta_up=1e6;
  double p_beta;

  LineSampler(double maxStep) : beta_lo(-maxStep), beta_up(maxStep){}

  double eval_beta(double beta);

  void add_constraints(const arr& gbar, const arr& gd, double temperature);
  void add_constraints_eq(const arr& hbar, const arr& hd, double temperature);

  void clip_beta(const arr& gbar, const arr& gd);

  double sample_beta();
  double sample_beta_uniform();

  void plot();
};

//===========================================================================

struct AlphaSchedule {
  enum Mode { _constBeta, _cosine, _linear, _sqrtLinear };
  arr alpha_bar;

  AlphaSchedule(Mode mode, uint T, double beta=-1.);
};

//===========================================================================

arr sample_direct(NLP& nlp, uint K=1000, int verbose=1, double temperature=0.);
arr sample_restarts(NLP& nlp, uint K=1000, int verbose=1, double temperature=0.);
arr sample_greedy(NLP& nlp, uint K=1000, int verbose=1, double temperature=0.);
arr sample_denoise(NLP& nlp, uint K=1000, int verbose=1);
