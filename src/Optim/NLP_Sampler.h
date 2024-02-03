/*  ------------------------------------------------------------------
    Copyright (c) 2011-2020 Marc Toussaint
    email: toussaint@tu-berlin.de

    This code is distributed under the MIT License.
    Please see <root-path>/LICENSE for details.
    --------------------------------------------------------------  */

#pragma once

#include "../Optim/NLP.h"
#include "../Core/util.h"

//===========================================================================

struct NLP_Sampler_Options {
  RAI_PARAM("sam/", double, eps, .05)
  RAI_PARAM("sam/", double, alpha, 1.)
  RAI_PARAM("sam/", double, maxStep, .5)
  RAI_PARAM("sam/", double, eqMargin, .1)
  RAI_PARAM("sam/", bool, useCentering, true)
};

struct NLP_Walker{
  NLP_Sampler_Options opt;

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
    void convert_eq_to_ineq(double margin);
  } ev;

  //h-threshold

  double a, sig;

  //counters
  uint samples=0;
  uint evals=0;

  NLP_Walker(NLP& _nlp, double alpha_bar=1.) : nlp(_nlp) {
    set_alpha_bar(alpha_bar);
  }

  void set_alpha_bar(double alpha_bar);
  void initialize(const arr& _x){ x=_x; ev.phi.clear(); ev.x.clear(); }
  void ensure_eval(){ ev.eval(x, *this); }

  bool step();
  bool step_slack();
  bool step_hit_and_run(double maxStep);
  bool step_hit_and_run_eq(bool includeEqualities=true);
  bool step_noise(double sig=-1.);
  bool step_noise_covariance(double sig=-1.);
  bool step_bound_clip();

protected:
  void clipBeta(const arr& d, const arr& xbar, double& beta_lo, double& beta_up);
  arr get_rnd_direction();
  void get_beta_mean(double& beta_mean, double& beta_sdv, const arr& dir, const arr& xbar);
};

//===========================================================================

struct LineSampler{
  arr b, s;
  double num_constraints=0.;
  double beta_lo=-1e6, beta_up=1e6;
  double p_beta;

  LineSampler(double maxStep=-1.) { if(maxStep>0.) init(maxStep); }

  void init(double maxStep){ beta_lo=-maxStep; beta_up=maxStep; }

  double eval_beta(double beta);

  void add_constraints(const arr& gbar, const arr& gd);
  void add_constraints_eq(const arr& hbar, const arr& hd, double margin);

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

arr sample_NLPwalking(NLP& nlp, uint K=1000, int verbose=1, double alpha_bar=0.);
arr sample_restarts(NLP& nlp, uint K=1000, int verbose=1, double alpha_bar=0.);
arr sample_greedy(NLP& nlp, uint K=1000, int verbose=1, double alpha_bar=0.);
arr sample_denoise(NLP& nlp, uint K=1000, int verbose=1);
arr sample_denoise_direct(NLP& nlp, uint K=1000, int verbose=1);
