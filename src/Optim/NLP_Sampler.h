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

struct NLP_Sampler_Options {
  RAI_PARAM("sam/", double, eps, .05)
  RAI_PARAM("sam/", bool, useCentering, true)

  //--new

  RAI_PARAM("sam/", int, verbose, 1)

  RAI_PARAM("sam/", int, initNovelty, 10)

  RAI_PARAM("sam/", int, downhillMaxSteps, 50)

  RAI_PARAM("sam/", double, penaltyMu, 1.)

  RAI_PARAM("sam/", double, slackStepAlpha, 1.)
  RAI_PARAM("sam/", double, slackMaxStep, .1)
  RAI_PARAM("sam/", double, slackRegLambda, 1e-2)

  RAI_PARAM("sam/", int, noiseSteps, 10)
  RAI_PARAM("sam/", double, noiseSigma, .1)
  RAI_PARAM("sam/", bool, noiseCovariant, true)

  RAI_PARAM("sam/", int, interiorSteps, -1)
  RAI_PARAM("sam/", double, hitRunEqMargin, .1)

  RAI_PARAM("sam/", bool, acceptBetter, false)
  RAI_PARAM("sam/", bool, acceptMetropolis, false)

  RAI_PARAM("sam/", double, lagevinTauPrime, -1.)

  NLP_Sampler_Options();

};

struct NLP_Walker {
  NLP_Sampler_Options opt;

  NLP& nlp;

  //evaluation data
  arr x;
  struct Eval {
    arr x;
    arr phi, J;
    arr g, Jg;
    arr h, Jh; //, Ph;
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
  Eval stored;

  NLP_Walker(NLP& _nlp, double alpha_bar=1.) : nlp(_nlp) {
    set_alpha_bar(alpha_bar);
  }

  void set_alpha_bar(double alpha_bar);
  void initialize(const arr& _x) { x=_x; ev.phi.clear(); ev.x.clear(); }
  void ensure_eval() { ev.eval(x, *this); }
  void store_eval() { ensure_eval(); stored = ev; }

  bool step(); //old

  bool step_slack(double penaltyMu=1., double alpha=-1., double maxStep=-1., double lambda=1e-2);
  bool step_hit_and_run_old(double maxStep);
  bool step_hit_and_run();
  bool step_noise(double sig);
  bool step_noise_covariant(double sig, double penaltyMu=1., double lambda=1e0);
  bool step_bound_clip();

  void run(arr& data, arr& trace=NoArr);

  void init_novelty(const arr& data, uint D);

  //--
  arr compute_slackStep(){
    ensure_eval();
    arr Hinv = lapack_inverseSymPosDef(((2.*opt.penaltyMu)*~ev.Js)*ev.Js+opt.slackRegLambda*eye(x.N));
    arr delta = (-2.*opt.penaltyMu) * Hinv * (~ev.Js) * ev.s;
    return delta;
  }
 protected:
  void clipBeta(const arr& d, const arr& xbar, double& beta_lo, double& beta_up);
  arr get_rnd_direction();
  void get_beta_mean(double& beta_mean, double& beta_sdv, const arr& dir, const arr& xbar);
};

//===========================================================================

struct LineSampler {
  arr b, s;
  double num_constraints=0.;
  double beta_lo=-1e6, beta_up=1e6;
  double p_beta;

  LineSampler(double maxStep=-1.) { if(maxStep>0.) init(maxStep); }

  void init(double maxStep) { beta_lo=-maxStep; beta_up=maxStep; }

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
