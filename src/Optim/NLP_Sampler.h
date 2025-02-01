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
  RAI_PARAM("sam/", double, tolerance, .01)
  RAI_PARAM("sam/", double, margin, .0)
  RAI_PARAM("sam/", bool, useCentering, true)

  //--new

  RAI_PARAM("sam/", int, verbose, 1)

  RAI_PARAM("sam/", rai::String, seedMethod, "uni")
  RAI_PARAM("sam/", uint, seedCandidates, 10)

  RAI_PARAM("sam/", double, penaltyMu, 1.)

  //-- downhill

  RAI_PARAM("sam/", rai::String, downhillMethod, "GN")

  RAI_PARAM("sam/", int, downhillMaxSteps, 50)
  RAI_PARAM("sam/", double, slackStepAlpha, 1.)
  RAI_PARAM("sam/", double, slackMaxStep, .1)
  RAI_PARAM("sam/", double, slackRegLambda, 1e-2)

  RAI_PARAM("sam/", double, ineqOverstep, -1)

  RAI_PARAM("sam/", rai::String, downhillNoiseMethod, "none")
  RAI_PARAM("sam/", rai::String, downhillRejectMethod, "none")
  RAI_PARAM("sam/", double, downhillNoiseSigma, .1)

  //-- interior

  RAI_PARAM("sam/", rai::String, interiorMethod, "HR")

  RAI_PARAM("sam/", int, interiorBurnInSteps, 0)
  RAI_PARAM("sam/", int, interiorSampleSteps, 1)
  RAI_PARAM("sam/", rai::String, interiorNoiseMethod, "iso")
  RAI_PARAM("sam/", double, hitRunEqMargin, .1)

  RAI_PARAM("sam/", double, interiorNoiseSigma, .5)

  RAI_PARAM("sam/", double, langevinTauPrime, -1.)

  NLP_Sampler_Options();

};

struct NLP_Sampler {
  NLP_Sampler_Options opt;
  std::shared_ptr<NLP> nlp;

  NLP_Sampler(const shared_ptr<NLP>& _nlp) : nlp(_nlp) {}

  NLP_Sampler& setOptions(const NLP_Sampler_Options& _opt) { opt = _opt; return *this; }
  void run(arr& data, uintA& evals);
  std::shared_ptr<SolverReturn> sample();

private:
  //evaluation data
  arr x;
  struct Eval {
    arr x;
    arr phi, J;
    arr g, Jg; //ineq
    arr h, Jh; //eq
    arr s, Js; //slack ([ineq]+ and eq)
    arr r, Jr; //sos
    arr gpos;
    double err;
    void eval(const arr& _x, NLP_Sampler& walker);
    void convert_eq_to_ineq(double margin);
  } ev;

  //counters
  uint evals=0;
  Eval ev_stored;

  void initialize(const arr& _x) { x=_x; ev.phi.clear(); ev.x.clear(); }
  void ensure_eval() { ev.eval(x, *this); }
  void store_eval() { ensure_eval(); ev_stored = ev; }

  bool step_GaussNewton(bool slackStep, double penaltyMu=1., double alpha=-1., double stepMax=-1., double lambda=1e-2);
  void step_PlainGrad(bool slackMode, double penaltyMu=1., double alpha=-1., double stepMax=-1.);
  bool step_hit_and_run();
  bool step_noise(double sig);
  bool step_noise_covariant(double sig, double penaltyMu=1., double lambda=1e0);
  void bound_clip();
  void step_Langevin(bool slackMode, double tauPrime, double penaltyMu);

  bool reject_MH(double gamma, double mu, const arr& asymmetric_del={}, double sigma=-1.);

  void init_novelty(const arr& data, uint D);
  void init_distance(const arr& data, uint D);

  //--
  arr compute_slackStep(){
    ensure_eval();
    arr Hinv = lapack_inverseSymPosDef(((2.*opt.penaltyMu)*~ev.Js)*ev.Js+opt.slackRegLambda*eye(x.N));
    arr delta = (-2.*opt.penaltyMu) * Hinv * (~ev.Js) * ev.s;
    return delta;
  }

public:
  bool run_downhill();
  void run_interior(arr& data, uintA& dataEvals);

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

  LineSampler(double stepMax=-1.) { if(stepMax>0.) init(stepMax); }

  void init(double stepMax) { beta_lo=-stepMax; beta_up=stepMax; }

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
