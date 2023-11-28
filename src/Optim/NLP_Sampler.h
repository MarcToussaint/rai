#include <Optim/NLP.h>

//===========================================================================

struct NLP_Walker{

  NLP& nlp;
  //evaluation data
  arr x;
  arr phi, J;
  arr g, Jg;
  arr h, Jh;
  arr s, Js;
  arr r, Jr;
  arr gpos;
  double beta_mean, beta_sdv;
  double err;

  //h-threshold
  double eps = .05;

  //steps
  double alpha = .1;

  uint samples=0;
  uint evals=0;

  NLP_Walker(NLP& _nlp) : nlp(_nlp) {}

  void initialize(const arr& _x){ x=_x; phi.clear(); }



  bool step(double maxStep = 1e6);
  bool step_delta();

protected:
  void clipBeta(const arr& d, const arr& xbar, double& beta_lo, double& beta_up);
  void get_rnd_direction(arr& dir, arr& delta);
  void eval(const arr& _x, bool update_phi);


};

//===========================================================================

struct AlphaSchedule {
  enum Mode { _constBeta, _cosine, _linear, _sqrtLinear };
  arr alpha_bar;

  AlphaSchedule(Mode mode, uint T, double beta=-1.);
};

//===========================================================================

arr sample_direct(NLP& nlp, uint K=1000, int verbose=1);
arr sample_restarts(NLP& nlp, uint K=1000, int verbose=1);
arr sample_denoise(NLP& nlp, uint K=1000, int verbose=1);
