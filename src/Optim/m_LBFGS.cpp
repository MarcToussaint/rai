#include "m_LBFGS.h"

extern "C" {
#include "liblbfgs/liblbfgs.h"
}

namespace rai {

double proc_evaluate(
  void *instance,
  const double *_x,
  double *_g,
  const int n,
  const double step
  ){
  LBFGS* This = (LBFGS*) instance;
  CHECK_EQ(_x, This->x.p, "");

  double f_x = This->f(This->g, NoArr, This->x);
  CHECK_EQ(This->g.N, This->x.N, "");
  for(uint i=0;i<This->x.N;i++) _g[i] = This->g.p[i];
  return f_x;
}

int proc_progress(
    void *instance,
    const double *x,
    const double *g,
    const double fx,
    const double xnorm,
    const double gnorm,
    const double step,
    int n,
    int k,
    int ls
    ){
  LBFGS* This = (LBFGS*) instance;
  if(This->opt->verbose>1) cout <<"--lbfgs-- it:" <<k <<" ls: " <<ls <<" f:" <<fx <<" |g|:" <<gnorm <<" |delta|:" <<step <<endl;
  return 0;
}


LBFGS::LBFGS(ScalarFunction _f, const arr& x_init, std::shared_ptr<OptOptions> _opt)
    : f(_f), opt(_opt), x(x_init) {
}

std::shared_ptr<SolverReturn> LBFGS::solve(){
  double f_x;
  int retval = lbfgs(x.N, x.p, &f_x, proc_evaluate, proc_progress, this, 0);
  if(retval){
    LOG(-2) <<"LBFGS returned error " <<retval;
  }
  std::shared_ptr<SolverReturn> ret = make_shared<SolverReturn>();
  ret->x = x;
  // uint evals=0;
  // double time=0.;
  ret->feasible=true;
  ret->f = f_x;
  ret->done=true;
  return ret;
}

} //namespace
