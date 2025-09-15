#include "lbfgs.h"

extern "C" {
#include "liblbfgs.h"
}


typedef std::function<double(arr& g, arr& H, const arr& x)> ScalarFunction;

double proc_evaluate(
  void *instance,
  const double *_x,
  double *_g,
  const int n,
  const double step
  ){
  OptLBFGS* This = (OptLBFGS*) instance;

  CHECK_EQ(_x, This->x.p, "");

  double f_x = This->f(This->g, NoArr, This->x);
  for(uint i=0;i<This->dimension;i++) _g[i] = This->g.p[i];
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
  OptLBFGS* This = (OptLBFGS*) instance;
  cout <<"==lbfgs== it:" <<k <<" ls: " <<ls <<" f:" <<fx <<" |g|:" <<gnorm <<" |delta|:" <<step <<endl;
  return 0;
}


OptLBFGS::OptLBFGS(arr& x, const ScalarFunction& f, rai::OptOptions o)
    : f(f), dimension(x.N), x(x), opt(o){
}

std::shared_ptr<SolverReturn> OptLBFGS::solve(){
  double f_x;
  int retval = lbfgs(dimension, x.p, &f_x, proc_evaluate, proc_progress, this, 0);
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
