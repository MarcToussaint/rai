/*  ------------------------------------------------------------------
    Copyright (c) 2011-2024 Marc Toussaint
    email: toussaint@tu-berlin.de

    This code is distributed under the MIT License.
    Please see <root-path>/LICENSE for details.
    --------------------------------------------------------------  */

#include "i_NLopt.h"
#include "NLP_Solver.h"
#include "utils.h"

#ifdef RAI_NLOPT

#include <nlopt.hpp>

enum NLopt_SolverOption { _NLopt_LD_SLSQP,
			  _NLopt_LD_MMA,
			  _NLopt_LN_COBYLA,
			  _NLopt_LD_AUGLAG,
			  _NLopt_LD_AUGLAG_EQ,
			  _NLopt_LN_NELDERMEAD,
			  _NLopt_LD_LBFGS,
			  _NLopt_LD_TNEWTON,
			  _NLopt_LD_TNEWTON_RESTART,
			  _NLopt_LD_TNEWTON_PRECOND,
			  _NLopt_LD_TNEWTON_PRECOND_RESTART,
			  };

template<> const char* Enum<NLopt_SolverOption>::names []= {
    "LD_SLSQP",
    "LD_MMA",
    "LN_COBYLA",
    "LD_AUGLAG",
    "LD_AUGLAG_EQ",
    "LN_NELDERMEAD",
    "LD_LBFGS",
    "LD_TNEWTON",
    "LD_TNEWTON_RESTART",
    "LD_TNEWTON_PRECOND",
    "LD_TNEWTON_PRECOND_RESTART", nullptr
};

struct FuncCallData {
  NLoptInterface* I=0;
  uint feature=0;
};

nlopt::algorithm getSolverEnum(const char* param, bool& needsSubsolver, int verbose) {
  rai::Enum<NLopt_SolverOption> sol;
  sol = rai::getParameter<rai::String>(param);
  if(verbose) {
    cout <<"NLopt option: " <<param <<": " <<sol <<endl;
  }
  switch(sol) {
#define CASE(x, s) case _NLopt_##x:{ needsSubsolver=s; return nlopt::x; }
      CASE(LD_SLSQP, true);
      CASE(LD_MMA, false);
      CASE(LN_COBYLA, false);
      CASE(LD_AUGLAG, true);
      CASE(LD_AUGLAG_EQ, true);
      CASE(LN_NELDERMEAD, false);
      CASE(LD_LBFGS, false);
      CASE(LD_TNEWTON, false);
      CASE(LD_TNEWTON_RESTART, false);
      CASE(LD_TNEWTON_PRECOND, false);
      CASE(LD_TNEWTON_PRECOND_RESTART, false);
#undef CASE
    default: HALT("shouldn't be here");
  }
  return nlopt::NUM_ALGORITHMS;
}

arr NLoptInterface::solve(const arr& x_init) {
  //-- get initialization
  arr x;
  if(!!x_init) {
    x = x_init;
  } else {
    x = P->getInitializationSample();
  }

  //-- get and check bounds, clip x
  arr bounds_lo=bounds[0], bounds_up=bounds[1];
  CHECK_EQ(x.N, bounds_up.N, "NLOpt requires bounds");
  CHECK_EQ(x.N, bounds_lo.N, "NLOpt requires bounds");
  for(uint i=0; i<bounds_lo.N; i++) CHECK(bounds_lo.elem(i)<bounds_up.elem(i), "NLOpt requires bounds");
  P->boundClip(x);

  //-- create NLopt solver
  bool needsSubsolver;
  nlopt::opt opt(getSolverEnum("NLopt_solver", needsSubsolver, verbose), x.N);
  opt.set_xtol_abs(rai::getParameter<double>("NLopt_xtol", 1e-4));
  //  opt.set_ftol_abs(1e-3);
  if(needsSubsolver) {
    nlopt::opt subopt(getSolverEnum("NLopt_subSolver", needsSubsolver, verbose), x.N);
    subopt.set_xtol_abs(rai::getParameter<double>("NLopt_xtol", 1e-4));
    opt.set_local_optimizer(subopt);
  }

  //-- set bounds
  if(bounds_lo.N==x.N) opt.set_lower_bounds(bounds_lo.vec());
  if(bounds_up.N==x.N) opt.set_upper_bounds(bounds_up.vec());

  //-- set cost and add constraints
  opt.set_min_objective(_f, this);
  rai::Array<FuncCallData> funcCallData(P->featureTypes.N);
  for(uint i=0; i<P->featureTypes.N; i++) {
    FuncCallData& d = funcCallData.elem(i);
    d.I = this;
    d.feature = i;
    if(P->featureTypes.elem(i) == OT_ineq) opt.add_inequality_constraint(_g, &d);
    if(P->featureTypes.elem(i) == OT_eq) opt.add_equality_constraint(_h, &d);
  }

  //-- optimize
  double fval;
  try {
    std::vector<double> x_vec = x.vec();
    //nlopt::result R =
    opt.solve(x_vec, fval);
    x = x_vec;
  } catch(const std::runtime_error& err) {
    cout <<"NLOPT terminated with " <<err.what() <<endl;
  }
  cout <<"NLOPT final costs:" <<opt.last_optimum_value() <<endl;
  return x;
}

double NLoptInterface::f(const arr& _x, arr& _grad) {
  if(x!=_x) { x=_x;  P->evaluate(phi_x, J_x, x); }
  CHECK(!isSparse(J_x), "NLopt can't handle sparse Jacobians")
  CHECK_EQ(phi_x.N, P->featureTypes.N, "");
  double fval=0;
  arr grad=zeros(_grad.N);
  for(uint i=0; i<phi_x.N; i++) {
    if(P->featureTypes.elem(i)==OT_f) { fval += phi_x.elem(i);  if(grad.N) grad += J_x[i]; }
    if(P->featureTypes.elem(i)==OT_sos) { double y = phi_x.elem(i);  fval += y*y;  if(grad.N) grad += (2.*y) * J_x[i]; }
  }
//  for(uint i=0; i<grad.N; i++) _grad[i] = grad.elem(i);
  _grad = grad;
//  cout <<fval <<endl;
  return fval;
}

double NLoptInterface::g(const arr& _x, arr& _grad, uint feature) {
  if(x!=_x) { x=_x;  P->evaluate(phi_x, J_x, x); }
  CHECK_EQ(P->featureTypes(feature), OT_ineq, "");
  if(_grad.N) _grad = J_x[feature];
  return phi_x.elem(feature);

}

double NLoptInterface::h(const arr& _x, arr& _grad, uint feature) {
  if(x!=_x) { x=_x;  P->evaluate(phi_x, J_x, x); }
  CHECK_EQ(P->featureTypes(feature), OT_eq, "");
  if(_grad.N) _grad = J_x[feature];
  return phi_x.elem(feature);
}

double NLoptInterface::_f(const std::vector<double>& _x, std::vector<double>& _grad, void* f_data) {
  arr x(_x, true);
  arr grad(_grad, true);
  return ((NLoptInterface*)f_data) -> f(x, grad);
}

double NLoptInterface::_g(const std::vector<double>& _x, std::vector<double>& _grad, void* f_data) {
  arr x(_x, true);
  arr grad(_grad, true);
  FuncCallData* d = (FuncCallData*)f_data;
  return d->I->g(x, grad, d->feature);
}

double NLoptInterface::_h(const std::vector<double>& _x, std::vector<double>& _grad, void* f_data) {
  arr x(_x, true);
  arr grad(_grad, true);
  FuncCallData* d = (FuncCallData*)f_data;
  return d->I->h(x, grad, d->feature);
}

#else

arr NLoptInterface::solve(const arr& x_init) { NICO; return arr(); }

#endif
