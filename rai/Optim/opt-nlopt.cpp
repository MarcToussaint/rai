/*  ------------------------------------------------------------------
    Copyright (c) 2011-2020 Marc Toussaint
    email: toussaint@tu-berlin.de

    This code is distributed under the MIT License.
    Please see <root-path>/LICENSE for details.
    --------------------------------------------------------------  */

#include "opt-nlopt.h"

#ifdef RAI_NLOPT

#include <nlopt.hpp>

struct FuncCallData {
  NLOptInterface* I=0;
  uint feature=0;
};

arr NLOptInterface::solve() {
  arr x = P.getInitializationSample();
  arr bounds_lo, bounds_up;
  P.getBounds(bounds_lo, bounds_up);

  CHECK_EQ(x.N, bounds_up.N, "NLOpt requires bounds");
  CHECK_EQ(x.N, bounds_lo.N, "NLOpt requires bounds");
  for(uint i=0; i<bounds_lo.N; i++) CHECK(bounds_lo.elem(i)<bounds_up.elem(i), "NLOpt requires bounds");
  CHECK_LE(max(x-bounds_up), 0., "initialization is above upper bound");
  CHECK_GE(min(x-bounds_lo), 0., "initialization is above upper bound");

#if 0
  //  nlopt::opt opt(nlopt::LD_SLSQP, x.N); //works well for toys
  //  nlopt::opt opt(nlopt::LD_MMA, x.N);
  nlopt::opt opt(nlopt::LN_COBYLA, x.N); //works for KOMO
#else
  nlopt::opt opt(nlopt::LD_AUGLAG, x.N);
//  nlopt::opt opt(nlopt::LD_AUGLAG_EQ, x.N);

//  nlopt::opt subopt(nlopt::LD_MMA, x.N);
//  nlopt::opt subopt(nlopt::LN_NELDERMEAD, x.N);
  nlopt::opt subopt(nlopt::LD_LBFGS, x.N);
//  nlopt::opt subopt(nlopt::LD_SLSQP, x.N); //works well for toys
//  nlopt::opt subopt(nlopt::LD_TNEWTON_PRECOND_RESTART, x.N);
  subopt.set_xtol_abs(1e-4);
  opt.set_local_optimizer(subopt);
#endif

  opt.set_min_objective(_f, this);
  opt.set_xtol_abs(1e-4);
//  opt.set_ftol_abs(1e-3);
  if(bounds_lo.N==x.N) opt.set_lower_bounds(bounds_lo.vec());
  if(bounds_up.N==x.N) opt.set_upper_bounds(bounds_up.vec());

  rai::Array<FuncCallData> funcCallData(featureTypes.N);
  for(uint i=0; i<featureTypes.N; i++) {
    FuncCallData& d = funcCallData.elem(i);
    d.I = this;
    d.feature = i;
    if(featureTypes.elem(i) == OT_ineq) opt.add_inequality_constraint(_g, &d);
    if(featureTypes.elem(i) == OT_eq) opt.add_equality_constraint(_h, &d);
  }

  double fval;
  try {
    std::vector<double> x_vec = x.vec();
    nlopt::result R = opt.optimize(x_vec, fval);
    x = x_vec;
  } catch(const std::runtime_error& err) {
    cout <<"NLOPT terminated with " <<err.what() <<endl;
  }
  cout <<"NLOPT final costs:" <<opt.last_optimum_value() <<endl;
  return x;
}

double NLOptInterface::f(const arr& _x, arr& _grad) {
  if(x!=_x) { x=_x;  P.evaluate(phi_x, J_x, x); }
  CHECK_EQ(phi_x.N, featureTypes.N, "");
  double fval=0;
  arr grad=zeros(_grad.N);
  for(uint i=0; i<phi_x.N; i++) {
    if(featureTypes.elem(i)==OT_f) { fval += phi_x.elem(i);  if(grad.N) grad += J_x[i]; }
    if(featureTypes.elem(i)==OT_sos) { double y = phi_x.elem(i);  fval += y*y;  if(grad.N) grad += (2.*y) * J_x[i]; }
  }
//  for(uint i=0; i<grad.N; i++) _grad[i] = grad.elem(i);
  _grad = grad;
//  cout <<fval <<endl;
  return fval;
}

double NLOptInterface::g(const arr& _x, arr& _grad, uint feature) {
  if(x!=_x) { x=_x;  P.evaluate(phi_x, J_x, x); }
  CHECK_EQ(featureTypes(feature), OT_ineq, "");
  for(uint i=0; i<_grad.N; i++) _grad[i] = J_x(feature, i);
  return phi_x.elem(feature);

}

double NLOptInterface::h(const arr& _x, arr& _grad, uint feature) {
  if(x!=_x) { x=_x;  P.evaluate(phi_x, J_x, x); }
  CHECK_EQ(featureTypes(feature), OT_eq, "");
  for(uint i=0; i<_grad.N; i++) _grad[i] = J_x(feature, i);
  return phi_x.elem(feature);
}

double NLOptInterface::_f(const std::vector<double>& _x, std::vector<double>& _grad, void* f_data) {
  arr x(_x, true);
  arr grad(_grad, true);
  return ((NLOptInterface*)f_data) -> f(x, grad);
}

double NLOptInterface::_g(const std::vector<double>& _x, std::vector<double>& _grad, void* f_data) {
  arr x(_x, true);
  arr grad(_grad, true);
  FuncCallData* d = (FuncCallData*)f_data;
  return d->I->g(x, grad, d->feature);
}

double NLOptInterface::_h(const std::vector<double>& _x, std::vector<double>& _grad, void* f_data) {
  arr x(_x, true);
  arr grad(_grad, true);
  FuncCallData* d = (FuncCallData*)f_data;
  return d->I->h(x, grad, d->feature);
}

#else

arr NLOptInterface::solve() { NICO; return arr(); }

#endif
