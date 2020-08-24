/*  ------------------------------------------------------------------
    Copyright (c) 2011-2020 Marc Toussaint
    email: toussaint@tu-berlin.de

    This code is distributed under the MIT License.
    Please see <root-path>/LICENSE for details.
    --------------------------------------------------------------  */

#include "phase_optimization.h"

PhaseOptimization::PhaseOptimization(arr& X, uint _kX, double _w) {
  T = X.d0;
  p = new rai::Spline(T, X, 1);
  kX = _kX;
  k = 3;
  w = sqrt(_w);
}

arr PhaseOptimization::get_postfix() {
  arr x = ones(get_k(), dim_x());
  return x;
}

arr PhaseOptimization::getInitialization() {
  arr s0 = linspace(0, 1, T-1); s0.reshapeFlat();
  s0 = s0.sub(1, s0.d0-2); // remove 0 and 1 from optimization variables
  return s0;
}

void PhaseOptimization::getSolution(arr& xOpt, arr& sOpt) {
  xOpt.clear();
  sOpt.prepend(0.); sOpt.append(1.); // add 0 and 1 to optimization variables
  for(uint i=0; i<sOpt.d0; i++) {
    xOpt.append(~p->eval(sOpt(i)));
  }
}

void PhaseOptimization::phi_t(arr& phi, arr& J, ObjectiveTypeA& tt, uint t, const arr& x_bar) {
  uint T=get_T(), n=dim_x(), k=get_k();

  //assert some dimensions
  CHECK_EQ(x_bar.d0, k+1, "");
  CHECK_EQ(x_bar.d1, n, "");
  CHECK_LE(t, T, "");

  //-- transition costs of trajectory: append to phi
  if(kX==1)  phi = p->eval(x_bar(1, 0)) - p->eval(x_bar(0, 0)); //penalize velocity
  if(kX==2)  phi = p->eval(x_bar(2, 0)) - 2.*p->eval(x_bar(1, 0)) + p->eval(x_bar(0, 0)); //penalize acceleration
  if(kX==3)  phi = p->eval(x_bar(3, 0)) - 3.*p->eval(x_bar(2, 0)) + 3.*p->eval(x_bar(1, 0)) - p->eval(x_bar(0, 0)); //penalize jerk

  //-- transition costs of phase: append to phi
  if(t<T) { phi.append((x_bar(2, 0) - 2.*x_bar(1, 0) + x_bar(0, 0))*w); }

  if(!!tt) tt.append(OT_sos, phi.N);

  uint m=phi.N;
  CHECK_EQ(m, dim_phi(t), "");
  if(!!tt) CHECK_EQ(m, tt.N, "");

  if(!!J) { //we also need to return the Jacobian
    J.resize(m, k+1, n).setZero();
    //-- transition costs of trajectory
    for(uint j=0; j<m-1; j++) {
      if(kX==1) { J(j, 1, 0) = p->eval(x_bar(1, 0), true)(j);  J(j, 0, 0) = -p->eval(x_bar(0, 0), true)(j);}
      if(kX==2) { J(j, 2, 0) = p->eval(x_bar(2, 0), true)(j);  J(j, 1, 0) = -2.*p->eval(x_bar(1, 0), true)(j);  J(j, 0, 0) = p->eval(x_bar(0, 0), true)(j); }
      if(kX==3) { J(j, 3, 0) = p->eval(x_bar(3, 0), true)(j);  J(j, 2, 0) = -3.*p->eval(x_bar(2, 0), true)(j);  J(j, 1, 0) = +3.*p->eval(x_bar(1, 0), true)(j);  J(j, 0, 0) = -1.*p->eval(x_bar(0, 0), true)(j); }
    }
    //-- transition costs of phase
    for(uint j=m-1; j<m; j++) {
      if(t<T) {J(j, 2, 0) = 1.*w;  J(j, 1, 0) = -2.*w;  J(j, 0, 0) = 1.*w;}
    }
  }
}
