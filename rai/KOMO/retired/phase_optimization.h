/*  ------------------------------------------------------------------
    Copyright (c) 2011-2020 Marc Toussaint
    email: toussaint@tu-berlin.de

    This code is distributed under the MIT License.
    Please see <root-path>/LICENSE for details.
    --------------------------------------------------------------  */

#ifndef PHASE_OPTIMIZATION_H
#define PHASE_OPTIMIZATION_H

#include "../Optim/optimization.h"
#include "../Algo/spline.h"
#include "../Optim/kOrderMarkov.h"

struct PhaseOptimization : KOrderMarkovFunction {
  //options of the problem
  uint k;   // [3]
  uint kX;  // transition type of trajectory [1=vel,2=acc,3=jerk]
  double w; // weight of transition costs of phase
  uint T;
  rai::Spline* p;

  PhaseOptimization(arr& X, uint _kX, double _w=1.);

  arr getInitialization();
  void getSolution(arr& xOpt, arr& sOpt);

  //implementations of the kOrderMarkov virtuals
  void phi_t(arr& phi, arr& J, ObjectiveTypeA& tt, uint t, const arr& x_bar);
  uint get_T() { return T; }
  uint get_k() { return k; }
  uint dim_x() { return 1; }
  uint dim_phi(uint t) {
    uint dim = 0;
    if(t>1 && t<(T+2-kX)) {
      dim += p->points.d1; // transition costs of trajectory
      if(t<T) { dim += 1;} // transition costs of phase
    }
    return dim;
  }
  arr get_postfix();
};

#endif // PHASE_OPTIMIZATION_H
