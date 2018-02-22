/*  ------------------------------------------------------------------
    Copyright 2016 Marc Toussaint
    email: marc.toussaint@informatik.uni-stuttgart.de
    
    This program is free software: you can redistribute it and/or modify
    it under the terms of the GNU General Public License as published by
    the Free Software Foundation, either version 3 of the License, or (at
    your option) any later version. This program is distributed without
    any warranty. See the GNU General Public License for more details.
    You should have received a COPYING file of the full GNU General Public
    License along with this program. If not, see
    <http://www.gnu.org/licenses/>
    --------------------------------------------------------------  */


#ifndef PHASE_OPTIMIZATION_H
#define PHASE_OPTIMIZATION_H

#include <Optim/optimization.h>
#include <Algo/spline.h>
#include <Optim/kOrderMarkov.h>

struct PhaseOptimization : KOrderMarkovFunction {
  //options of the problem
  uint k;   // [3]
  uint kX;  // transition type of trajectory [1=vel,2=acc,3=jerk]
  double w; // weight of transition costs of phase
  uint T;
  mlr::Spline* p;

  PhaseOptimization(arr &X, uint _kX, double _w=1.);

  arr getInitialization();
  void getSolution(arr &xOpt, arr &sOpt);

  //implementations of the kOrderMarkov virtuals
  void phi_t(arr& phi, arr& J, ObjectiveTypeA& tt, uint t, const arr& x_bar);
  uint get_T(){ return T; }
  uint get_k(){ return k; }
  uint dim_x(){ return 1; }
  uint dim_phi(uint t){
    uint dim = 0;
    if (t>1 && t<(T+2-kX)) {
      dim += p->points.d1; // transition costs of trajectory
      if (t<T){ dim += 1;} // transition costs of phase
    }
    return dim;
  }
  arr get_postfix();
};

#endif // PHASE_OPTIMIZATION_H
