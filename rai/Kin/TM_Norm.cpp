/*  ------------------------------------------------------------------
    Copyright (c) 2017 Marc Toussaint
    email: marc.toussaint@informatik.uni-stuttgart.de
    
    This code is distributed under the MIT License.
    Please see <root-path>/LICENSE for details.
    --------------------------------------------------------------  */

#include "TM_Norm.h"

void TM_Norm::phi(arr& y, arr& J, const mlr::KinematicWorld& G, int t){
  map->phi(y, J, G, t);
  double l = sqrt(sumOfSqr(y));
  if(&J) J = ~(y/l)*J;
  y = ARR(l);
}

uint TM_Norm::dim_phi(const mlr::KinematicWorld& G){
  return 1;
}
