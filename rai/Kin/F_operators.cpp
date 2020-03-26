/*  ------------------------------------------------------------------
    Copyright (c) 2019 Marc Toussaint
    email: marc.toussaint@informatik.uni-stuttgart.de

    This code is distributed under the MIT License.
    Please see <root-path>/LICENSE for details.
    --------------------------------------------------------------  */

#include "F_operators.h"

//===========================================================================

void F_Max::phi(arr& y, arr& J, const rai::Configuration& G) {
  f->__phi(y, J, G);
  uint i=argmax(y);
  y = ARR(y(i));
  if(!!J) J=~J[i];
  if(neg) { y*=-1.; if(!!J) J*=-1.; }
}

//===========================================================================

void F_Norm::phi(arr& y, arr& J, const rai::Configuration& G) {
  f->__phi(y, J, G);
  double l = sqrt(sumOfSqr(y));
  if(!!J) J = ~(y/l)*J;
  y = ARR(l);
}

//===========================================================================

void F_Normalized::phi(arr& y, arr& J, const rai::Configuration& G) {
  f->__phi(y, J, G);
  normalizeWithJac(y, J);
}
