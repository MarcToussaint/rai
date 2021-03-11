/*  ------------------------------------------------------------------
    Copyright (c) 2011-2020 Marc Toussaint
    email: toussaint@tu-berlin.de

    This code is distributed under the MIT License.
    Please see <root-path>/LICENSE for details.
    --------------------------------------------------------------  */

#include "F_operators.h"

//===========================================================================

void F_Max::phi2(arr& y, arr& J,  const FrameL& F) {
  f->eval(y, J, F);
  uint i=argmax(y);
  y = ARR(y(i));
  if(!!J) J=~J[i];
  if(neg) { y*=-1.; if(!!J) J*=-1.; }
}

//===========================================================================

void F_Norm::phi2(arr& y, arr& J,  const FrameL& F) {
  f->eval(y, J, F);
  double l = sqrt(sumOfSqr(y));
  if(!!J) J = ~(y/l)*J;
  y = ARR(l);
}

//===========================================================================

void F_Normalized::phi2(arr& y, arr& J,  const FrameL& F) {
  f->eval(y, J, F);
  normalizeWithJac(y, J);
}
