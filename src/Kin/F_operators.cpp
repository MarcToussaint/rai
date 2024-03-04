/*  ------------------------------------------------------------------
    Copyright (c) 2011-2024 Marc Toussaint
    email: toussaint@tu-berlin.de

    This code is distributed under the MIT License.
    Please see <root-path>/LICENSE for details.
    --------------------------------------------------------------  */

#include "F_operators.h"

//===========================================================================

void F_Max::phi2(arr& y, arr& J,  const FrameL& F) {
  arr z = f->eval(F);
  uint i=argmax(z);
  y = arr{z(i)};
  if(!!J) J=~z.J()[i];
  if(neg) { y*=-1.; if(!!J) J*=-1.; }
}

//===========================================================================

void F_Norm::phi2(arr& y, arr& J,  const FrameL& F) {
  arr z = f->eval(F);
  double l = sqrt(sumOfSqr(z));
  if(!!J) J = ~(z/l)*z.J();
  y = arr{l};
}

//===========================================================================

void F_Normalized::phi2(arr& y, arr& J,  const FrameL& F) {
  y = f->eval(F);
  J = y.J();
  normalizeWithJac(y, J);
}
