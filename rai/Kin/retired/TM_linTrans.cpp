/*  ------------------------------------------------------------------
    Copyright (c) 2011-2020 Marc Toussaint
    email: toussaint@tu-berlin.de

    This code is distributed under the MIT License.
    Please see <root-path>/LICENSE for details.
    --------------------------------------------------------------  */

#include "TM_linTrans.h"

void TM_LinTrans::phi(arr& y, arr& J, const rai::Configuration& G) {
  map->__phi(y, J, G);
  if(A.N) {
    y = A*y;
    if(!!J) J = A*J;
  }
  if(a.N) y += a;
}

uint TM_LinTrans::dim_phi(const rai::Configuration& G) {
  return A.d0;
}
