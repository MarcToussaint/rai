/*  ------------------------------------------------------------------
    Copyright (c) 2017 Marc Toussaint
    email: marc.toussaint@informatik.uni-stuttgart.de

    This code is distributed under the MIT License.
    Please see <root-path>/LICENSE for details.
    --------------------------------------------------------------  */

#include "TM_Max.h"

void TM_Max::phi(arr& y, arr& J, const rai::KinematicWorld& G) {
  map->__phi(y, J, G);
  uint i=argmax(y);
  y = ARR(y(i));
  if(!!J) J=~J[i];
  if(neg) { y*=-1.; if(!!J) J*=-1.; }
}

