/*  ------------------------------------------------------------------
    Copyright (c) 2011-2020 Marc Toussaint
    email: toussaint@tu-berlin.de

    This code is distributed under the MIT License.
    Please see <root-path>/LICENSE for details.
    --------------------------------------------------------------  */

#include "TM_time.h"

void TM_Time::phi2(arr& y, arr& J, const FrameL& F) {
  if(order==0) {
    rai::Frame *f = F.scalar();
    double tau;
    f->C.kinematicsTau(tau, J, f);
    y.resize(1) = tau;
  }
  if(order==1) { //WARNING: this is neg velocity... for ineq constraint
    CHECK_EQ(F.N, 2, "");
    arr y0, y1, J0, J1;
    order=0;
    phi2(y0, J0, {F.elem(0)});
    phi2(y1, J1, {F.elem(1)});
    order=1;
    y = y0 - y1;
    if(!!J) J = J0 - J1;
  }
  if(order==2) {
    CHECK_EQ(F.N, 3, "");
    arr y0, y1, y2, J0, J1, J2;
    order=0;
    phi2(y0, J0, {F.elem(0)});
    phi2(y1, J1, {F.elem(1)});
    phi2(y2, J2, {F.elem(2)});
    order=2;
    y = y2 - 2.*y1 + y0;
    if(!!J)  J = J2 - 2.*J1 + J0;
  }
}
