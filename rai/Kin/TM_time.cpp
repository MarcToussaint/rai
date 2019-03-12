/*  ------------------------------------------------------------------
    Copyright (c) 2017 Marc Toussaint
    email: marc.toussaint@informatik.uni-stuttgart.de

    This code is distributed under the MIT License.
    Please see <root-path>/LICENSE for details.
    --------------------------------------------------------------  */

#include "TM_time.h"

void TM_Time::phi(arr &y, arr &J, const rai::KinematicWorld &K) {
  y = ARR(K.frames(0)->tau);
  
  if(!!J) {
    K.jacobianTime(J, K.frames(0));
  }
}

void TM_Time::phi(arr &y, arr &J, const WorldL &Ktuple) {
  if(order==0) {
    phi(y, J, *Ktuple(-1));
    if(!!J) expandJacobian(J, Ktuple, -1);
  }
  if(order==1) { //WARNING: this is neg velocity... for ineq constraint
    arr y0, y1, J0, J1;
    phi(y0, (!!J?J0:NoArr), *Ktuple(-2));
    phi(y1, (!!J?J1:NoArr), *Ktuple(-1));
    y = y0 - y1;
    if(!!J) {
      expandJacobian(J0, Ktuple, -2);
      expandJacobian(J1, Ktuple, -1);
      J = J0 - J1;
    }
  }
  if(order==2) {
    arr y0, y1, y2, J0, J1, J2;
    phi(y0, (!!J?J0:NoArr), *Ktuple(-3));
    phi(y1, (!!J?J1:NoArr), *Ktuple(-2));
    phi(y2, (!!J?J2:NoArr), *Ktuple(-1));
    y = y2 - 2.*y1 + y0;
    if(!!J) {
      expandJacobian(J0, Ktuple, -3);
      expandJacobian(J1, Ktuple, -2);
      expandJacobian(J2, Ktuple, -1);
      J = J2 - 2.*J1 + J0;
    }
  }
}
