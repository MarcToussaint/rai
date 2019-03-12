/*  ------------------------------------------------------------------
    Copyright (c) 2017 Marc Toussaint
    email: marc.toussaint@informatik.uni-stuttgart.de

    This code is distributed under the MIT License.
    Please see <root-path>/LICENSE for details.
    --------------------------------------------------------------  */

#include "feature.h"

//===========================================================================

void Feature::phi(arr& y, arr& J, const WorldL& Ktuple) {
  CHECK_GE(Ktuple.N, order+1,"I need at least " <<order+1 <<" configurations to evaluate");
  if(order==0) {
    phi(y, J, *Ktuple(-1));
    if(!!J) expandJacobian(J, Ktuple, -1);
    return;
  }

  arr y0, y1, Jy0, Jy1;
  order--;
  phi(y0, (!!J?Jy0:NoArr), Ktuple({0,-2}));  if(!!J) padJacobian(Jy0, Ktuple);
  phi(y1, (!!J?Jy1:NoArr), Ktuple);
  order++;

  if(flipTargetSignOnNegScalarProduct) if(scalarProduct(y0, y1)<-.0) { y0 *= -1.;  if(!!J) Jy0 *= -1.; }

  y = y1-y0;
  if(!!J) J = Jy1 - Jy0;

#if 1 //feature itself does not care for tau!!! use specialized features, e.g. linVel, angVel
  if(Ktuple(-1)->hasTimeJoint()){
    double tau; arr Jtau;
    Ktuple(-1)->kinematicsTau(tau, (!!J?Jtau:NoArr));
    CHECK_GE(tau, 1e-10, "");
    y /= tau;
    if(!!J){
      J /= tau;
      expandJacobian(Jtau, Ktuple, -1);
      J += (-1./tau)*y*Jtau;
    }
  }else{
    double tau = Ktuple(-1)->frames(0)->tau;
    if(tau){
      CHECK_GE(tau, 1e-10, "");
      y /= tau;
      if(!!J) J /= tau;
    }
  }
#endif
}

VectorFunction Feature::vf(rai::KinematicWorld& K) { ///< direct conversion to vector function: use to check gradient or evaluate
  return [this, &K](arr& y, arr& J, const arr& x) -> void {
    K.setJointState(x);
    phi(y, J, K);
  };
}


VectorFunction Feature::vf(WorldL& Ktuple) { ///< direct conversion to vector function: use to check gradient or evaluate
  return [this, &Ktuple](arr& y, arr& J, const arr& x) -> void {
    uintA qdim = getKtupleDim(Ktuple);
    qdim.prepend(0);
    for(uint i=0;i<Ktuple.N;i++)
      Ktuple(i)->setJointState(x({qdim(i), qdim(i+1)-1}));
    phi(y, J, Ktuple);
  };
}

void Feature::applyLinearTrans(arr& y, arr& J){
  if(target.N){
    if(flipTargetSignOnNegScalarProduct){
      if(scalarProduct(y, target)<-.0) { y *= -1.;  if(!!J) J *= -1.; }
    }
    y -= target;
  }
  if(scale.N) {
    if(scale.N==1){ //scalar
      y *= scale.scalar();
      if(!!J) J *= scale.scalar();
    }else if(scale.nd==1){ //element-wise
      CHECK_EQ(scale.d0, y.N, "");
      y = scale % y;
      if(!!J) J = scale % J;
    }else if(scale.nd==2){ //matrix
      CHECK_EQ(scale.d1, y.N, "");
      y = scale * y;
      if(!!J) J = scale * J;
    }
  }
}

uint Feature::applyLinearTrans_dim(uint d){
  if(scale.N){
    if(scale.N==1){ //scalar
      return d;
    }else if(scale.nd==1){ //element-wise
      return d;
    }else if(scale.nd==2){ //matrix
      CHECK_EQ(scale.d1, d, "");
      return scale.d0;
    }
  }
  return d;
}
