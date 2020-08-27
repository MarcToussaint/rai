/*  ------------------------------------------------------------------
    Copyright (c) 2011-2020 Marc Toussaint
    email: toussaint@tu-berlin.de

    This code is distributed under the MIT License.
    Please see <root-path>/LICENSE for details.
    --------------------------------------------------------------  */

#include "feature.h"

//===========================================================================

void Feature::phi2(arr& y, arr& J, const FrameL& F) {
  CHECK(order>0, "phi needs to be implemented at least for order=0");

  arr y0, y1, Jy0, Jy1;
//  if(isSparseMatrix(J)){ Jy0.sparse(); Jy1.sparse(); }
  order--;
  phi2(y0, (!!J?Jy0:NoArr), F({0, -2}));
  phi2(y1, (!!J?Jy1:NoArr), F({1,-1}));
  order++;

  if(flipTargetSignOnNegScalarProduct) if(scalarProduct(y0, y1)<-.0) { y0 *= -1.;  if(!!J) Jy0 *= -1.; }

  y = y1-y0;
  if(!!J) J = Jy1 - Jy0;

#if 0 //feature itself does not care for tau!!! use specialized features, e.g. linVel, angVel
  if(Ctuple(-1)->hasTauJoint()) {
    double tau; arr Jtau;
    Ctuple(-1)->kinematicsTau(tau, (!!J?Jtau:NoArr));
    CHECK_GE(tau, 1e-10, "");
    y /= tau;
    if(!!J) {
      J /= tau;
      expandJacobian(Jtau, Ctuple, -1);
      J += (-1./tau)*y*Jtau;
    }
  } else {
    double tau = Ctuple(-1)->frames(0)->tau;
    if(tau) {
      CHECK_GE(tau, 1e-10, "");
      y /= tau;
      if(!!J) J /= tau;
    }
  }
#else
  double tau = F.first()->C.frames(0)->tau;
  if(tau) {
    CHECK_GE(tau, 1e-10, "");
    y /= tau;
    if(!!J) J /= tau;
  }
#endif
}

void Feature::phi(arr& y, arr& J, const ConfigurationL& Ctuple) {
  CHECK_GE(Ctuple.N, order+1, "I need at least " <<order+1 <<" configurations to evaluate");
  if(order==0) {
    phi(y, J, *Ctuple(-1));
    if(!!J) expandJacobian(J, Ctuple, -1);
    return;
  }

  arr y0, y1, Jy0, Jy1;
  order--;
  phi(y0, (!!J?Jy0:NoArr), Ctuple({0, -2}));  if(!!J) padJacobian(Jy0, Ctuple);
  phi(y1, (!!J?Jy1:NoArr), Ctuple);
  order++;

  if(flipTargetSignOnNegScalarProduct) if(scalarProduct(y0, y1)<-.0) { y0 *= -1.;  if(!!J) Jy0 *= -1.; }

  y = y1-y0;
  if(!!J) J = Jy1 - Jy0;

#if 1 //feature itself does not care for tau!!! use specialized features, e.g. linVel, angVel
  if(Ctuple(-1)->hasTauJoint()) {
    double tau; arr Jtau;
    Ctuple(-1)->kinematicsTau(tau, (!!J?Jtau:NoArr));
    CHECK_GE(tau, 1e-10, "");
    y /= tau;
    if(!!J) {
      J /= tau;
      expandJacobian(Jtau, Ctuple, -1);
      J += (-1./tau)*y*Jtau;
    }
  } else {
    double tau = Ctuple(-1)->frames(0)->tau;
    if(tau) {
      CHECK_GE(tau, 1e-10, "");
      y /= tau;
      if(!!J) J /= tau;
    }
  }
#endif
}

//void Feature::signature(intA& S, const ConfigurationL& Ctuple){
//  uintA cdim = getKtupleDim(Ctuple);
//  cdim.prepend(0);
//  S.clear();
//  for(uint k=0;k<Ctuple.N;k++){
//    intA Sk;
//    signature(Sk, *Ctuple(k));
//    Sk += (int)cdim(k);
//    S.setAppend(Sk);
//  }
//}

VectorFunction Feature::vf(rai::Configuration& C) { ///< direct conversion to vector function: use to check gradient or evaluate
  return [this, &C](arr& y, arr& J, const arr& x) -> void {
    C.setJointState(x);
    phi(y, J, C);
  };
}

VectorFunction Feature::vf(ConfigurationL& Ctuple) { ///< direct conversion to vector function: use to check gradient or evaluate
  return [this, &Ctuple](arr& y, arr& J, const arr& x) -> void {
    uintA qdim = getKtupleDim(Ctuple);
    qdim.prepend(0);
    for(uint i=0; i<Ctuple.N; i++)
      Ctuple(i)->setJointState(x({qdim(i), qdim(i+1)-1}));
    phi(y, J, Ctuple);
  };
}

void Feature::applyLinearTrans(arr& y, arr& J) {
  if(target.N) {
    if(flipTargetSignOnNegScalarProduct) {
      if(scalarProduct(y, target)<-.0) { y *= -1.;  if(!!J) J *= -1.; }
    }
    y -= target;
  }
  if(scale.N) {
    if(scale.N==1) { //scalar
      y *= scale.scalar();
      if(!!J) J *= scale.scalar();
    } else if(scale.nd==1) { //element-wise
      CHECK_EQ(scale.d0, y.N, "");
      y = scale % y;
      if(!!J){
        if(isSparseMatrix(J)) J.sparse().rowWiseMult(scale);
        else J = scale % J;
      }
    } else if(scale.nd==2) { //matrix
      CHECK_EQ(scale.d1, y.N, "");
      y = scale * y;
      if(!!J) J = scale * J;
    }
  }
}

uint Feature::applyLinearTrans_dim(uint d) {
  if(scale.N) {
    if(scale.N==1) { //scalar
      return d;
    } else if(scale.nd==1) { //element-wise
      return d;
    } else if(scale.nd==2) { //matrix
      CHECK_EQ(scale.d1, d, "");
      return scale.d0;
    }
  }
  return d;
}
