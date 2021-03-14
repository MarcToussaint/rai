/*  ------------------------------------------------------------------
    Copyright (c) 2011-2020 Marc Toussaint
    email: toussaint@tu-berlin.de

    This code is distributed under the MIT License.
    Please see <root-path>/LICENSE for details.
    --------------------------------------------------------------  */

#include "feature.h"

//===========================================================================



void Feature::phi_finiteDifferenceReduce(arr& y, arr& J, const FrameL& F) {
  CHECK(order>0, "can't reduce for order=0");

  arr y0, y1, Jy0, Jy1;

  timeIntegral--;
  order--;
  phi2(y0, Jy0, F({0, -2}));
  phi2(y1, Jy1, F({1,-1}));
  order++;
  timeIntegral++;

  if(flipTargetSignOnNegScalarProduct) if(scalarProduct(y0, y1)<-.0) { y0 *= -1.;  if(!!J) Jy0 *= -1.; }

  y = y1-y0;
  if(!!J) J = Jy1 - Jy0;

  if(!y.N) return;

  if(!diffInsteadOfVel){
    double tau; arr Jtau;
    F.elem(-1)->C.kinematicsTau(tau, Jtau, F.elem(-1));
    CHECK_GE(tau, 1e-10, "");
    if(timeIntegral<=0){
      y /= tau;
      if(!!J) {
        J /= tau;
        if(Jtau.N) J += (-1./tau)*y*Jtau;
      }
    }else{ //this assumes that we talk about a SOS feature! and that the cost is multiplied by tau (the feature by sqrt(tau))
      y /= sqrt(tau);
      if(!!J) {
        J /= sqrt(tau);
        if(Jtau.N) J += (-0.5/tau)*y*Jtau;
      }
    }
  }
}

//void Feature::phi(arr& y, arr& J, const rai::Configuration& C) {
//  FrameL F(order+1, frameIDs.N);
//  if(order==0 && C.frames.nd==1){
//    F[0] = C.frames.sub(frameIDs);
//  }else{
//    CHECK_EQ(C.frames.nd, 2, "");
//    CHECK_GE(C.frames.d0, order+1, "");
//    for(uint k=0;k<F.d0;k++){
//      F[k] = C.frames[C.frames.d0-F.d0+k].sub(frameIDs);
//    }
//  }
//  if(frameIDs.nd==2) F.reshape(F.d0, frameIDs.d0, frameIDs.d1);
//  phi2(y, J, F);
//}

//void Feature::phi(arr& y, arr& J, const ConfigurationL& Ctuple) {
//  FrameL F(order+1, frameIDs.N);
//  for(uint k=0;k<F.d0;k++){
//    F[k] = Ctuple(Ctuple.N-F.d0+k)->frames.sub(frameIDs);
//  }
//  if(frameIDs.nd==2) F.reshape(F.d0, frameIDs.d0, frameIDs.d1);
//  phi2(y, J, F);
//}

FrameL Feature::getFrames(const rai::Configuration& C, uint s) {
  FrameL F;
  if(C.frames.nd==1){
    CHECK(!s, "C does not have multiple slices");
    CHECK(!order, "can't ground a order>0 feature on configuration without slices");
    F = C.getFrames(frameIDs);
    F.reshape(1, F.N);
  }else{
    CHECK_EQ(C.frames.nd, 2, "");
    CHECK_GE(C.frames.d0, order+s+1, "");
    F.resize(order+1, frameIDs.N);
    for(uint i=0;i<=order;i++){
      for(uint j=0;j<frameIDs.N;j++){
        uint fID = frameIDs.elem(j);
        F(i,j) = C.frames(s+i-order, fID);
      }
    }
  }
  if(frameIDs.nd==2){
    F.reshape(order+1, frameIDs.d0, frameIDs.d1);
  }
  return F;
}



rai::String Feature::shortTag(const rai::Configuration& C) {
  rai::String s;
  s <<niceTypeidName(typeid(*this));
  s <<'/' <<order;
  if(frameIDs.N<=3){
    for(uint i:frameIDs) s <<'-' <<C.frames.elem(i)->name;
  }else{
    s <<"-#" <<frameIDs.N;
  }
  return s;
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

//VectorFunction Feature::vf(rai::Configuration& C) { ///< direct conversion to vector function: use to check gradient or evaluate
//  return [this, &C](arr& y, arr& J, const arr& x) -> void {
//    C.setJointState(x);
//    C.setJacModeAs(J);
//    phi(y, J, C);
//    C.jacMode=C.JM_dense;
//  };
//}

VectorFunction Feature::vf2(const FrameL& F) { ///< direct conversion to vector function: use to check gradient or evaluate
  return [this, &F](arr& y, arr& J, const arr& x) -> void {
    F.first()->C.setJointState(x);
    auto jacMode = F.first()->C.jacMode;
    F.first()->C.setJacModeAs(J);
    phi2(y, J, F);
    F.first()->C.jacMode=jacMode;
  };
}

//VectorFunction Feature::vf(ConfigurationL& Ctuple) { ///< direct conversion to vector function: use to check gradient or evaluate
//  return [this, &Ctuple](arr& y, arr& J, const arr& x) -> void {
//    uintA qdim = getKtupleDim(Ctuple);
//    qdim.prepend(0);
//    for(uint i=0; i<Ctuple.N; i++){
//      Ctuple(i)->setJointState(x({qdim(i), qdim(i+1)-1}));
//    }
//    phi(y, J, Ctuple);
//  };
//}

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
