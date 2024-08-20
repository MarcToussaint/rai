/*  ------------------------------------------------------------------
    Copyright (c) 2011-2024 Marc Toussaint
    email: toussaint@tu-berlin.de

    This code is distributed under the MIT License.
    Please see <root-path>/LICENSE for details.
    --------------------------------------------------------------  */

#include "feature.h"

//===========================================================================

arr Feature::phi_finiteDifferenceReduce(const FrameL& F) {
  CHECK(order>0, "can't reduce for order=0");

  timeIntegral--;
  order--;
  arr y0 = phi(F({0, -2}));
  arr y1 = phi(F({1, -1}));
  order++;
  timeIntegral++;

  if(flipTargetSignOnNegScalarProduct) if(scalarProduct(y0, y1)<-.0) { y0 *= -1.; }

  CHECK_EQ(y0.N, y1.N, "feature dim differs over time slices -- that's unusual. Possible case: qZeroVel across a switch, which happens in walker skeleton if the last entry does not indicate switch of robot");
  arr y = y1-y0;

  if(!y.N) return y;

  if(!diffInsteadOfVel) {
    double tau; arr Jtau;
    F.elem(-1)->C.kinematicsTau(tau, Jtau, F.elem(-1));
    CHECK_GE(tau, 1e-10, "");
    if(timeIntegral<=0) {
      y /= tau;
      if(Jtau.N && y.jac) y.J() += (-1./tau)*y.noJ()*Jtau;
    } else { //this assumes that we talk about a SOS feature! and that the cost is multiplied by tau (the feature by sqrt(tau))
      y /= sqrt(tau);
      if(Jtau.N && y.jac) y.J() += (-0.5/tau)*y.noJ()*Jtau;
    }
  }
  return y;
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

void Feature::setup(const rai::Configuration& C, const StringA& frames, const arr& _scale, const arr& _target, int _order) {
  //-- if arguments are given, modify the feature's frames, scaling and order
  if(frames.N) {
    if(frames.N==1 && frames.scalar()=="ALL") frameIDs = framesToIndices(C.frames); //important! this means that, if no explicit selection of frames was made, all frames (of a time slice) are referred to
    else frameIDs = C.getFrameIDs(frames);
  }
  if(!!_scale) scale = _scale;
  if(!!_target) target = _target;
  if(_order>=0) order = _order;
}

FrameL Feature::getFrames(const rai::Configuration& C, uint s) {
  FrameL F;
  if(C.frames.nd==1) {
    CHECK(!s, "C does not have multiple slices");
    CHECK(!order, "can't ground a order>0 feature on configuration without slices");
    F = C.getFrames(frameIDs);
    F.reshape(1, F.N);
  } else {
    CHECK_EQ(C.frames.nd, 2, "");
    CHECK_GE(C.frames.d0, order+s+1, "");
    F.resize(order+1, frameIDs.N);
    for(uint i=0; i<=order; i++) {
      for(uint j=0; j<frameIDs.N; j++) {
        uint fID = frameIDs.elem(j);
        F(i, j) = C.frames(s+i-order, fID);
      }
    }
  }
  if(frameIDs.nd==2) {
    F.reshape(order+1, frameIDs.d0, frameIDs.d1);
  }
  return F;
}

arr Feature::phi(const FrameL& F) {
  arr y, J;
  phi2(y, J, F);
  if(!!J) {
    CHECK_EQ(J.d0, y.N, "wrong Jacobian size");
    CHECK(!J.jac, "");
    y.J() = J;
  }
  return y;
}

void grabJ(arr& y, arr& J) {
  CHECK(&J != y.jac.get(), "");
  if(!!J) {
    if(y.jac) { J=(*y.jac); y.jac.reset(); }
    else J.setNoArr();
  }
}

void Feature::phi2(arr& y, arr& J, const FrameL& F) {
  y = phi_finiteDifferenceReduce(F);
  grabJ(y, J);
}

rai::String Feature::shortTag(const rai::Configuration& C) {
  rai::String s;
  s <<rai::niceTypeidName(typeid(*this));

  if(order==1) s <<"/vel";
  else if(order==2) s <<"/acc";
  else if(order>2) s <<'/' <<order;

  s <<'[';
  if(frameIDs.N<=3) {
    int comma=0;
    for(uint i:frameIDs){ if(comma++) s <<',';  s <<C.frames.elem(i)->name; }
  } else {
    s <<'#' <<frameIDs.N;
  }
  s <<']';
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

fct Feature::vf2(const FrameL& F) { ///< direct conversion to vector function: use to check gradient or evaluate
  return [this, &F](const arr& x) -> arr {
    F.first()->C.setJointState(x);
//    auto jacMode = F.first()->C.jacMode;
//    F.first()->C.setJacModeAs(J);
    arr y = phi(F);
//    grabJ(y, J);
    //if(!!J && y.jac) J = *y.jac;
//    F.first()->C.jacMode=jacMode;
    return y;
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

void Feature::applyLinearTrans(arr& y) {
  if(target.N) {
    if(flipTargetSignOnNegScalarProduct) {
      if(scalarProduct(y, target)<-.0) { y *= -1.;  /*if(!!J) J *= -1.; */}
    }
    if(target.N==1) { //scalar
      y -= target.scalar();
    } else {
      y -= target;
    }
  }
  if(scale.N) {
    if(scale.N==1) { //scalar
      y *= scale.scalar();
      //      if(!!J) J *= scale.scalar(); //automatic
    } else if(scale.nd==1) { //element-wise
      CHECK_EQ(scale.d0, y.N, "");
      y = y % scale;
//      if(y.jac){
//        if(isSparseMatrix(y.J())) y.J().sparse().rowWiseMult(scale);
//        else y.J() = scale % y.J();
//      }
    } else if(scale.nd==2) { //matrix
      CHECK_EQ(scale.d1, y.N, "");
      y = scale * y;
      //if(!!J) J = scale * J; //automatic
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
