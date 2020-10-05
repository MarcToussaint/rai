/*  ------------------------------------------------------------------
    Copyright (c) 2011-2020 Marc Toussaint
    email: toussaint@tu-berlin.de

    This code is distributed under the MIT License.
    Please see <root-path>/LICENSE for details.
    --------------------------------------------------------------  */

#include "F_pose.h"
#include "TM_default.h"

//===========================================================================

void F_Position::phi2(arr& y, arr& J, const FrameL& F) {
  if(order>0){  Feature::phi2(y, J, F);  return;  }
  CHECK_EQ(F.N, 1, "");
  rai::Frame *f = F.elem(0);
  f->C.kinematicsPos(y, J, f);
}

//===========================================================================

void F_PositionDiff::phi2(arr& y, arr& J, const FrameL& F) {
  if(order>0){  Feature::phi2(y, J, F);  return;  }
  CHECK_EQ(F.N, 2, "");
  rai::Frame *f1 = F.elem(0);
  rai::Frame *f2 = F.elem(1);
  arr y2, J2;
  f1->C.kinematicsPos(y, J, f1);
  f2->C.kinematicsPos(y2, J2, f2);
  y -= y2;
  J -= J2;
}

//===========================================================================

void F_PositionRel::phi2(arr& y, arr& J, const FrameL& F) {
  if(order>0){  Feature::phi2(y, J, F);  return;  }
  CHECK_EQ(F.N, 2, "");
  rai::Frame *f1 = F.elem(0);
  rai::Frame *f2 = F.elem(1);
  arr y1, y2, J1, J2;
  f1->C.kinematicsPos(y1, J1, f1);
  f2->C.kinematicsPos(y2, J2, f2);
  arr Rinv = ~(f2->ensure_X().rot.getArr());
  y = Rinv * (y1 - y2);
  if(!!J) {
    arr A;
    f2->C.jacobian_angular(A, f2);
    J = Rinv * (J1 - J2 - crossProduct(A, y1 - y2));
  }
}

//===========================================================================

void F_Vector::phi2(arr& y, arr& J, const FrameL& F){
  if(order>0){  Feature::phi2(y, J, F);  return;  }
  CHECK_EQ(F.N, 1, "");
  rai::Frame *f = F.elem(0);
  f->C.kinematicsVec(y, J, f, vec);
}

//===========================================================================

void F_VectorDiff::phi2(arr& y, arr& J, const FrameL& F){
  if(order>0){  Feature::phi2(y, J, F);  return;  }
  CHECK_EQ(F.N, 2, "");
  rai::Frame *f1 = F.elem(0);
  rai::Frame *f2 = F.elem(1);
  arr y2, J2;
  f1->C.kinematicsVec(y, J, f1, vec1);
  f2->C.kinematicsVec(y2, J2, f2, vec2);
  y -= y2;
  J -= J2;
}

//===========================================================================

void F_VectorRel::phi2(arr& y, arr& J, const FrameL& F){
  NIY;
}

//===========================================================================

void F_Quaternion::phi2(arr& y, arr& J, const FrameL& F){
  flipTargetSignOnNegScalarProduct = true;
  if(order>0){  Feature::phi2(y, J, F);  return;  }
  CHECK_EQ(F.N, 1, "");
  rai::Frame *f = F.elem(0);

  f->C.kinematicsQuat(y, J, f);
}

//===========================================================================

void F_QuaternionDiff::phi2(arr& y, arr& J, const FrameL& F){
  if(order>0){  Feature::phi2(y, J, F);  return;  }
  CHECK_EQ(F.N, 2, "");
  rai::Frame *f1 = F.elem(0);
  rai::Frame *f2 = F.elem(1);
  arr y2, J2;
  f1->C.kinematicsQuat(y, J, f1);
  f2->C.kinematicsQuat(y2, J2, f2);
  if(scalarProduct(y, y2)>=0.) {
    y -= y2;
    J -= J2;
  } else {
    y += y2;
    J += J2;
  }
}

//===========================================================================

void F_QuaternionRel::phi2(arr& y, arr& J, const FrameL& F){
  flipTargetSignOnNegScalarProduct = true;
  if(order>0){  Feature::phi2(y, J, F);  return;  }
  CHECK_EQ(F.N, 2, "");
  rai::Frame *f1 = F.elem(0);
  rai::Frame *f2 = F.elem(1);

  arr qa, qb, Ja, Jb;
  f1->C.kinematicsQuat(qb, Jb, f1);
  f2->C.kinematicsQuat(qa, Ja, f2);

  arr Jya, Jyb;
  arr ainv = qa;
  if(qa(0)!=1.) ainv(0) *= -1.;
  quat_concat(y, Jya, Jyb, ainv, qb);
  if(qa(0)!=1.) for(uint i=0; i<Jya.d0; i++) Jya(i, 0) *= -1.;

  J = Jya * Ja + Jyb * Jb;
  checkNan(J);
}

//===========================================================================

void F_ScalarProduct::phi2(arr& y, arr& J, const FrameL& F){
  if(order>0){  Feature::phi2(y, J, F);  return;  }
  CHECK_EQ(F.N, 2, "");
  rai::Frame *f1 = F.elem(0);
  rai::Frame *f2 = F.elem(1);

  CHECK(fabs(vec1.length()-1.)<1e-4, "vector references must be normalized");
  CHECK(fabs(vec2.length()-1.)<1e-4, "vector references must be normalized");

  arr zi, Ji, zj, Jj;
  f1->C.kinematicsVec(zi, Ji, f1, vec1);
  f2->C.kinematicsVec(zj, Jj, f2, vec2);

  y.resize(1);
  y(0) = scalarProduct(zi, zj);
  J = ~zj * Ji + ~zi * Jj;
}

//===========================================================================

void F_Pose::phi2(arr& y, arr& J, const FrameL& F) {
  auto pos = evalFeature<F_Position>(F, order);
  auto quat = evalFeature<F_Quaternion>(F, order);
  y.setBlockVector(pos.y, quat.y);
  J.setBlockMatrix(pos.J, quat.J);
}

//===========================================================================

void F_PoseDiff::phi2(arr& y, arr& J, const FrameL& F) {
  auto pos = evalFeature<F_PositionDiff>(F, order);
  auto quat = evalFeature<F_QuaternionDiff>(F, order);
  y.setBlockVector(pos.y, quat.y);
  J.setBlockMatrix(pos.J, quat.J);
}

//===========================================================================

void F_PoseRel::phi2(arr& y, arr& J, const FrameL& F) {
  auto pos = evalFeature<F_PositionRel>(F, order);
  auto quat = evalFeature<F_QuaternionRel>(F, order);
  y.setBlockVector(pos.y, quat.y);
  J.setBlockMatrix(pos.J, quat.J);
}

//===========================================================================

void angVel_base(rai::Frame* f0, rai::Frame* f1, arr& y, arr& J) {

  arr a, b, y_tmp, Ja, Jb;
  f0->C.kinematicsQuat(a, Ja, f0);
  f1->C.kinematicsQuat(b, Jb, f1);
  arr J0, J1;
//  quat_diffVector(y, J0, J1, a, b);
  if(scalarProduct(a, b)<0.) {
    b*=-1.;
    Jb*=-1.;
  }
  arr dq = b-a;
  a(0) *=-1.;
  quat_concat(y_tmp, J0, J1, dq, a); //y_tmp = (b-a)*a^{-1}
  for(uint i=0; i<J1.d0; i++) J1(i, 0) *= -1.;
  y_tmp.remove(0);
  J0.delRows(0);
  J1.delRows(0);

  y_tmp *= 2.;
  J0 *= 2.;
  J1 *= 2.;

  y = y_tmp;

  checkNan(y);

  if(!!J && !!Ja) {
    if(&f0->C!=&f1->C){ //different configurations -> assume consecutive
      J = catCol((J1-J0)*Ja, J0*Jb);
    }else{//same configuration
      J = (J1-J0)*Ja;
      J += J0*Jb;
    }
    checkNan(J);
  }else J.setNoArr();
}

//===========================================================================

void F_LinVel::phi2(arr& y, arr& J, const FrameL& F) {
  CHECK_GE(order, 1, "");
  if(order==1) {
    rai::Frame* f0 = F.elem(0);
    rai::Frame* f1 = F.elem(1);

    arr a, b, Ja, Jb;
    f0->C.kinematicsPos(a, Ja, f0);
    f1->C.kinematicsPos(b, Jb, f1);

    y = b-a;
    if(!!J) J = Jb-Ja;

#if 1
    rai::Frame *r = f1->getRoot();
    if(r->C.hasTauJoint(r)) {
      double tau; arr Jtau;
      r->C.kinematicsTau(tau, Jtau, r);
      CHECK_GE(tau, 1e-10, "");
      y /= tau;
      if(!!J) {
        J /= tau;
        J += (-1./tau)*y*Jtau;
      }
    } else {
      double tau = r->C.frames.first()->tau;
      CHECK_GE(tau, 1e-10, "");
      y /= tau;
      if(!!J) J /= tau;
    }
#endif
    return;
  }

  if(order==2) {
    if(impulseInsteadOfAcceleration) diffInsteadOfVel=true;
    Feature::phi2(y, J, F);
    if(impulseInsteadOfAcceleration) diffInsteadOfVel=false;
  }
}

//===========================================================================

void F_AngVel::phi2(arr& y, arr& J, const FrameL& F){
  CHECK_GE(order, 1, "");
  if(order==1) {
    rai::Frame* f0 = F.elem(0);
    rai::Frame* f1 = F.elem(1);

    angVel_base(f0, f1, y, J);

#if 1
    rai::Frame *r = f1->getRoot();
    if(r->C.hasTauJoint(r)) {
      double tau; arr Jtau;
      r->C.kinematicsTau(tau, Jtau, r);
      CHECK_GE(tau, 1e-10, "");
      y /= tau;
      if(!!J) {
        J /= tau;
        J += (-1./tau)*y*Jtau;
      }
    } else {
      double tau = r->C.frames.first()->tau;
      CHECK_GE(tau, 1e-10, "");
      y /= tau;
      if(!!J) J /= tau;
    }
#endif
    return;
  }

  if(order==2) {
    if(impulseInsteadOfAcceleration) diffInsteadOfVel=true;
    Feature::phi2(y, J, F);
    if(impulseInsteadOfAcceleration) diffInsteadOfVel=false;
  }
}

//===========================================================================

void F_LinAngVel::phi2(arr& y, arr& J, const FrameL& F) {

  F_LinVel lin;
  lin.order=order;
  lin.impulseInsteadOfAcceleration = impulseInsteadOfAcceleration;
  arr  yl, Jl;
  lin.__phi2(yl, Jl, F);

  F_AngVel ang;
  ang.order=order;
  ang.impulseInsteadOfAcceleration = impulseInsteadOfAcceleration;
  arr ya, Ja;
  ang.__phi2(ya, Ja, F);

  y.setBlockVector(yl, ya);
  J.setBlockMatrix(Jl, Ja);
}

//===========================================================================

void F_NoJumpFromParent_OBSOLETE::phi2(arr& y, arr& J, const FrameL& F) {
  CHECK_EQ(order, 1, "");
  CHECK_EQ(F.d1, 2, "");

  auto pos = F_PositionRel()
             .setOrder(1)
             .setDiffInsteadOfVel()
             .eval(F);
  auto quat = F_QuaternionRel()
              .setOrder(1)
              .setDiffInsteadOfVel()
              .eval(F);

  y.setBlockVector(pos.y, quat.y);
  J.setBlockMatrix(pos.J, quat.J);
}
