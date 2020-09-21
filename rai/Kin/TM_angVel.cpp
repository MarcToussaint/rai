/*  ------------------------------------------------------------------
    Copyright (c) 2011-2020 Marc Toussaint
    email: toussaint@tu-berlin.de

    This code is distributed under the MIT License.
    Please see <root-path>/LICENSE for details.
    --------------------------------------------------------------  */

#include "TM_angVel.h"
#include "F_pose.h"
#include "../Geo/geo.h"

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

void TM_LinVel::phi2(arr& y, arr& J, const FrameL& F) {
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

void TM_AngVel::phi2(arr& y, arr& J, const FrameL& F){
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

void TM_LinAngVel::phi2(arr& y, arr& J, const FrameL& F) {

  TM_LinVel lin;
  lin.order=order;
  lin.impulseInsteadOfAcceleration = impulseInsteadOfAcceleration;
  arr  yl, Jl;
  lin.__phi2(yl, Jl, F);

  TM_AngVel ang;
  ang.order=order;
  ang.impulseInsteadOfAcceleration = impulseInsteadOfAcceleration;
  arr ya, Ja;
  ang.__phi2(ya, Ja, F);

  y.setBlockVector(yl, ya);
  J.setBlockMatrix(Jl, Ja);
}

//===========================================================================

void TM_NoJumpFromParent::phi2(arr& y, arr& J, const FrameL& F) {
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
