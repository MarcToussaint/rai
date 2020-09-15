/*  ------------------------------------------------------------------
    Copyright (c) 2011-2020 Marc Toussaint
    email: toussaint@tu-berlin.de

    This code is distributed under the MIT License.
    Please see <root-path>/LICENSE for details.
    --------------------------------------------------------------  */

#include "TM_angVel.h"
#include "TM_default.h"
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

void TM_LinVel::phi2(arr& y, arr& J, const FrameL& F){
  if(order>1){  Feature::phi2(y, J, F);  return;  }
  CHECK_EQ(order, 1, "");
  CHECK_EQ(F.d0, 2, "");
  CHECK_EQ(F.d1, 1, "");
  rai::Frame *f0 = F(0,0), *f1 = F(1,0);

  CHECK_EQ(&f0->C, &f1->C, "");
  arr a, b, Ja, Jb;
  f0->C.kinematicsPos(a, Ja, f0);
  f1->C.kinematicsPos(b, Jb, f1);
  y = a-b;
  J = Ja-Jb;

  double tau = f0->C.frames(0)->tau;
  CHECK_GE(tau, 1e-10, "");
  y /= tau;
  J /= tau;
}

void TM_LinVel::phi(arr& y, arr& J, const ConfigurationL& Ktuple) {
  if(order==1) {
    rai::Frame* f0 = Ktuple(-2)->frames(i);
    rai::Frame* f1 = Ktuple(-1)->frames(i);

    arr a, b, Ja, Jb;
    Ktuple(-2)->kinematicsPos(a, Ja, f0);
    Ktuple(-1)->kinematicsPos(b, Jb, f1);

    y = b-a;
    if(!!J && !!Ja && !!Jb) {
      expandJacobian(Ja, Ktuple, -2);
      expandJacobian(Jb, Ktuple, -1);
      J = Jb-Ja;
    }else J.setNoArr();

#if 1
    if(Ktuple(-1)->hasTauJoint()) {
      double tau; arr Jtau;
      Ktuple(-1)->kinematicsTau(tau, Jtau);
      CHECK_GE(tau, 1e-10, "");
      y /= tau;
      if(!!J) {
        J /= tau;
        expandJacobian(Jtau, Ktuple, -1);
        J += (-1./tau)*y*Jtau;
      }
    } else {
      double tau = Ktuple(-1)->frames(0)->tau;
      CHECK_GE(tau, 1e-10, "");
      y /= tau;
      if(!!J) J /= tau;
    }
#endif
    return;
  }

  if(order==2) {
    arr y0, y1, Jy0, Jy1;
    order--;
    phi(y0, Jy0, Ktuple({0, -2}));  if(!!Jy0) padJacobian(Jy0, Ktuple);
    phi(y1, Jy1, Ktuple);
    order++;

    double tau = Ktuple(-2)->frames(0)->tau;
    if(impulseInsteadOfAcceleration) tau=1.;
    y = (y1-y0)/tau; //difference!
    if(!!Jy0) J = (Jy1-Jy0)/tau;
    else J.setNoArr();
  }
}

//===========================================================================

void TM_AngVel::phi2(arr& y, arr& J, const FrameL& F){
  if(order>1){  Feature::phi2(y, J, F);  return;  }
  CHECK_EQ(order, 1, "");
  CHECK_EQ(F.d0, 2, "");
  CHECK_EQ(F.d1, 1, "");
  rai::Frame *f0 = F(0,0), *f1 = F(1,0);

  CHECK_EQ(&f0->C, &f1->C, "");
  angVel_base(f0, f1, y, J);

  double tau = f0->C.frames(0)->tau;
  CHECK_GE(tau, 1e-10, "");
  y /= tau;
  J /= tau;
}

void TM_AngVel::phi(arr& y, arr& J, const ConfigurationL& Ktuple) {
  if(order==1) {
    arr J_tmp;
    angVel_base(Ktuple(-2)->frames(i), Ktuple(-1)->frames(i), y, J_tmp);

    if(!!J_tmp) {
      if(Ktuple.N==3){
        uint shift = Ktuple(-3)->q.N;
        if(!J_tmp.isSparse()) {
          J = catCol(zeros(y.N, shift), J_tmp);
        } else {
          J = J_tmp;
          J.sparse().reshape(J.d0, J.d1+shift);
          J.sparse().rowShift(shift);
        }
      } else{
        J=J_tmp;
      }
    }else J.setNoArr();

#if 1
    if(Ktuple(-1)->hasTauJoint()) {
      double tau; arr Jtau;
      Ktuple(-1)->kinematicsTau(tau, Jtau);
      CHECK_GE(tau, 1e-10, "");

      y /= tau;
      if(!!J) {
        J /= tau;
        expandJacobian(Jtau, Ktuple, -1);
        J += (-1./tau)*y*Jtau;
      }
    } else {
      double tau = Ktuple(-1)->frames(0)->tau;
      CHECK_GE(tau, 1e-10, "");
      y /= tau;
      if(!!J) J /= tau;
    }
#endif
    return;
  }

  if(order==2) {
    arr y0, y1, Jy0, Jy1;
    order--;
    phi(y0, Jy0, Ktuple({0, -2}));  if(!!Jy0) padJacobian(Jy0, Ktuple);
    phi(y1, Jy1, Ktuple);
    order++;

    double tau = Ktuple(-2)->frames(0)->tau;
    if(impulseInsteadOfAcceleration) tau=1.;
    y = (y1-y0)/tau; //difference!
    if(!!J && !!Jy0) J = (Jy1 - Jy0)/tau;
    else J.setNoArr();
  }
}

uint TM_AngVel::dim_phi(const rai::Configuration& G) { return 3; }

//===========================================================================

void TM_LinAngVel::phi(arr& y, arr& J, const ConfigurationL& Ctuple) {
  TM_LinVel lin(frameIDs.scalar());
  lin.order=order;
  lin.impulseInsteadOfAcceleration = impulseInsteadOfAcceleration;
  Value _lin = lin.eval(Ctuple);

  TM_AngVel ang(frameIDs.scalar());
  ang.order=order;
  ang.impulseInsteadOfAcceleration = impulseInsteadOfAcceleration;
  Value _ang = ang.eval(Ctuple);

  y.setBlockVector(_lin.y, _ang.y);
  J.setBlockMatrix(_lin.J, _ang.J);
}


void TM_LinAngVel::phi2(arr& y, arr& J, const FrameL& F) {

  TM_LinVel lin(-1);
  lin.order=order;
  lin.impulseInsteadOfAcceleration = impulseInsteadOfAcceleration;
  arr  yl, Jl;
  lin.__phi2(yl, Jl, F);

  TM_AngVel ang(-1);
  ang.order=order;
  ang.impulseInsteadOfAcceleration = impulseInsteadOfAcceleration;
  arr ya, Ja;
  ang.__phi2(ya, Ja, F);

  y.setBlockVector(yl, ya);
  J.setBlockMatrix(Jl, Ja);
}

uint TM_LinAngVel::dim_phi(const rai::Configuration& G) { return 6; }

//===========================================================================

void TM_NoJumpFromParent::phi(arr& y, arr& J, const ConfigurationL& Ktuple) {
  rai::Frame* obj = Ktuple.elem(-2)->frames(i);
  rai::Frame* link = obj->getUpwardLink();
  rai::Frame* parent = link->parent;

  if(parent && parent->ID == Ktuple.elem(-1)->frames(i)->getUpwardLink()->parent->ID) {
#if 1
    LOG(-1) <<"this frame isn't switching - are you sure you want to do this?";
#else
    y.resize(7).setZero();
    if(!!J) J.resize(7, getKtupleDim(Ktuple).last()).setZero();
    return;
#endif
  }

//  LOG(0) <<"link:" <<link->name <<" parent:" <<parent->name;
  
  {
//  if(link->joint && link->joint->type==rai::JT_rigid){
    arr yp, Jp, yq, Jq;
    ptr<TM_Default> tmp;
    if(parent)
      tmp = make_shared<TM_Default>(TMT_pos, link->ID, NoVector, parent->ID, NoVector);
    else{
      LOG(-1) <<"this frame has no parent?";
      tmp = make_shared<TM_Default>(TMT_pos, link->ID);
    }
    tmp->order = 1;
    tmp->type = TMT_pos;
    tmp->Feature::__phi(yp, Jp, Ktuple);
    tmp->type = TMT_quat;
    tmp->flipTargetSignOnNegScalarProduct=true;
    tmp->Feature::__phi(yq, Jq, Ktuple);

    y.setBlockVector(yp, yq);
    J.setBlockMatrix(Jp, Jq);
  }
//  else{
//    y.resize(7).setZero();
//    if(!!J) J.resize(7,getKtupleDim(Ktuple).last()).setZero();
//  }
}

uint TM_NoJumpFromParent::dim_phi(const rai::Configuration& G) {
  return 7;
}
