/*  ------------------------------------------------------------------
    Copyright (c) 2017 Marc Toussaint
    email: marc.toussaint@informatik.uni-stuttgart.de

    This code is distributed under the MIT License.
    Please see <root-path>/LICENSE for details.
    --------------------------------------------------------------  */

#include "TM_NewtonEuler.h"
#include <Kin/flag.h>
#include <Kin/frame.h>
#include <Kin/contact.h>
#include <Kin/TM_default.h>
#include <Kin/TM_angVel.h>
#include <Kin/TM_PairCollision.h>

void shapeFunction(double &x, double &dx);

//===========================================================================

TM_NewtonEuler::TM_NewtonEuler(int iShape, bool _transOnly) : i(iShape), transOnly(_transOnly) {
  order=2;
  gravity = rai::getParameter<double>("TM_NewtonEuler/gravity", 9.81);
}

void TM_NewtonEuler::phi(arr &y, arr &J, const WorldL &Ktuple) {
  CHECK_EQ(order, 2, "");

  rai::KinematicWorld& K = *Ktuple(-2); // ! THIS IS THE MID TIME SLICE !
  rai::Frame *a = K.frames(i);
  if((a->flags & (1<<FL_impulseExchange))){
    y.resize(3).setZero();
    if(!!J) J.resize(3, getKtupleDim(Ktuple).last()).setZero();
    return;
  }

  //get linear and angular accelerations
  arr acc, Jacc, wcc, Jwcc;
  TM_LinVel pos(i);
  pos.order=2;
  pos.impulseInsteadOfAcceleration=true;
  pos.phi(acc, (!!J?Jacc:NoArr), Ktuple);

  if(Ktuple(-1)->hasTimeJoint()){
    double tau; arr Jtau;
    Ktuple(-1)->kinematicsTau(tau, Jtau);
    acc(2) += gravity*tau;
    if(!!J){
      expandJacobian(Jtau, Ktuple, -1);
      Jacc[2] += gravity*Jtau;
    }
  }else{
    acc(2) += gravity * Ktuple(-1)->frames.first()->tau;
  }

  TM_AngVel rot(i);
  rot.order=2;
  rot.impulseInsteadOfAcceleration=true;
  if(!transOnly) rot.phi(wcc, (!!J?Jwcc:NoArr), Ktuple);

//  rai::KinematicWorld& K = *Ktuple(-2); // ! THIS IS THE MID TIME SLICE !
//  rai::Frame *a = K.frames(i);
  double mass=1;
  arr Imatrix = diag(.1, 3);
  if(a->inertia){
    mass = a->inertia->mass;
    Imatrix = 2.*conv_mat2arr(a->inertia->matrix);
  }

  mass = 1./mass;
  Imatrix = inverse_SymPosDef(Imatrix);
  double forceScaling = 1e1;

  for(rai::Contact *con:a->contacts){
    double sign = +1.;
    CHECK(&con->a==a || &con->b==a, "");
    if(&con->b==a) sign=-1.;

    //get the force
    arr f, Jf;
    K.kinematicsContactForce(f, Jf, con);
    if(!!J) expandJacobian(Jf, Ktuple, -2);

    //get the POA
    arr cp, Jcp;
    K.kinematicsContactPOA(cp, Jcp, con);
    if(!!J) expandJacobian(Jcp, Ktuple, -2);

    //get object center
    arr p, Jp;
    K.kinematicsPos(p, Jp, a);
    if(!!J) expandJacobian(Jp, Ktuple, -2);

    acc -= sign * forceScaling * mass * f;
    if(!transOnly) wcc += sign * forceScaling * Imatrix * crossProduct(cp-p, f);

    if(!!J){
      Jacc -= sign * forceScaling *mass* Jf;
      if(!transOnly) Jwcc += sign * forceScaling * Imatrix * (skew(cp-p) * Jf - skew(f) * (Jcp-Jp));
    }
  }

  if(!transOnly) y.resize(6).setZero();
  else y.resize(3).setZero();
  y.setVectorBlock(acc, 0);
  if(!transOnly) y.setVectorBlock(wcc, 3);

  if(!!J) {
    J.resize(y.N, Jacc.d1).setZero();
    J.setMatrixBlock(Jacc, 0, 0);
    if(!transOnly) J.setMatrixBlock(Jwcc, 3, 0);
  }
}

uint TM_NewtonEuler::dim_phi(const WorldL& Ktuple){
  if(transOnly) return 3;
  return 6;
}


//===========================================================================


TM_Wrench::TM_Wrench(int iShape, const arr& _vec, bool _torqueOnly) : i(iShape), vec(_vec), torqueOnly(_torqueOnly) {
  order=2;
  gravity = rai::getParameter<double>("TM_Wrench/gravity", 9.81);
}

void TM_Wrench::phi(arr &y, arr &J, const WorldL &Ktuple) {
  CHECK_EQ(order, 2, "");

  rai::KinematicWorld& K = *Ktuple(-2); // ! THIS IS THE MID TIME SLICE !
  rai::Frame *a = K.frames(i);
  if((a->flags & (1<<FL_impulseExchange))){
    y.resize(3).setZero();
    if(!!J) J.resize(3, getKtupleDim(Ktuple).last()).setZero();
    return;
  }

  //get linear acceleration
  arr acc, Jacc;
  TM_LinVel pos(i);
  pos.order=2;
  pos.impulseInsteadOfAcceleration=false;
  pos.phi(acc, (!!J?Jacc:NoArr), Ktuple);

  acc(2) += gravity;

  //get relative vector
  arr v, Jv;
  K.kinematicsVec(v, Jv, K.frames(i), vec);
  if(!!J) expandJacobian(Jv, Ktuple, -2);

  //torque
  arr torque = crossProduct(v, acc);
  arr Jtorque;
  if(!!J) Jtorque = skew(v) * Jacc - skew(acc) * Jv;

  //compose
  if(torqueOnly){
    y = torque;
    if(!!J) J = Jtorque;
  }else{
    NIY
  }
}

uint TM_Wrench::dim_phi(const WorldL& Ktuple){
  if(torqueOnly) return 3;
  return 6;
}

