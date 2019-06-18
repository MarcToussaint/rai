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
#include <Kin/F_static.h>

void shapeFunction(double &x, double &dx);

//===========================================================================

void TM_NewtonEuler::phi(arr &y, arr &J, const WorldL &Ktuple) {
  CHECK_EQ(order, 2, "");

  rai::Frame *a = Ktuple(-2)->frames(i);
  if((a->flags & (1<<FL_impulseExchange))){
    y.resize(3).setZero();
    if(!!J) J.resize(3, getKtupleDim(Ktuple).last()).setZero();
    return;
  }

  //get linear and angular accelerations
  TM_LinAngVel pos(i);
  pos.order=2;
  pos.impulseInsteadOfAcceleration=true;
  pos.phi(y, J, Ktuple);

  //add gravity
  if(Ktuple(-1)->hasTimeJoint()){
    double tau; arr Jtau;
    Ktuple(-1)->kinematicsTau(tau, Jtau);
    y(2) += gravity*tau;
    if(!!J){
      expandJacobian(Jtau, Ktuple, -1);
      J[2] += gravity*Jtau;
    }
  }else{
    y(2) += gravity * Ktuple(-1)->frames.first()->tau;
  }

  //collect mass info (assume diagonal inertia matrix!!)
  double mass=1;
  arr Imatrix = diag(.1, 3);
  if(a->inertia){
    mass = a->inertia->mass;
    Imatrix = 2.*conv_mat2arr(a->inertia->matrix);
  }
  arr one_over_mass(6);
  for(uint i=0;i<3;i++) one_over_mass(i) = 1./mass;
  for(uint i=0;i<3;i++) one_over_mass(i+3) = 1./Imatrix(i,i);
  double forceScaling = 1e1;
  one_over_mass *= forceScaling;

  //collect total contact forces
  Value F = F_netForce(a->ID, false, true)(*Ktuple(-1)); // ! THIS IS THE MID TIME SLICE !
  if(!!J) expandJacobian(F.J, Ktuple, -1);

  y += one_over_mass % F.y;
  if(!!J) J += one_over_mass % F.J;
}

//===========================================================================

void TM_NewtonEuler_DampedVelocities::phi(arr &y, arr &J, const WorldL &Ktuple) {
  CHECK_EQ(order, 1, "");

  //get linear and angular accelerations
  TM_LinAngVel pos(i);
  pos.order=1;
  pos.phi(y, J, Ktuple);

  //add gravity
  if(Ktuple(-1)->hasTimeJoint()){
    double tau; arr Jtau;
    Ktuple(-1)->kinematicsTau(tau, Jtau);
    y(2) += gravity*tau;
    if(!!J){
      expandJacobian(Jtau, Ktuple, -1);
      J[2] += gravity*Jtau;
    }
  }else{
    y(2) += gravity * Ktuple(-1)->frames.first()->tau;
  }

  //collect mass info (assume diagonal inertia matrix!!)
  double mass=1;
  arr Imatrix = diag(.1, 3);
  rai::Frame *a = Ktuple(-2)->frames(i);
  if(a->inertia){
    mass = a->inertia->mass;
    Imatrix = 2.*conv_mat2arr(a->inertia->matrix);
  }
  arr one_over_mass(6);
  for(uint i=0;i<3;i++) one_over_mass(i) = 1./mass;
  for(uint i=0;i<3;i++) one_over_mass(i+3) = 1./Imatrix(i,i);
  double forceScaling = 1e1;
  one_over_mass *= forceScaling;

  //collect total contact forces
  Value F = F_netForce(a->ID, false, true)(*Ktuple(-1));
  if(!!J) expandJacobian(F.J, Ktuple, -1);

  y += one_over_mass % F.y;
  if(!!J) J += one_over_mass % F.J;
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

