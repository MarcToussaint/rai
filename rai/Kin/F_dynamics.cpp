/*  ------------------------------------------------------------------
    Copyright (c) 2017 Marc Toussaint
    email: marc.toussaint@informatik.uni-stuttgart.de

    This code is distributed under the MIT License.
    Please see <root-path>/LICENSE for details.
    --------------------------------------------------------------  */

#include "F_dynamics.h"
#include "frame.h"
#include "contact.h"
#include "TM_default.h"
#include "TM_angVel.h"
#include "F_PairCollision.h"
#include "F_static.h"

void shapeFunction(double &x, double &dx);

//===========================================================================

void F_NewtonEuler::phi(arr &y, arr &J, const WorldL &Ktuple) {
  CHECK_EQ(order, 2, "");

  rai::Frame *a = Ktuple(-2)->frames(i);
//  if((a->flags & (1<<FL_impulseExchange))){
//    y.resize(3).setZero();
//    if(!!J) J.resize(3, getKtupleDim(Ktuple).last()).setZero();
//    return;
//  }

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
  double mass=1.;
  arr Imatrix = diag(.1, 3);
  if(a->inertia){
    mass = a->inertia->mass;
    Imatrix = 2.*conv_mat2arr(a->inertia->matrix);
    //      rai::Quaternion &rot = f->X.rot;
    //      I=(rot).getMatrix() * f->inertia->matrix * (-rot).getMatrix();
  }
  arr one_over_mass(6);
  for(uint i=0;i<3;i++) one_over_mass(i) = 1./mass;
  for(uint i=0;i<3;i++) one_over_mass(i+3) = 1./Imatrix(i,i);
  double forceScaling = 1e1;
  one_over_mass *= forceScaling;

  //collect total contact forces
  Value F = F_netForce(a->ID, false, true)(*Ktuple(-2)); // ! THIS IS THE MID TIME SLICE !
  if(!!J) expandJacobian(F.J, Ktuple, -2);

  y += one_over_mass % F.y;
  if(!!J) J += one_over_mass % F.J;
}

//===========================================================================

void F_NewtonEuler_DampedVelocities::phi(arr &y, arr &J, const WorldL &Ktuple) {
  CHECK_EQ(order, 1, "");

  //get linear and angular velocities
  TM_LinAngVel pos(i);
  pos.order=1;
  pos.phi(y, J, Ktuple);

  double friction=1.;
  y *= friction;
  if(!!J) J *= friction;

  //add gravity
  if(gravity){
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
  }

  //collect mass info (assume diagonal inertia matrix!!)
  double mass=1.;
  arr Imatrix = diag(.03, 3);
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
  Value F = F_netForce(a->ID, false, true)(Ktuple);

  y += one_over_mass % F.y;
  if(!!J) J += one_over_mass % F.J;

  if(onlyXYPhi){
    y({2,4}).setZero();
    if(!!J) J({2,4}).setZero();
  }
}

//===========================================================================


F_Wrench::F_Wrench(int iShape, const arr& _vec, bool _torqueOnly) : i(iShape), vec(_vec), torqueOnly(_torqueOnly) {
  order=2;
  gravity = rai::getParameter<double>("TM_Wrench/gravity", 9.81);
}

void F_Wrench::phi(arr &y, arr &J, const WorldL &Ktuple) {
  CHECK_EQ(order, 2, "");

  rai::KinematicWorld& K = *Ktuple(-2); // ! THIS IS THE MID TIME SLICE !
  rai::Frame *a = K.frames(i);
//  if((a->flags & (1<<FL_impulseExchange))){
//    y.resize(3).setZero();
//    if(!!J) J.resize(3, getKtupleDim(Ktuple).last()).setZero();
//    return;
//  }

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

uint F_Wrench::dim_phi(const WorldL& Ktuple){
  if(torqueOnly) return 3;
  return 6;
}


//===========================================================================

F_Energy::F_Energy() {
  order=1;
  gravity = rai::getParameter<double>("TM_Physics/gravity", 9.81);
}

void F_Energy::phi(arr &y, arr &J, const WorldL &Ktuple) {
  if(order==2){
    arr y0, y1, J0, J1;
    order=1;
    phi(y0, J0, Ktuple({-3,-2}));
    phi(y1, J1, Ktuple({-2,-1}));
    order=2;

    y = y1 - y0;
    if(!!J){
      uintA qdim = getKtupleDim(Ktuple);
      J.resize(y.N, qdim.last()).setZero();
      CHECK_EQ(J0.d1, qdim(1), "");
      CHECK_EQ(J1.d1, qdim(2)-qdim(0), "");
      for(uint i=0;i<y.N;i++){
        for(uint j=0;j<J0.d1;j++) J(i,j) -= J0(i,j);
        for(uint j=0;j<J1.d1;j++) J(i,qdim(0)+j) += J1(i,j);
      }
    }
    return;
  }

  CHECK_EQ(order, 1, "");

  rai::KinematicWorld& K = *Ktuple(-1);

  double E=0.;
  arr p, Jp, v, Jv, w, Jw;

  uintA qdim = getKtupleDim(Ktuple);
  if(!!J) J = zeros(1, qdim.last());

  for(rai::Frame *a:K.frames) {
    double mass=1.;
    arr Imatrix = diag(.1, 3);
    if(a->inertia){
      mass = a->inertia->mass;
      Imatrix = 2.*conv_mat2arr(a->inertia->matrix);
      //      rai::Quaternion &rot = f->X.rot;
      //      I=(rot).getMatrix() * f->inertia->matrix * (-rot).getMatrix();
    }

    TM_Default pos(TMT_pos, a->ID);
    pos.order=0;
    pos.Feature::__phi(p, (!!J?Jp:NoArr), Ktuple);

    pos.order=1;
    pos.Feature::__phi(v, (!!J?Jv:NoArr), Ktuple);

//      TM_AngVel rot(a->ID);
//      rot.order=1;
//      rot.phi(w, (!!J?Jw:NoArr), Ktuple);

    E += .5*mass*sumOfSqr(v);
    E += gravity * mass * p(2); //p(2)=height //(a->X*a->inertia->com).z;
//      E += .5*m*sumOfSqr(w); //(w*(I*w));

    if(!!J){
      J += (mass*~v) * Jv;
      J += (gravity*mass) * Jp[2];
    }
  }

  y = ARR(E);
}

uint F_Energy::dim_phi(const WorldL &Ktuple) {
  return 1;
}

//===========================================================================

F_StaticStability::F_StaticStability(int iShape, double _margin)
  : i(iShape), margin(_margin) {
}

F_StaticStability::F_StaticStability(const rai::KinematicWorld& G, const char* iShapeName, double _margin)
  :i(-1), margin(_margin) {
  rai::Frame *a = iShapeName ? G.getFrameByName(iShapeName):NULL;
  if(a) i=a->ID;
}

FrameL getShapesAbove(rai::Frame *a) {
  FrameL aboves;
  if(a->shape) aboves.append(a);
  for(rai::Frame *b:a->parentOf) aboves.append(getShapesAbove(b));
  return aboves;
}

void F_StaticStability::phi(arr& y, arr& J, const rai::KinematicWorld& K) {
  //get shapes above
  rai::Frame *a = K.frames(i);
  FrameL aboves = getShapesAbove(a);
//  cout <<"ABOVES="<<endl; listWrite(aboves);

  //get average center of all shapes
  arr cog(3) ,J_cog(3, K.getJointStateDimension());
  cog.setZero(); J_cog.setZero();
  double M=0.;
  for(rai::Frame *b:aboves) if(b!=a) {
      double mass=0.;
      if(b->shape) mass=1.;
      if(b->inertia) mass=b->inertia->mass;
      arr y,J;
      K.kinematicsPos(y, J, b);
      cog += mass*y;
      J_cog += mass*J;
      M += mass;
    }
  CHECK(M>0., "");
  cog  /= M;
  J_cog /= M;

  //align avg with object center
  K.kinematicsPos(y, J, a);
  y = (y-cog)({0,1});
  if(!!J) J=(J-J_cog)({0,1});

#if 1
  CHECK(a->shape, "");
  CHECK_EQ(a->shape->type(), rai::ST_ssBox, "the supporting shape needs to be a box");
  arr range = { .5*a->shape->size(0)-margin, .5*a->shape->size(1)-margin };
  arr pos=y, posJ=J;

  y.resize(4);
  y(0) =  pos(0) - range(0);
  y(1) = -pos(0) - range(0);
  y(2) =  pos(1) - range(1);
  y(3) = -pos(1) - range(1);
  if(!!J) {
    J.resize(4, posJ.d1);
    J[0] =  posJ[0];
    J[1] = -posJ[0];
    J[2] =  posJ[1];
    J[3] = -posJ[1];
  }
#endif
}

rai::String F_StaticStability::shortTag(const rai::KinematicWorld &K) {
  return STRING("StaticStability:"<<(i<0?"WORLD":K.frames(i)->name));
}
