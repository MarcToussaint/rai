/*  ------------------------------------------------------------------
    Copyright (c) 2011-2020 Marc Toussaint
    email: toussaint@tu-berlin.de

    This code is distributed under the MIT License.
    Please see <root-path>/LICENSE for details.
    --------------------------------------------------------------  */

#include "F_dynamics.h"
#include "frame.h"
#include "forceExchange.h"
#include "TM_default.h"
#include "TM_angVel.h"
#include "F_PairCollision.h"
#include "F_static.h"
#include "F_pose.h"

void shapeFunction(double& x, double& dx);

//===========================================================================

void F_NewtonEuler::phi2(arr& y, arr& J, const FrameL& F) {
  CHECK_EQ(order, 2, "");
  CHECK_EQ(F.d0, 3, "");
  CHECK_EQ(F.d1, 1, "");

  rai::Frame* a = F.elem(-2);
//  if((a->flags & (1<<FL_impulseExchange))){
//    y.resize(3).setZero();
//    if(!!J) J.resize(3, getKtupleDim(Ktuple).last()).setZero();
//    return;
//  }

  //get linear and angular accelerations
  TM_LinAngVel pos;
  pos.order=2;
  pos.impulseInsteadOfAcceleration=true;
  pos.phi2(y, J, F);

  //add gravity
  rai::Frame *r = F.elem(-1)->getRoot();
  if(r->C.hasTauJoint(r)) {
    double tau; arr Jtau;
    r->C.kinematicsTau(tau, Jtau, r);
    y(2) += gravity*tau;
    if(!!J){
      if(!J.isSparse()) J[2] += gravity*Jtau;
      else J.setMatrixBlock(gravity*Jtau, 2, 0);
    }
  } else {
    y(2) += gravity * r->C.frames.first()->tau;
  }

  //collect mass info (assume diagonal inertia matrix!!)
  double mass=1.;
  arr Imatrix = diag(.1, 3);
  if(a->inertia) {
    mass = a->inertia->mass;
    Imatrix = 2.*conv_mat2arr(a->inertia->matrix);
    //      rai::Quaternion &rot = f->X.rot;
    //      I=(rot).getMatrix() * f->inertia->matrix * (-rot).getMatrix();
  }
  arr one_over_mass(6);
  for(uint i=0; i<3; i++) one_over_mass(i) = 1./mass;
  for(uint i=0; i<3; i++) one_over_mass(i+3) = 1./Imatrix(i, i);
  double forceScaling = 1e1;
  one_over_mass *= forceScaling;

  //collect total contact forces
  Value fo = F_netForce(false, true)
            .eval({a}); // ! THIS IS THE MID TIME SLICE !

  y += one_over_mass % fo.y;
  if(!!J) J += one_over_mass % fo.J;
}

//===========================================================================

void F_NewtonEuler_DampedVelocities::phi2(arr& y, arr& J, const FrameL& F) {
  CHECK_EQ(order, 1, "");

  //get linear and angular velocities
  TM_LinAngVel pos;
  pos.order=1;
  pos.phi2(y, J, F);

  double friction=1.;
  y *= friction;
  if(!!J) J *= friction;

  //add gravity
  if(gravity) {
    rai::Frame *r = F.elem(-1)->getRoot();
    if(r->C.hasTauJoint(r)) {
      double tau; arr Jtau;
      r->C.kinematicsTau(tau, Jtau, r);
      y(2) += gravity*tau;
      if(!!J){
        if(!J.isSparse()) J[2] += gravity*Jtau;
        else J.setMatrixBlock(gravity*Jtau, 2, 0);
      }
    } else {
      y(2) += gravity * F(-1)->C.frames.first()->tau;
    }
  }

  //collect mass info (assume diagonal inertia matrix!!)
  double mass=1.;
  arr Imatrix = diag(.03, 3);
  rai::Frame* a = F.elem(-2);
  if(a->inertia) {
    mass = a->inertia->mass;
    Imatrix = 2.*conv_mat2arr(a->inertia->matrix);
  }
  arr one_over_mass(6);
  for(uint i=0; i<3; i++) one_over_mass(i) = 1./mass;
  for(uint i=0; i<3; i++) one_over_mass(i+3) = 1./Imatrix(i, i);
  double forceScaling = 1e1;
  one_over_mass *= forceScaling;

  //collect total contact forces
  Value fo = F_netForce(false, true)
            .eval({a});

  y += one_over_mass % fo.y;
  if(!!J) J += one_over_mass % fo.J;

  if(onlyXYPhi) {
    y({2, 4}).setZero();
    if(!!J) J({2, 4}).setZero();
  }
}

//===========================================================================

F_Wrench::F_Wrench(const arr& _vec, bool _torqueOnly) : vec(_vec), torqueOnly(_torqueOnly) {
  order=2;
  gravity = rai::getParameter<double>("TM_Wrench/gravity", 9.81);
}

void F_Wrench::phi2(arr& y, arr& J, const FrameL& F) {
  CHECK_EQ(order, 2, "");

  //get linear acceleration
  arr acc, Jacc;
  TM_LinVel pos;
  pos.order=2;
  pos.impulseInsteadOfAcceleration=false;
  pos.phi2(acc, Jacc, F);

  acc(2) += gravity;

  //get relative vector
  arr v, Jv;
  F(-2)->C.kinematicsVec(v, Jv, F(-2), vec);

  //torque
  arr torque = crossProduct(v, acc);
  arr Jtorque;
  if(!!J) Jtorque = skew(v) * Jacc - skew(acc) * Jv;

  //compose
  if(torqueOnly) {
    y = torque;
    if(!!J) J = Jtorque;
  } else {
    NIY
  }
}

//===========================================================================

F_Energy::F_Energy() {
  order=1;
  gravity = rai::getParameter<double>("TM_Physics/gravity", 9.81);
}

void F_Energy::phi2(arr& y, arr& J, const FrameL& F) {
  if(order==2){
    diffInsteadOfVel=true;
    Feature::phi2(y, J, F);
    diffInsteadOfVel=false;
    return;
  }

  CHECK_EQ(order, 1, "");

  double E=0.;
  Value p, v, w;

  F.elem(-1)->C.kinematicsZero(y, J, 1);

  arr grav = {0.,0.,gravity};

  for(uint i=0;i<F.d1;i++) {
    double mass=1.;
    arr Imatrix = diag(.1, 3);
    rai::Frame *a = F(1,i);
    if(a->inertia) {
      mass = a->inertia->mass;
      Imatrix = 2.*conv_mat2arr(a->inertia->matrix);
      //      rai::Quaternion &rot = f->X.rot;
      //      I=(rot).getMatrix() * f->inertia->matrix * (-rot).getMatrix();
    }

    p = F_Position()
        .eval({F(1,i)});

    v = F_Position()
        .setOrder(1)
        .eval({F(0,i), F(1,i)});

//    w = TM_AngVel()
//        .eval({F(0,i), F(1,i)});

    E += .5*mass*sumOfSqr(v.y);
    E += mass * scalarProduct(grav,p.y); //p(2)=height //(a->X*a->inertia->com).z;
//      E += .5*m*sumOfSqr(w); //(w*(I*w));

    if(!!J) {
      J += (mass*~v.y) * v.J;
      J += (mass*~grav) * p.J;
    }
  }

  y = ARR(E);
}

//===========================================================================

FrameL getShapesAbove(rai::Frame* a) {
  FrameL aboves;
  if(a->shape) aboves.append(a);
  for(rai::Frame* b:a->children) aboves.append(getShapesAbove(b));
  return aboves;
}

void F_StaticStability::phi2(arr& y, arr& J, const FrameL& F) {
  NIY;
#if 0
  //get shapes above
  rai::Frame* a = K.frames(i);
  FrameL aboves = getShapesAbove(a);
//  cout <<"ABOVES="<<endl; listWrite(aboves);

  //get average center of all shapes
  arr cog(3), J_cog(3, K.getJointStateDimension());
  cog.setZero(); J_cog.setZero();
  double M=0.;
  for(rai::Frame* b:aboves) if(b!=a) {
      double mass=0.;
      if(b->shape) mass=1.;
      if(b->inertia) mass=b->inertia->mass;
      arr y, J;
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
  y = (y-cog)({0, 1});
  if(!!J) J=(J-J_cog)({0, 1});

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
#endif
}
