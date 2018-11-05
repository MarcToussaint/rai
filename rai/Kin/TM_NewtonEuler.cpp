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


TM_NewtonEuler::TM_NewtonEuler(int iShape, bool _transOnly) : i(iShape), transOnly(_transOnly) {
  order=2;
  gravity = rai::getParameter<double>("TM_NewtonEuler/gravity", 9.81);
}

void TM_NewtonEuler::phi(arr &y, arr &J, const WorldL &Ktuple) {
  CHECK_EQ(order, 2, "");

  //get linear and angular accelerations
  arr acc, Jacc, wcc, Jwcc;
  TM_Default pos(TMT_posDiff, i);
  pos.order=2;
  pos.Feature::phi(acc, (!!J?Jacc:NoArr), Ktuple);
  acc(2) += gravity;

  TM_AngVel rot(i);
  rot.order=2;
  if(!transOnly) rot.phi(wcc, (!!J?Jwcc:NoArr), Ktuple);

  rai::KinematicWorld& K = *Ktuple(-2); // ! THIS IS THE MID TIME SLICE !
  rai::Frame *a = K.frames(i);
  double mass=1;
  arr Imatrix = diag(.1, 3);
  if(a->inertia){
    mass = a->inertia->mass;
    Imatrix = 10.*conv_mat2arr(a->inertia->matrix);
  }

  mass = 1./mass;
  Imatrix = inverse_SymPosDef(Imatrix);
  double forceScaling = 1e2;

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

    acc -= sign * forceScaling *mass* con->force;
    if(!transOnly) wcc += sign * forceScaling *Imatrix* crossProduct(cp-p, con->force);

    if(!!J){
      Jacc -= sign * forceScaling *mass* Jf;
      if(!transOnly) Jwcc += sign * forceScaling *Imatrix* (skew(cp-p) * Jf - skew(con->force) * (Jcp-Jp));
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

