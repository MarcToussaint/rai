/*  ------------------------------------------------------------------
    Copyright (c) 2017 Marc Toussaint
    email: marc.toussaint@informatik.uni-stuttgart.de

    This code is distributed under the MIT License.
    Please see <root-path>/LICENSE for details.
    --------------------------------------------------------------  */

#include "F_static.h"
#include "contact.h"

F_static::F_static(int iShape, bool _transOnly) : i(iShape), transOnly(_transOnly) {
  order=0;
  gravity = rai::getParameter<double>("F_static/gravity", 9.81);
}

void F_static::phi(arr &y, arr &J, const rai::KinematicWorld& K) {
  rai::Frame *a = K.frames(i);

  arr force = zeros(3);
  arr torque = zeros(3);
  arr Jforce, Jtorque;
  if(!!J){
    Jforce = Jtorque = zeros(3, K.getJointStateDimension());
  }

  double mass=.1;
  if(a->inertia) mass = a->inertia->mass;
  force(2) += gravity * mass;


  for(rai::Contact *con:a->contacts){
    double sign = +1.;
    CHECK(&con->a==a || &con->b==a, "");
    if(&con->b==a) sign=-1.;

    //get the force
    arr f, Jf;
    K.kinematicsContactForce(f, Jf, con);

    //get the POA
    arr cp, Jcp;
    K.kinematicsContactPOA(cp, Jcp, con);

    //get object center
    arr p, Jp;
    K.kinematicsPos(p, Jp, a);

    force -= sign * con->force;
    if(!transOnly) torque += sign * crossProduct(cp-p, con->force);

    if(!!J){
      Jforce -= sign * Jf;
      if(!transOnly) Jtorque += sign * (skew(cp-p) * Jf - skew(con->force) * (Jcp-Jp));
    }
  }

  if(!transOnly) y.resize(6).setZero();
  else y.resize(3).setZero();
  y.setVectorBlock(force, 0);
  if(!transOnly) y.setVectorBlock(torque, 3);

  if(!!J) {
    J.resize(y.N, Jforce.d1).setZero();
    J.setMatrixBlock(Jforce, 0, 0);
    if(!transOnly) J.setMatrixBlock(Jtorque, 3, 0);
  }
}

uint F_static::dim_phi(const rai::KinematicWorld& K){
  if(transOnly) return 3;
  return 6;
}

