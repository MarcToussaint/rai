/*  ------------------------------------------------------------------
    Copyright (c) 2019 Marc Toussaint
    email: marc.toussaint@informatik.uni-stuttgart.de

    This code is distributed under the MIT License.
    Please see <root-path>/LICENSE for details.
    --------------------------------------------------------------  */

#include "F_static.h"
#include "contact.h"

F_netForce::F_netForce(int iShape, bool _transOnly, bool _zeroGravity) : i(iShape), transOnly(_transOnly) {
  order=0;
  if(_zeroGravity) {
    gravity = 0.;
  } else {
    gravity = rai::getParameter<double>("F_static/gravity", 9.81);
  }
}

void F_netForce::phi(arr& y, arr& J, const rai::Configuration& C) {
  rai::Frame* a = C.frames(i);

  arr force = zeros(3);
  arr torque = zeros(3);
  arr Jforce, Jtorque;
  if(!!J) {
    uint n = C.getJointStateDimension();
    if(!C.useSparseJacobians) {
      Jforce.resize(3, n).setZero();
      Jtorque.resize(3, n).setZero();
    } else {
      Jforce.sparse().resize(3, n, 0);
      Jtorque.sparse().resize(3, n, 0);
    }
  }

  if(gravity) {
    double mass=.1;
    if(a->inertia) mass = a->inertia->mass;
    force(2) += gravity * mass;
  }

  //-- collect contacts and signs FOR ALL shapes attached to this link
  rai::Array<rai::Contact*> contacts;
  arr signs;
  FrameL F;
  F.append(a);
  a->getRigidSubFrames(F);
  for(rai::Frame* f:F) {
    for(rai::Contact* con:f->contacts) {
      CHECK(&con->a==f || &con->b==f, "");
      contacts.append(con);
      signs.append((&con->a==f ? +1. : -1.));
    }
  }

#if 0
  for(rai::Contact* con:a->contacts) {
    double sign = +1.;
    CHECK(&con->a==a || &con->b==a, "");
    if(&con->b==a) sign=-1.;
#else
  for(uint i=0; i<contacts.N; i++) {
    rai::Contact* con = contacts(i);
    double sign = signs(i);
#endif

    //get the force
    arr f, Jf;
    C.kinematicsContactForce(f, Jf, con);

    //get the POA
    arr poa, Jpoa;
    C.kinematicsContactPOA(poa, Jpoa, con);

    //get object center
    arr p, Jp;
    C.kinematicsPos(p, Jp, a);

    force -= sign * con->force;
    if(!transOnly) torque += sign * crossProduct(poa-p, con->force);

    if(!!J) {
      Jforce -= sign * Jf;
      if(!transOnly) Jtorque += sign * (skew(poa-p) * Jf - skew(con->force) * (Jpoa-Jp));
    }
  }

  if(!transOnly) y.resize(6).setZero();
  else y.resize(3).setZero();
  y.setVectorBlock(force, 0);
  if(!transOnly) y.setVectorBlock(torque, 3);

  if(!!J) {
    if(!C.useSparseJacobians) {
      J.resize(y.N, Jforce.d1).setZero();
      J.setMatrixBlock(Jforce, 0, 0);
      if(!transOnly) J.setMatrixBlock(Jtorque, 3, 0);
    } else {
      J.sparse().resize(y.N, Jforce.d1, 0);
      Jforce.sparse().reshape(6, Jtorque.d1);
      J += Jforce;
      Jtorque.sparse().reshape(6, Jtorque.d1);
      Jtorque.sparse().colShift(3);
      J += Jtorque;
    }
  }
}

uint F_netForce::dim_phi(const rai::Configuration& K) {
  if(transOnly) return 3;
  return 6;
}

