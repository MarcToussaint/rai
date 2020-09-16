/*  ------------------------------------------------------------------
    Copyright (c) 2011-2020 Marc Toussaint
    email: toussaint@tu-berlin.de

    This code is distributed under the MIT License.
    Please see <root-path>/LICENSE for details.
    --------------------------------------------------------------  */

#include "F_static.h"
#include "forceExchange.h"

F_netForce::F_netForce(bool _transOnly, bool _zeroGravity) : transOnly(_transOnly) {
  order=0;
  if(_zeroGravity) {
    gravity = 0.;
  } else {
    gravity = rai::getParameter<double>("F_static/gravity", 9.81);
  }
}

void F_netForce::phi2(arr& y, arr& J, const FrameL& F) {
  CHECK_EQ(order, 0, "");

  rai::Frame* a = F.scalar();

  arr force, torque, Jforce, Jtorque;
  a->C.kinematicsZero(force, Jforce, 3);
  a->C.kinematicsZero(torque, Jtorque, 3);

  if(gravity) {
    double mass=.1;
    if(a->inertia) mass = a->inertia->mass;
    force(2) += gravity * mass;
  }

  //-- collect contacts and signs FOR ALL shapes attached to this link
  rai::Array<rai::ForceExchange*> contacts;
  arr signs;
  FrameL linkF;
  linkF.append(a);
  a->getRigidSubFrames(linkF);
  for(rai::Frame* f:linkF) {
    for(rai::ForceExchange* ex:f->forces) {
      contacts.append(ex);
      signs.append(ex->sign(f));
//      auto con = dynamic_cast<rai::ForceExchange_Wrench*>(ex);
//      if(con){
//        CHECK(&con->a==f || &con->b==f, "");
//        contacts.append(con);
//        signs.append((&con->a==f ? +1. : -1.));
//      }else{
//        auto tor = dynamic_cast<rai::ForceExchange_JointTorque*>(ex);
//        if(tor){
//          contacts.append(tor);
//          signs.append((tor->j.frame==f) ? +1. : -1.);
//        }else NIY;
//      }
    }
  }

#if 0
  for(rai::ForceExchange* con:a->forces) {
    double sign = +1.;
    CHECK(&con->a==a || &con->b==a, "");
    if(&con->b==a) sign=-1.;
#else
  for(uint i=0; i<contacts.N; i++) {
    rai::ForceExchange* con = contacts(i);
    double sign = signs(i);
#endif

    //get the force
    arr f, Jf;
    con->kinForce(f, Jf);
//    a->C.kinematicsContactForce(f, Jf, con);

    //get the torque
    arr w, Jw;
    con->kinTorque(w, Jw);

    //get the POA
    arr poa, Jpoa;
    con->kinPOA(poa, Jpoa);
//    a->C.kinematicsContactPOA(poa, Jpoa, con);

    //get object center
    arr p, Jp;
    a->C.kinematicsPos(p, Jp, a);

    force -= sign * f;
    Jforce -= sign * Jf;

    if(!transOnly){
      torque += sign * w;
      torque += sign * crossProduct(poa-p, f);

      Jtorque += sign * Jw;
      Jtorque += sign * (skew(poa-p) * Jf - skew(f) * (Jpoa-Jp));
    }
  }

  if(!transOnly){
    y.setBlockVector(force, torque);
    J.setBlockMatrix(Jforce, Jtorque);
  }else{
    y=force;
    J=Jforce;
  }
}

