/*  ------------------------------------------------------------------
    Copyright (c) 2017 Marc Toussaint
    email: marc.toussaint@informatik.uni-stuttgart.de

    This code is distributed under the MIT License.
    Please see <root-path>/LICENSE for details.
    --------------------------------------------------------------  */

#include "TM_physics.h"
#include <Kin/flag.h>
#include <Kin/frame.h>
#include <Kin/contact.h>
#include <Kin/TM_default.h>
#include <Kin/TM_angVel.h>
#include <Kin/TM_PairCollision.h>

void shapeFunction(double &x, double &dx);


TM_Physics::TM_Physics() {
  gravity = rai::getParameter<double>("TM_Physics/gravity", 9.81);
}

void TM_Physics::phi(arr &y, arr &J, const WorldL &Ktuple) {
  y.clear();
  if(&J) J.clear();

  CHECK_EQ(order, 2, "");
  
  rai::KinematicWorld& K = *Ktuple(-2); // ! THIS IS THE MID TIME SLICE !
//  uintA qdim = getKtupleDim(Ktuple);
    
  arr acc, Jacc, wcc, Jwcc;
  arr acc_ref = {0.,0.,-gravity};

  for(rai::Frame *a:K.frames) {
    if(a->inertia && a->inertia->type==rai::BT_dynamic){
      TM_Default pos(TMT_posDiff, a->ID);
      pos.order=2;
      pos.TaskMap::phi(acc, (&J?Jacc:NoArr), Ktuple);

      TM_AngVel rot(a->ID);
      rot.order=2;
      rot.phi(wcc, (&J?Jwcc:NoArr), Ktuple);

      acc -= acc_ref;

      for(rai::Contact *c:a->contacts){
        double sign = +1.;
        CHECK(&c->a==a || &c->b==a, "");
        if(&c->b==a) sign=-1.;

        arr f, Jf;
        K.kinematicsForce(f, Jf, c);
        if(&J) expandJacobian(Jf, Ktuple, -2);

//        arr cp, Jcp;
//        K.kinematicsVec(cp, Jcp, a, c->b_rel); //contact point VECTOR only
//        expandJacobian(Jcp, Ktuple, -2);

        acc += sign * 20. * c->force;
//        wcc -= .1 * crossProduct((a->X.rot*c->b_rel).getArr(), c->force);
//        wcc -= 2. * crossProduct(cp, c->force);
        if(&J){
          Jacc += sign * 20. * Jf;
//          Jwcc -= 2. * (skew(cp) * Jf - skew(c->force) * Jcp);
        }
      }
        
      y.append(acc);
      y.append(wcc);

      if(&J) {
        J.append( Jacc );
        J.append( Jwcc );
      }

    }
  }
  
  uintA KD = getKtupleDim(Ktuple);
  if(&J) J.reshape(y.N, KD.last());
}

uint TM_Physics::dim_phi(const WorldL &Ktuple) {
  rai::KinematicWorld& K = *Ktuple(-1);
  uint d = 0;
  for(rai::Frame *a: K.frames)
    if(a->inertia && a->inertia->type==rai::BT_dynamic){
      d+=6;
    }
  return d;
}
