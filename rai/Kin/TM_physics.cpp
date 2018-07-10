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


TM_Physics::TM_Physics(int iShape) : i(iShape) {
  order=2;
  gravity = rai::getParameter<double>("TM_Physics/gravity", 9.81);
}

void TM_Physics::phi(arr &y, arr &J, const WorldL &Ktuple) {

  CHECK_EQ(order, 2, "");
  
  rai::KinematicWorld& K = *Ktuple(-2); // ! THIS IS THE MID TIME SLICE !
    
  arr acc, Jacc, wcc, Jwcc;
  arr acc_ref = {0.,0.,-gravity};


  TM_Default pos(TMT_posDiff, i);
  pos.order=2;
  pos.Feature::phi(acc, (&J?Jacc:NoArr), Ktuple);

  TM_AngVel rot(i);
  rot.order=2;
  rot.phi(wcc, (&J?Jwcc:NoArr), Ktuple);

  acc -= acc_ref;

  rai::Frame *a = K.frames(i);
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
        
  y.resize(6).setZero();
  y.setVectorBlock(acc, 0);
  y.setVectorBlock(wcc, 3);

  if(&J) {
    J.resize(6, Jacc.d1).setZero();
    J.setMatrixBlock(Jacc, 0, 0);
    J.setMatrixBlock(Jwcc, 3, 0);
  }
}

