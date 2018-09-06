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


TM_NewtonEuler::TM_NewtonEuler(int iShape) : i(iShape) {
  order=2;
  gravity = rai::getParameter<double>("TM_Physics/gravity", 9.81);
}

void TM_NewtonEuler::phi(arr &y, arr &J, const WorldL &Ktuple) {
  CHECK_EQ(order, 2, "");

  //this is the direct impuls exchange case, where NewtonEuler is switched off
  if((Ktuple(-1)->frames(i)->flags & (1<<FL_impulseExchange))){
    y.resize(6).setZero();
    if(!!J) J.resize(6, getKtupleDim(Ktuple).last()).setZero();
    return;
  }

  //get linear and angular accelerations
  arr acc, Jacc, wcc, Jwcc;
  TM_Default pos(TMT_posDiff, i);
  pos.order=2;
  pos.Feature::phi(acc, (!!J?Jacc:NoArr), Ktuple);
  acc(2) += gravity;

  TM_AngVel rot(i);
  rot.order=2;
  rot.phi(wcc, (!!J?Jwcc:NoArr), Ktuple);

  rai::KinematicWorld& K = *Ktuple(-2); // ! THIS IS THE MID TIME SLICE !
  rai::Frame *a = K.frames(i);
  double mass=1;
  arr Imatrix = diag(.1, 3);
  if(a->inertia){
    mass = a->inertia->mass;
    Imatrix = conv_mat2arr(a->inertia->matrix);
  }

  mass = 1./mass;
  Imatrix = inverse_SymPosDef(Imatrix);
  double forceScaling = 1e3;

  for(rai::Contact *c:a->contacts){
    double sign = +1.;
    CHECK(&c->a==a || &c->b==a, "");
    if(&c->b==a) sign=-1.;

    arr f, Jf;
    K.kinematicsContactForce(f, Jf, c);
    if(!!J) expandJacobian(Jf, Ktuple, -2);

//    arr d, Jd;
//    TM_PairCollision dist(con->a.ID, con->b.ID, TM_PairCollision::_normal, false);
//    dist.phi(d, (!!J?Jd:NoArr), K);
//    con->y = d.scalar();
//    con->setFromPairCollision(*dist.coll);


    arr cp, Jcp;
#if 0
    if(&c->a==a)
      K.kinematicsVec(cp, Jcp, a, c->a_rel); //contact point VECTOR only
    else
      K.kinematicsVec(cp, Jcp, a, c->b_rel); //contact point VECTOR only
#else
//    TM_PairCollision dist(c->a.ID, c->b.ID, TM_PairCollision::_p1, false);
//    if(&c->b==a) dist.type=TM_PairCollision::_p2;

//    dist.phi(cp, (!!J?Jcp:NoArr), K);

    K.kinematicsContactPosition(cp, Jcp, c);

    arr p,Jp;
    K.kinematicsPos(p, Jp, a);
    cp -= p;
    if(!!J) Jcp -= Jp;
#endif
    if(!!J) expandJacobian(Jcp, Ktuple, -2);

    acc -= sign * forceScaling *mass* c->force;
    wcc += sign * .1 * forceScaling *Imatrix* crossProduct(cp, c->force);

    if(!!J){
      Jacc -= sign * forceScaling *mass* Jf;
      Jwcc += sign * .1 * forceScaling *Imatrix* (skew(cp) * Jf - skew(c->force) * Jcp);
    }
  }

        
  y.resize(6).setZero();
  y.setVectorBlock(acc, 0);
  y.setVectorBlock(1.*wcc, 3);

  if(!!J) {
    J.resize(6, Jacc.d1).setZero();
    J.setMatrixBlock(Jacc, 0, 0);
    J.setMatrixBlock(1.*Jwcc, 3, 0);
  }
}

