/*  ------------------------------------------------------------------
    Copyright (c) 2017 Marc Toussaint
    email: marc.toussaint@informatik.uni-stuttgart.de

    This code is distributed under the MIT License.
    Please see <root-path>/LICENSE for details.
    --------------------------------------------------------------  */

#include "TM_ContactConstraints.h"
#include "TM_PairCollision.h"
#include "frame.h"
#include "contact.h"

rai::Contact *getContact(const rai::KinematicWorld &K, int aId, int bId){
  rai::Frame *a = K.frames(aId);
  rai::Frame *b = K.frames(bId);
  for(rai::Contact *c : a->contacts) if(&c->a==a && &c->b==b) return c;
  HALT("can't retrieve contact " <<a->name <<"--" <<b->name);
  return NULL;
}

void TM_ContactConstraints::phi(arr &y, arr &J, const rai::KinematicWorld &K) {
  rai::Contact *con = getContact(K,a,b);

  if(!con->soft){
    y.resize(4).setZero();
    if(&J) J.resize(4, K.getJointStateDimension()).setZero();
  }else{
    y.resize(6).setZero();
    if(&J) J.resize(6, K.getJointStateDimension()).setZero();
  }

  //-- non-aligned force
  //get collision normal
  arr c, Jc;
  TM_PairCollision cvec(con->a.ID, con->b.ID, TM_PairCollision::_normal, true);
  cvec.phi(c, (&J?Jc:NoArr), K);
  //get force
  arr ferr, Jferr;
  K.kinematicsForce(ferr, Jferr, con);
  //subtract c-aligned projection
  if(&J) Jferr -= (c*~c*Jferr + c*~ferr*Jc + scalarProduct(c,ferr)*Jc);
  ferr -= c*scalarProduct(c,ferr);

  y({0,2}) = ferr;
  if(&J) J({0,2}) = Jferr;

  if(!con->soft){
    //-- needs to touch!!
    arr d, Jd;
    TM_PairCollision dist(con->a.ID, con->b.ID, TM_PairCollision::_negScalar, false);
    dist.phi(d, (&J?Jd:NoArr), K);
    con->y = d.scalar();
    con->setFromPairCollision(*dist.coll);

    y(3) = d.scalar();
    if(&J) J[3] = Jd;
  }else{
    arr d, Jd;
    TM_PairCollision dist(con->a.ID, con->b.ID, TM_PairCollision::_negScalar, false);
    dist.phi(d, (&J?Jd:NoArr), K);
    con->y = d.scalar();
    con->setFromPairCollision(*dist.coll);

    //soft! complementarity

    //get force
    arr ferr, Jferr;
    K.kinematicsForce(ferr, Jferr, con);

    double s = 1e-1;
    if(d.scalar()>0.) s=0.;
    y({3,5}) = s*d.scalar() * ferr;
    if(&J) J({3,5}) = (s*d.scalar())*Jferr + (s*ferr) * Jd;
  }

}

uint TM_ContactConstraints::dim_phi(const rai::KinematicWorld &K) {
  rai::Contact *c = getContact(K,a,b);
  if(!c->soft) return 4;
  return 6;
}
