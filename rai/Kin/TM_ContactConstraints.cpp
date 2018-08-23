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

  if(!con->soft) y.resize(5).setZero();
  else  y.resize(7).setZero();
  if(&J) J.resize(y.N, K.getJointStateDimension()).setZero();

  //-- from the contact we need position & force
  arr pos, Jpos;
  K.kinematicsContactPosition(pos, Jpos, con);

  arr force, Jforce;
  K.kinematicsContactForce(force, Jforce, con);

  //-- from the geometry we need center, normal, distance (TODO: use same object!!)
  arr cen, Jcen, normal, Jnormal, dist, Jdist;
  TM_PairCollision coll(con->a.ID, con->b.ID, TM_PairCollision::_center, false);
  coll.phi(cen, (&J?Jcen:NoArr), K);

  coll.type=TM_PairCollision::_normal;
  coll.phi(normal, (&J?Jnormal:NoArr), K);

  coll.type=TM_PairCollision::_negScalar;
  coll.phi(dist, (&J?Jdist:NoArr), K);

  con->setFromPairCollision(*coll.coll);

  //-- position needs to be on contact surface
  y(0) = scalarProduct(normal, pos - cen);
  if(&J) J[0] = ~normal * (Jpos - Jcen) + ~(pos-cen) * Jnormal;

  //-- force needs to align with normal -> project force along normal
  y({1,3}) = force - normal*scalarProduct(normal,force);
  if(&J) J({1,3}) = Jforce - (normal*~normal*Jforce + normal*~force*Jnormal + scalarProduct(normal,force)*Jnormal);

  if(!con->soft){
    //-- needs to touch!!
    y(4) = dist.scalar();
    if(&J) J[4] = Jdist;
  }else{
    //-- enforce complementarity
    double s = 1e-1;
    y({4,6}) = s*dist.scalar() * force;
    if(&J) J({4,6}) = (s*dist.scalar())*Jforce + (s*force) * Jdist;
  }

}

uint TM_ContactConstraints::dim_phi(const rai::KinematicWorld &K) {
  rai::Contact *c = getContact(K,a,b);
  if(!c->soft) return 5;
  return 7;
}
