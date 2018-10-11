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
#include <Geo/pairCollision.h>
#include "TM_angVel.h"

rai::Contact *getContact(const rai::KinematicWorld &K, int aId, int bId){
  rai::Frame *a = K.frames(aId);
  rai::Frame *b = K.frames(bId);
  for(rai::Contact *c : a->contacts) if(&c->a==a && &c->b==b) return c;
  HALT("can't retrieve contact " <<a->name <<"--" <<b->name);
  return NULL;
}

void TM_Contact_ForceIsNormal::phi(arr &y, arr &J, const rai::KinematicWorld &K) {
  rai::Contact *con = getContact(K,a,b);

  //-- from the contact we need force
  arr force, Jforce;
  K.kinematicsContactForce(force, Jforce, con);

  //-- from the geometry we need normal
  arr normal, Jnormal;
  TM_PairCollision coll(con->a.ID, con->b.ID, TM_PairCollision::_normal, false);
  coll.phi(normal, (!!J?Jnormal:NoArr), K);

  //-- force needs to align with normal -> project force along normal
  y = force - normal*scalarProduct(normal,force);
  if(!!J) J = Jforce - (normal*~normal*Jforce + normal*~force*Jnormal + scalarProduct(normal,force)*Jnormal);
}

void TM_Contact_ForceIsComplementary::phi(arr &y, arr &J, const rai::KinematicWorld &K) {
  rai::Contact *con = getContact(K,a,b);

  //-- from the contact we need force
  arr force, Jforce;
  K.kinematicsContactForce(force, Jforce, con);

  //-- from the geometry we need distance
  arr dist, Jdist;
  TM_PairCollision coll(con->a.ID, con->b.ID, TM_PairCollision::_negScalar, false);
  coll.phi(dist, (!!J?Jdist:NoArr), K);

  //-- enforce complementarity
  double s = 1e-0;
  y = s*dist.scalar() * force;
  if(!!J) J = (s*dist.scalar())*Jforce + (s*force) * Jdist;

}

void TM_Contact_POAisInIntersection_InEq::phi(arr& y, arr& J, const rai::KinematicWorld& K){
  rai::Contact *con = getContact(K,a,b);

  y.resize(3).setZero();
  if(!!J){ J.resize(3, K.getJointStateDimension()).setZero(); }

  //-- only pushing forces
  arr force, Jforce;
  K.kinematicsContactForce(force, Jforce, con);

  arr normal, Jnormal;
  TM_PairCollision coll(con->a.ID, con->b.ID, TM_PairCollision::_normal, false);
  coll.phi(normal, (!!J?Jnormal:NoArr), K);

  y(0) = - scalarProduct(normal, force);
  if(!!J) J[0] = - ~normal * Jforce - ~force * Jnormal;

  //-- POA inside objects (eventually on surface!)
  rai::Shape *s1 = K.frames(a)->shape;
  rai::Shape *s2 = K.frames(b)->shape;
  CHECK(s1 && s2,"");
  double r1=s1->size(3);
  double r2=s2->size(3);
  rai::Mesh *m1 = &s1->sscCore();  if(!m1->V.N) { m1 = &s1->mesh(); r1=0.; }
  rai::Mesh *m2 = &s2->sscCore();  if(!m2->V.N) { m2 = &s2->mesh(); r2=0.; }

  rai::Mesh M0;
  M0.setDot();
  rai::Transformation X0=0;
  arr pos, Jpos;
  K.kinematicsContactPosition(pos, Jpos, con);
  X0.pos = pos;

  PairCollision coll1(M0, *m1, X0, s1->frame.X, 0., r1);
  PairCollision coll2(M0, *m2, X0, s2->frame.X, 0., r2);

  arr Jp1, Jp2;
  K.jacobianPos(Jp1, &s1->frame, coll1.p1);
  K.jacobianPos(Jp2, &s2->frame, coll2.p2);

  coll1.kinDistance(y({1,1})(), (!!J?J[1]():NoArr), Jpos, Jp1);
  coll2.kinDistance(y({2,2})(), (!!J?J[2]():NoArr), Jpos, Jp2);

  y(1) -= .001;
  y(2) -= .001;

  if(!!J) checkNan(J);
}

uint TM_Contact_POAisInIntersection_InEq::dim_phi(const rai::KinematicWorld& K){
  return 3;
}

void TM_ContactConstraints_SOS::phi(arr& y, arr& J, const rai::KinematicWorld& K){
  rai::Contact *con = getContact(K,a,b);

  K.kinematicsContactForce(y, J, con);
}

uint TM_ContactConstraints_SOS::dim_phi(const rai::KinematicWorld& K){ return 3; }

void TM_ContactConstraints_Vel::phi(arr& y, arr& J, const WorldL& Ktuple){
  CHECK_EQ(order, 1, "");

  rai::KinematicWorld& K = *Ktuple(-1);

  rai::Contact *con = getContact(K,a,b);

  arr cp, Jcp;
  K.kinematicsContactPosition(cp, Jcp, con);
  expandJacobian(Jcp, Ktuple, -1);

  arr p1, p2, Jp1, Jp2;
  arr v1, v2, Jv1, Jv2;
  TM_Default lin(TMT_pos, a);
  lin.order=0; lin.i=a;
  lin.Feature::phi(p1, Jp1, Ktuple);
  lin.order=0; lin.i=b;
  lin.Feature::phi(p2, Jp2, Ktuple);
  lin.order=1; lin.i=a;
  lin.Feature::phi(v1, Jv1, Ktuple);
  lin.order=1; lin.i=b;
  lin.Feature::phi(v2, Jv2, Ktuple);

  arr w1, w2, Jw1, Jw2;
  TM_AngVel ang(a);
  ang.order=1; ang.i=a;
  ang.phi(w1, Jw1, Ktuple);
  ang.order=1; ang.i=b;
  ang.phi(w2, Jw2, Ktuple);

  arr vc1 = v1 - crossProduct(w1, cp - p1);
  arr Jvc1 = Jv1 - skew(w1) * (Jcp - Jp1) + skew(cp-p1) * Jw1;
  arr vc2 = v2 - crossProduct(w2, cp - p2);
  arr Jvc2 = Jv2 - skew(w2) * (Jcp - Jp2) + skew(cp-p2) * Jw2;

  y = vc1 - vc2;
  if(!!J) J = Jvc1 - Jvc2;

  if(con->soft){
    //-- enforce complementarity
    NIY;
  }

}

uint TM_ContactConstraints_Vel::dim_phi(const rai::KinematicWorld& K)
{
  return 3;
}
