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

void POA_distance(arr& y, arr& J, rai::Contact* con, bool b_or_a){
  rai::Shape *s = con->a.shape;
  if(b_or_a) s = con->b.shape;
  CHECK(s,"contact object does not have a shape!");
  double r=s->radius();
  rai::Mesh *m = &s->sscCore();  if(!m->V.N) { m = &s->mesh(); r=0.; }

  CHECK_EQ(&con->a.K, &con->b.K, "");
  rai::KinematicWorld &K = con->a.K;

  rai::Mesh M0;
  M0.setDot();
  rai::Transformation X0=0;
  arr pos, Jpos;
  K.kinematicsContactPOA(pos, Jpos, con);
  X0.pos = pos;

  PairCollision coll(M0, *m, X0, s->frame.X, 0., r);

  arr Jp;
  K.jacobianPos(Jp, &s->frame, coll.p1);

  coll.kinDistance(y, J, Jpos, Jp);
}

void POA_rel_vel2(arr& y, arr& J, const WorldL& Ktuple, rai::Contact* con, bool after_or_before){
  rai::KinematicWorld *Kc = Ktuple(-2);

  // p1, p2 are the CENTERS! of the frame a and b
  // v1, v2 are the CENTER velocities of the frame a and b
  arr cp, Jcp;
  Kc->kinematicsContactPOA(cp, Jcp, con);
  if(!!J) expandJacobian(Jcp, Ktuple, -2);

  arr Ra = con->a.X.rot.getArr();
  arr Rb = con->b.X.rot.getArr();

  arr p0a, p0b, Jp0a, Jp0b;
  Kc->kinematicsPos(p0a, Jp0a, &con->a);
  Kc->kinematicsPos(p0b, Jp0b, &con->b);
  if(!!J){
    expandJacobian(Jp0a, Ktuple, -2);
    expandJacobian(Jp0b, Ktuple, -2);
  }

  arr rela = ~Ra * (cp - p0a);
  arr relb = ~Rb * (cp - p0b);
  arr Jrela = ~Ra * (Jcp - Jp0a);
  arr Jrelb = ~Rb * (Jcp - Jp0b);

  rai::KinematicWorld *K;
  if(after_or_before) K = Ktuple(-1);
  else K = Ktuple(-3);
  rai::Frame *fa = K->frames(con->a.ID);
  rai::Frame *fb = K->frames(con->b.ID);
  arr pa, pb, Jpa, Jpb;
  K->kinematicsPos(pa, Jpa, fa, rela);
  K->kinematicsPos(pb, Jpb, fb, relb);
  if(!!J){
    if(after_or_before){
      expandJacobian(Jpa, Ktuple, -1);
      expandJacobian(Jpb, Ktuple, -1);
    }else{
      expandJacobian(Jpa, Ktuple, -3);
      expandJacobian(Jpb, Ktuple, -3);
    }
    Jpa += fa->X.rot.getArr() * Jrela;
    Jpb += fb->X.rot.getArr() * Jrelb;
  }
  y = pa - pb;
  if(!!J){
    J = Jpa - Jpb;
  }
}

//3-dim feature: the difference in POA velocities (V)
void POA_rel_vel(arr& y, arr& J, const WorldL& Ktuple, rai::Contact* con, bool after_or_before){
  CHECK_EQ(Ktuple.N, 3, "");

  rai::KinematicWorld *Kc = Ktuple(-2);

  arr cp, Jcp;
  Kc->kinematicsContactPOA(cp, Jcp, con);
  expandJacobian(Jcp, Ktuple, -2);

  // p1, p2 are the CENTERS! of the frame a and b
  // v1, v2 are the CENTER velocities of the frame a and b
  arr p1, p2, Jp1, Jp2;
  arr v1, v2, Jv1, Jv2;
  TM_Default lin(TMT_pos, con->a.ID);
  lin.order=0;  lin.i=con->a.ID;  lin.Feature::__phi(p1, Jp1, Ktuple({0,1}));
  lin.order=0;  lin.i=con->b.ID;  lin.Feature::__phi(p2, Jp2, Ktuple({0,1}));
  padJacobian(Jp1, Ktuple);
  padJacobian(Jp2, Ktuple);
  if(after_or_before){
    lin.order=1;  lin.i=con->a.ID;  lin.Feature::__phi(v1, Jv1, Ktuple);
    lin.order=1;  lin.i=con->b.ID;  lin.Feature::__phi(v2, Jv2, Ktuple);
  }else{
    lin.order=1;  lin.i=con->a.ID;  lin.Feature::__phi(v1, Jv1, Ktuple({0,1}));
    lin.order=1;  lin.i=con->b.ID;  lin.Feature::__phi(v2, Jv2, Ktuple({0,1}));
    padJacobian(Jv1, Ktuple);
    padJacobian(Jv2, Ktuple);
  }

  arr w1, w2, Jw1, Jw2;
  TM_AngVel ang(con->a.ID);
  if(after_or_before){
    ang.order=1;  ang.i=con->a.ID;  ang.__phi(w1, Jw1, Ktuple);
    ang.order=1;  ang.i=con->b.ID;  ang.__phi(w2, Jw2, Ktuple);
  }else{
    ang.order=1;  ang.i=con->a.ID;  ang.__phi(w1, Jw1, Ktuple({0,1}));
    ang.order=1;  ang.i=con->b.ID;  ang.__phi(w2, Jw2, Ktuple({0,1}));
    padJacobian(Jw1, Ktuple);
    padJacobian(Jw2, Ktuple);
  }

  arr vc1 = v1 - crossProduct(w1, cp - p1);
  arr Jvc1 = Jv1 - skew(w1) * (Jcp - Jp1) + skew(cp-p1) * Jw1;
  arr vc2 = v2 - crossProduct(w2, cp - p2);
  arr Jvc2 = Jv2 - skew(w2) * (Jcp - Jp2) + skew(cp-p2) * Jw2;

  y = vc1 - vc2;
  if(!!J) J = Jvc1 - Jvc2;
}

//3-dim feature: the POA velocities (V)
void POA_vel(arr& y, arr& J, const WorldL& Ktuple, rai::Contact* con, bool b_or_a){
  CHECK_GE(Ktuple.N, 2, "");

  rai::Frame *f = &con->a;
  if(b_or_a) f = &con->b;

  //POA
  arr cp, Jcp;
  Ktuple(-2)->kinematicsContactPOA(cp, Jcp, con);
  expandJacobian(Jcp, Ktuple, -2);

  //object center
  arr p, Jp;
  TM_Default pos(TMT_pos, f->ID);
  pos.Feature::__phi(p, Jp, Ktuple);

  //object vel
  arr v, Jv;
  TM_LinVel vel(f->ID);
  vel.phi(v, Jv, Ktuple);

  //object ang vel
  arr w, Jw;
  TM_AngVel ang(f->ID);
  ang.phi(w, Jw, Ktuple);

  y = v - crossProduct(w, cp - p);
  if(!!J) J = Jv - skew(w) * (Jcp - Jp) + skew(cp-p) * Jw;
}

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
  arr d0, Jd0;
  arr d1, Jd1;
  POA_distance(d0, Jd0, con, false);
  POA_distance(d1, Jd1, con, true);

  //-- enforce complementarity
  y.resize(2,3);
  if(!!J) J.resize(2, 3, Jd0.d1);

  y[0] = d0.scalar() * force;
  y[1] = d1.scalar() * force;
  y.reshape(6);
  if(!!J){
    J[0] = d0.scalar()*Jforce + force * Jd0;
    J[1] = d1.scalar()*Jforce + force * Jd1;
    J.reshape(6,J.d2);
  }
}

uint TM_Contact_ForceIsComplementary::dim_phi(const rai::KinematicWorld& K){ return 6; }


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
  double r1=s1->radius();
  double r2=s2->radius();
  rai::Mesh *m1 = &s1->sscCore();  if(!m1->V.N) { m1 = &s1->mesh(); r1=0.; }
  rai::Mesh *m2 = &s2->sscCore();  if(!m2->V.N) { m2 = &s2->mesh(); r2=0.; }

  rai::Mesh M0;
  M0.setDot();
  rai::Transformation X0=0;
  arr pos, Jpos;
  K.kinematicsContactPOA(pos, Jpos, con);
  X0.pos = pos;

  PairCollision coll1(M0, *m1, X0, s1->frame.X, 0., r1);
  PairCollision coll2(M0, *m2, X0, s2->frame.X, 0., r2);

  arr Jp1, Jp2;
  K.jacobianPos(Jp1, &s1->frame, coll1.p1);
  K.jacobianPos(Jp2, &s2->frame, coll2.p2);

  coll1.kinDistance(y({1,1})(), (!!J?J[1]():NoArr), Jpos, Jp1);
  coll2.kinDistance(y({2,2})(), (!!J?J[2]():NoArr), Jpos, Jp2);

  if(margin){
    y(1) -= margin;
    y(2) -= margin;
  }

  if(!!J) checkNan(J);
}

uint TM_Contact_POAisInIntersection_InEq::dim_phi(const rai::KinematicWorld& K){
  return 3;
}

void TM_Contact_ForceRegularization::phi(arr& y, arr& J, const rai::KinematicWorld& K){
  rai::Contact *con = getContact(K,a,b);

  K.kinematicsContactForce(y, J, con);
}

uint TM_Contact_ForceRegularization::dim_phi(const rai::KinematicWorld& K){ return 3; }

void TM_ContactConstraints_Vel::phi(arr& y, arr& J, const WorldL& Ktuple){
  CHECK_EQ(order, 1, "");

  rai::KinematicWorld& K = *Ktuple(-2); //!!! use LAST contact, and velocities AFTER contact

  rai::Contact *con = getContact(K,a,b);

  arr cp, Jcp;
  K.kinematicsContactPOA(cp, Jcp, con);
  expandJacobian(Jcp, Ktuple, -2);

  //  HALT("should be velocity in the NEXT time slice...?");

  // p1, p2 are the CENTERS! of the frame a and b
  // v1, v2 are the CENTER velocities of the frame a and b
  arr p1, p2, Jp1, Jp2;
  arr v1, v2, Jv1, Jv2;
  TM_Default lin(TMT_pos, a);
  lin.order=0; lin.i=a;
  lin.Feature::__phi(p1, Jp1, Ktuple);
  lin.order=0; lin.i=b;
  lin.Feature::__phi(p2, Jp2, Ktuple);
  lin.order=1; lin.i=a;
  lin.Feature::__phi(v1, Jv1, Ktuple);
  lin.order=1; lin.i=b;
  lin.Feature::__phi(v2, Jv2, Ktuple);

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
}

uint TM_ContactConstraints_Vel::dim_phi(const rai::KinematicWorld& K)
{
  return 3;
}

void TM_Contact_POAmovesContinuously::phi(arr& y, arr& J, const WorldL& Ktuple){
  arr cp1, Jcp1;
  arr cp2, Jcp2;
  Ktuple(-2)->kinematicsContactPOA(cp1, Jcp1, getContact(*Ktuple(-2),a,b));
  Ktuple(-1)->kinematicsContactPOA(cp2, Jcp2, getContact(*Ktuple(-1),a,b));

  y = cp1 - cp2;
  if(!!J){
    expandJacobian(Jcp1, Ktuple, -2);
    expandJacobian(Jcp2, Ktuple, -1);
    J = Jcp1 - Jcp2;
  }
}

void TM_Contact_POAzeroRelVel::phi(arr& y, arr& J, const WorldL& Ktuple){
  rai::Contact *con = getContact(*Ktuple(-2),a,b);
#if 0
  POA_rel_vel(y, J, Ktuple, con, true);
#else
  arr v1, Jv1, v2, Jv2;
  POA_vel(v1, Jv1, Ktuple, con, false);
  POA_vel(v2, Jv2, Ktuple, con, true);
  y = v1 - v2;
  if(!!J) J = Jv1 - Jv2;
#endif
}

void TM_Contact_ElasticVel::phi(arr& y, arr& J, const WorldL& Ktuple){
  rai::Contact *con = getContact(*Ktuple(-2),a,b);
  arr v0, Jv0, v1, Jv1;
  POA_rel_vel(v0, Jv0, Ktuple, con, false);
  POA_rel_vel(v1, Jv1, Ktuple, con, true);

  //-- from the geometry we need normal
  arr normal, Jnormal;
  TM_PairCollision coll(con->a.ID, con->b.ID, TM_PairCollision::_normal, false);
  coll.phi(normal, (!!J?Jnormal:NoArr), *Ktuple(-2));
  if(!!J) expandJacobian(Jnormal, Ktuple, -2);

  y.resize(4).setZero();
  if(!!J) J.resize(4, Jv1.d1).setZero();

  //tangential vel
  if(stickiness==1.){
    y({0,2}) = v1 - normal*scalarProduct(normal,v1);
    if(!!J) J({0,2}) = Jv1 - (normal*~normal*Jv1 + normal*~v1*Jnormal + scalarProduct(normal,v1)*Jnormal);
  }else if(stickiness>0.){
    CHECK_LE(stickiness, 1., "");
    double alpha=1.-stickiness;
    y({0,2}) = (v1-alpha*v0) - normal*scalarProduct(normal, v1-alpha*v0);
    if(!!J) J({0,2}) = (Jv1-alpha*Jv0) - (normal*~normal*(Jv1-alpha*Jv0) + normal*~(v1-alpha*v0)*Jnormal + scalarProduct(normal,(v1-alpha*v0))*Jnormal);
  }

  //normal vel
  if(elasticity>0.){
    y(3) = scalarProduct(normal, v1 + elasticity*v0);
    if(!!J) J[3] = ~normal*(Jv1+elasticity*Jv0) + ~(v1+elasticity*v0)*Jnormal;
  }else if(elasticity==0.){
    y(3) = scalarProduct(normal, v1);
    if(!!J) J[3] = ~normal*(Jv1) + ~(v1)*Jnormal;
  }
}

void TM_Contact_ElasticVelIsComplementary::phi(arr& y, arr& J, const WorldL& Ktuple){
  rai::KinematicWorld& K = *Ktuple(-2);
  rai::Contact *con = getContact(K,a,b);

  //-- get the pre and post V:
  arr v0, Jv0, v1, Jv1;
  POA_rel_vel(v0, Jv0, Ktuple, con, false);
  POA_rel_vel(v1, Jv1, Ktuple, con, true);

  //-- get the force
  arr force, Jforce;
  K.kinematicsContactForce(force, Jforce, con);
  if(!!J) expandJacobian(Jforce, Ktuple, -2);

  y.resize(1);
  y(0) = scalarProduct(force, v1);

  if(!!J){
    J.resize(y.N, Jforce.d1);
    J = ~force * Jv1 + ~v1 * Jforce;
  }
}
