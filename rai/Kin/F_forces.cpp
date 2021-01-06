/*  ------------------------------------------------------------------
    Copyright (c) 2011-2020 Marc Toussaint
    email: toussaint@tu-berlin.de

    This code is distributed under the MIT License.
    Please see <root-path>/LICENSE for details.
    --------------------------------------------------------------  */

#include "F_forces.h"

#include "F_pose.h"
#include "frame.h"
#include "forceExchange.h"
#include "F_collisions.h"
#include "../Geo/pairCollision.h"

//===========================================================================
//helper methods:

void POA_distance(arr& y, arr& J, rai::ForceExchange* ex, bool b_or_a) {
  rai::Shape* s = ex->a.shape;
  if(b_or_a) s = ex->b.shape;
  CHECK(s, "contact object does not have a shape!");
  double r=s->radius();
  rai::Mesh* m = &s->sscCore();  if(!m->V.N) { m = &s->mesh(); r=0.; }

  CHECK_EQ(&ex->a.C, &ex->b.C, "");
  rai::Configuration& K = ex->a.C;

  rai::Mesh M0;
  M0.setDot();
  rai::Transformation X0=0;
  arr pos, Jpos;
  ex->kinPOA(pos, Jpos);
  X0.pos = pos;

  PairCollision coll(M0, *m, X0, s->frame.ensure_X(), 0., r);

  arr Jp;
  K.jacobian_pos(Jp, &s->frame, coll.p1);

  coll.kinDistance(y, J, Jpos, Jp);
}

void POA_rel_vel2(arr& y, arr& J, const FrameL& F, rai::ForceExchange* ex, bool after_or_before) {
  CHECK_EQ(F.d0, 3, "");
  CHECK_EQ(F.d1, 2, "");
  CHECK_EQ(F(1,0), &ex->a, "");
  CHECK_EQ(F(1,1), &ex->b, "");

  // p1, p2 are the CENTERS! of the frame a and b
  // v1, v2 are the CENTER velocities of the frame a and b

  arr cp, Jcp;
  ex->kinPOA(cp, Jcp);

  arr Ra = ex->a.ensure_X().rot.getArr();
  arr Rb = ex->b.ensure_X().rot.getArr();

  arr p0a, p0b, Jp0a, Jp0b;
  ex->a.C.kinematicsPos(p0a, Jp0a, &ex->a);
  ex->a.C.kinematicsPos(p0b, Jp0b, &ex->b);

  arr rela = ~Ra * (cp - p0a);
  arr relb = ~Rb * (cp - p0b);
  arr Jrela = ~Ra * (Jcp - Jp0a);
  arr Jrelb = ~Rb * (Jcp - Jp0b);

  FrameL K;
  if(after_or_before) K = F[-1];
  else K = F[-3];
  arr pa, pb, Jpa, Jpb;
  K(0)->C.kinematicsPos(pa, Jpa, K(0), rela);
  K(2)->C.kinematicsPos(pb, Jpb, K(1), relb);
  if(!!J) {
    Jpa += F(0)->ensure_X().rot.getArr() * Jrela;
    Jpb += F(1)->ensure_X().rot.getArr() * Jrelb;
  }
  y = pa - pb;
  if(!!J) J = Jpa - Jpb;
}

//3-dim feature: the difference in POA velocities (V)
void POA_rel_vel(arr& y, arr& J, const FrameL& F, rai::ForceExchange* ex, bool after_or_before) {
  CHECK_EQ(F.d0, 2, "");
  CHECK_EQ(F.d1, 2, "");
  if(after_or_before){
    CHECK_EQ(F(0,0), &ex->a, "");
    CHECK_EQ(F(0,1), &ex->b, "");
  }else{
    CHECK_EQ(F(1,0), &ex->a, "");
    CHECK_EQ(F(1,1), &ex->b, "");
  }

  arr cp, cpJ;
  ex->kinPOA(cp, cpJ);

  // p1, p2 are the CENTERS! of the frame a and b
  // v1, v2 are the CENTER velocities of the frame a and b
  Value p1 = F_Position().eval({&ex->a});
  Value p2 = F_Position().eval({&ex->b});
  Value v1, v2;
  v1 = F_Position().setOrder(1).eval({F(0,0), F(1,0)});
  v2 = F_Position().setOrder(1).eval({F(0,1), F(1,1)});
  Value w1, w2;
  w1 = F_AngVel().eval({{2,1}, {F(0,0), F(1,0)}});
  w2 = F_AngVel().eval({{2,1}, {F(0,1), F(1,1)}});

  arr vc1 = v1.y - crossProduct(w1.y, cp - p1.y);
  arr Jvc1 = v1.J - skew(w1.y) * (cpJ - p1.J) + skew(cp-p1.y) * w1.J;
  arr vc2 = v2.y - crossProduct(w2.y, cp - p2.y);
  arr Jvc2 = v2.J - skew(w2.y) * (cpJ - p2.J) + skew(cp-p2.y) * w2.J;

  y = vc1 - vc2;
  if(!!J) J = Jvc1 - Jvc2;
}

//3-dim feature: the POA velocities (V)
void POA_vel(arr& y, arr& J, const FrameL& F, rai::ForceExchange* ex, bool b_or_a) {
  CHECK_GE(F.d0, 2, "");
  CHECK_GE(F.d1, 2, "");
  CHECK_EQ(F(1,0), &ex->a, "");
  CHECK_EQ(F(1,1), &ex->b, "");

  FrameL ff = {F(0,0), F(1,0)};
  if(b_or_a) ff = {F(0,1), F(1,1)};

  //POA
  arr poa, Jpoa;
  ex->kinPOA(poa, Jpoa);

  Value p = F_Position() .eval({ff(1)});
  Value v = F_LinVel() .eval(ff);
  Value w = F_AngVel() .eval(ff);

  y = v.y - crossProduct(w.y, poa - p.y);
  if(!!J) J = v.J - skew(w.y) * (Jpoa - p.J) + skew(poa-p.y) * w.J;
}

void shapeFunction(double& x, double& dx);

//===========================================================================

void F_fex_POA::phi2(arr& y, arr& J, const FrameL& F) {
  if(order>0){  Feature::phi2(y, J, F);  return;  }
  CHECK_EQ(F.N, 2, "");
  rai::ForceExchange* ex = getContact(F.elem(0), F.elem(1));
  ex->kinPOA(y, J);
}

void F_fex_Force::phi2(arr& y, arr& J, const FrameL& F) {
  if(order>0){  Feature::phi2(y, J, F);  return;  }
  CHECK_EQ(F.N, 2, "");
  rai::ForceExchange* ex = getContact(F.elem(0), F.elem(1));
  ex->kinForce(y, J);
}

void F_fex_Wrench::phi2(arr& y, arr& J, const FrameL& F){
  if(order>0){  Feature::phi2(y, J, F);  return;  }
  CHECK_EQ(F.N, 2, "");
  rai::ForceExchange* ex = getContact(F.elem(0), F.elem(1));
  arr y1, y2, J1, J2;
  ex->kinForce(y1, J1);
  ex->kinTorque(y2, J2);
  y.setBlockVector(y1, y2);
  J.setBlockMatrix(J1, J2);
}

//===========================================================================

void F_HingeXTorque::phi2(arr& y, arr& J, const FrameL& F){
  if(order>0){  Feature::phi2(y, J, F);  return;  }
  CHECK_EQ(F.N, 2, "");
  rai::Frame *f1 = F.elem(0);
  rai::Frame *f2 = F.elem(1);
  CHECK(f2->joint, "second frame needs to be a joint");
  CHECK_EQ(f2->joint->type, rai::JT_hingeX, "second frame needs to be a joint")
  rai::ForceExchange* ex = getContact(f1, f2);
  arr y2, J2;
  ex->kinTorque(y2, J2);

  auto axis = F_Vector(Vector_x)
              .eval({f2});

  y.resize(1) = scalarProduct(y2, axis.y);
  if(!!J) J = ~y2 * axis.J + ~axis.y * J2;
}

F_TotalForce::F_TotalForce(bool _zeroGravity) {
  order=0;
  if(_zeroGravity) {
    gravity = 0.;
  } else {
    gravity = rai::getParameter<double>("gravity", 9.81);
  }
}

void F_TotalForce::phi2(arr& y, arr& J, const FrameL& F) {
  CHECK_EQ(order, 0, "");

  rai::Frame* a = F.scalar();

  arr force, torque, Jforce, Jtorque;
  a->C.kinematicsZero(force, Jforce, 3);
  a->C.kinematicsZero(torque, Jtorque, 3);

  if(gravity) {
    double mass=1.;
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
    }
  }

  for(uint i=0; i<contacts.N; i++) {
    rai::ForceExchange* con = contacts(i);
    double sign = signs(i);

    //get the force
    arr f, Jf;
    con->kinForce(f, Jf);

    //get the torque
    arr w, Jw;
    con->kinTorque(w, Jw);

    //get the POA
    arr poa, Jpoa;
    con->kinPOA(poa, Jpoa);

    //get object center
    arr p, Jp;
    a->C.kinematicsPos(p, Jp, a);

    force -= sign * f;
    Jforce -= sign * Jf;

    torque += sign * w;
    torque += sign * crossProduct(poa-p, f);

    Jtorque += sign * Jw;
    Jtorque += sign * (skew(poa-p) * Jf - skew(f) * (Jpoa-Jp));
  }

  y.setBlockVector(force, torque);
  J.setBlockMatrix(Jforce, Jtorque);
}

//===========================================================================

void F_GravityAcceleration::phi2(arr& y, arr& J, const FrameL& F){
  CHECK_EQ(F.N, 1, "");

  rai::Frame *a = F.scalar();
  a->C.kinematicsZero(y, J, 6);
  if(!impulseInsteadOfAcceleration){
    y(2) -= gravity;
  }else{
    a = a->getRoot();
    if(a->C.hasTauJoint(a)) {
      double tau; arr Jtau;
      a->C.kinematicsTau(tau, Jtau, a);
      y(2) -= gravity*tau;
      J.setMatrixBlock(-gravity*Jtau, 2, 0);
    } else {
      y(2) -= gravity * a->C.frames.first()->tau;
    }
  }
}

//===========================================================================

void F_NewtonEuler::phi2(arr& y, arr& J, const FrameL& F) {
  CHECK_EQ(order, 2, "");
  CHECK_EQ(F.d0, 3, "");
  CHECK_EQ(F.d1, 1, "");

  //-- get linear and angular accelerations - ACTUALLY change-of-velocity!! (~IMPULSE)
  Value acc = F_LinAngVel()
              .setImpulseInsteadOfAcceleration()
              .setOrder(2)
              .eval(F);

#if 1
  //-- collect total contact forces (actually impulse) without gravity
  Value fo = F_TotalForce(true) // ignore gravity
             .eval({F.elem(-2)}); // ! THIS IS THE MID TIME SLICE !

  //-- collect gravity change-of-velocities -> MULTIPLIES WITH TAU! (this is where tau optimization has major effect!)
  Value grav = F_GravityAcceleration()
               .setImpulseInsteadOfAcceleration()
               .eval({F.elem(-1)}); //END TIME SLICE!
  //-- subtract nominal gravity change-of-velocity from object change-of-velocity
  acc.y -= grav.y;
  acc.J -= grav.J;
#else
  //-- add static and exchange forces
  Value fo = F_TotalForce(false)
             .eval({a}); // ! THIS IS THE MID TIME SLICE !
#endif

  //-- collect mass info (assume diagonal inertia matrix!!)
  double mass=1.;
  arr Imatrix = diag(.1, 3);
  rai::Frame* a = F.elem(-2);
  if(a->inertia) {
    mass = a->inertia->mass;
    Imatrix = conv_mat2arr(a->inertia->matrix);
  }
  arr mass_diag(6);
  for(uint i=0; i<3; i++) mass_diag(i) = mass;
  for(uint i=0; i<3; i++) mass_diag(i+3) = Imatrix(i, i);

#if 1
  arr one_over_mass = ones(6);
  one_over_mass /= mass_diag;
  y = acc.y + one_over_mass % fo.y;
  if(!!J) J = acc.J + one_over_mass % fo.J;
#else
  y = mass_diag % acc.y + fo.y; //THIS IS ACTUALLY AN IMPULSE EQUATION: COLLECTED FORCES fo.y ARE INTERPRETED AS IMPULSE (and that's why gravity should not be mixed in)
  if(!!J) J = mass_diag % acc.J + fo.J;
#endif

}

//===========================================================================

void F_NewtonEuler_DampedVelocities::phi2(arr& y, arr& J, const FrameL& F) {
  CHECK_EQ(order, 1, "");

  //get linear and angular velocities
  Value vel = F_LinAngVel()
              .setOrder(1)
              .eval(F);

  if(useGravity){
    //-- collect gravity change-of-velocities -> MULTIPLIES WITH TAU! (this is where tau optimization has major effect!)
    Value grav = F_GravityAcceleration()
                 .setImpulseInsteadOfAcceleration()
                 .eval({F.elem(-1)}); //END TIME SLICE!
    vel.y -= grav.y;
    vel.J -= grav.J;
  }

  //collect mass info (assume diagonal inertia matrix!!)
  double mass=1.;
  arr Imatrix = diag(.03, 3);
  rai::Frame* a = F.elem(-2);
  if(a->inertia) {
    mass = a->inertia->mass;
    Imatrix = conv_mat2arr(a->inertia->matrix);
  }
  arr mass_diag(6);
  for(uint i=0; i<3; i++) mass_diag(i) = mass;
  for(uint i=0; i<3; i++) mass_diag(i+3) = Imatrix(i, i);

  //collect total contact forces
  Value fo = F_TotalForce(true)
             .eval({F.elem(-2)});

  double friction = .3;
#if 1
  arr one_over_mass = ones(6);
  one_over_mass /= mass_diag;
//  one_over_mass *= 1e1;
  y = friction*vel.y + one_over_mass % fo.y;
  if(!!J) J = friction*vel.J + one_over_mass % fo.J;
#else
  y = (friction*mass_diag) % vel.y + fo.y; //THIS IS ACTUALLY AN IMPULSE EQUATION: COLLECTED FORCES fo.y ARE INTERPRETED AS IMPULSE (and that's why gravity should not be mixed in)
  if(!!J) J = (friction*mass_diag) % vel.J + fo.J;
#endif
}

//===========================================================================

void F_Energy::phi2(arr& y, arr& J, const FrameL& F) {
  if(order==2){
    diffInsteadOfVel=true;
    Feature::phi2(y, J, F);
    diffInsteadOfVel=false;
    return;
  }

  CHECK_EQ(order, 1, "");

  double E=0.;
  Value p, v, w;

  F.elem(-1)->C.kinematicsZero(y, J, 1);

  arr grav = {0.,0.,gravity};

  for(uint i=0;i<F.d1;i++) {
    double mass=1.;
    arr Imatrix = diag(.1, 3);
    rai::Frame *a = F(1,i);
    if(a->inertia) {
      mass = a->inertia->mass;
      Imatrix = 2.*conv_mat2arr(a->inertia->matrix);
      //      rai::Quaternion &rot = f->X.rot;
      //      I=(rot).getMatrix() * f->inertia->matrix * (-rot).getMatrix();
    }

    p = F_Position()
        .eval({F(1,i)});

    v = F_Position()
        .setOrder(1)
        .eval({F(0,i), F(1,i)});

//    w = TM_AngVel()
//        .eval({F(0,i), F(1,i)});

    E += .5*mass*sumOfSqr(v.y);
    E += mass * scalarProduct(grav,p.y); //p(2)=height //(a->X*a->inertia->com).z;
//      E += .5*m*sumOfSqr(w); //(w*(I*w));

    if(!!J) {
      J += (mass*~v.y) * v.J;
      J += (mass*~grav) * p.J;
    }
  }

  y = ARR(E);
}

//===========================================================================

FrameL getShapesAbove(rai::Frame* a) {
  FrameL aboves;
  if(a->shape) aboves.append(a);
  for(rai::Frame* b:a->children) aboves.append(getShapesAbove(b));
  return aboves;
}


//===========================================================================

void F_fex_ForceIsNormal::phi2(arr& y, arr& J, const FrameL& F) {
  //-- from the contact we need force
  Value force = F_fex_Force()
                .eval(F);

  //-- from the geometry we need normal
#if 0
  Value normal = F_PairCollision(F_PairCollision::_normal, true)
                 .eval(F);
#else
  Value normal = F_fex_POASurfaceAvgNormal()
                 .eval(F);
  normalizeWithJac(normal.y, normal.J);
#endif

  //-- force needs to align with normal -> project force along normal
  y = force.y - normal.y*scalarProduct(normal.y, force.y);
  if(!!J){
    J = force.J;
    J -= (normal.y^normal.y)*force.J;
    J -= (normal.y^force.y)*normal.J;
    J -= scalarProduct(normal.y, force.y)*normal.J;
  }
}

void F_fex_ForceIsComplementary::phi2(arr& y, arr& J, const FrameL& F) {
  CHECK_EQ(F.N, 2, "");
  rai::ForceExchange* ex = getContact(F.elem(0), F.elem(1));

  //-- from the contact we need force
  arr force, Jforce;
  ex->kinForce(force, Jforce);

  //-- from the geometry we need distance
  arr d0, Jd0;
  arr d1, Jd1;
  POA_distance(d0, Jd0, ex, false);
  POA_distance(d1, Jd1, ex, true);

  //-- enforce complementarity
//  y.resize(2, 3);
//  if(!!J) J.resize(2, 3, Jd0.d1);

  arr y1 = d0.scalar() * force;
  arr y2 = d1.scalar() * force;
  arr J1 = d0.scalar()*Jforce + force * Jd0;
  arr J2 = d1.scalar()*Jforce + force * Jd1;

  y.setBlockVector(y1, y2);
  J.setBlockMatrix(J1, J2);
}

uint F_fex_ForceIsComplementary::dim_phi2(const FrameL& F) { return 6; }

void F_fex_ForceIsPositive::phi2(arr& y, arr& J, const FrameL& F) {
  //-- from the contact we need force
  Value force = F_fex_Force()
                .eval(F);

  //-- from the geometry we need normal
#if 0
  Value normal = F_PairCollision(F_PairCollision::_normal, true)
                 .eval(F);
#else
  Value normal = F_fex_POASurfaceAvgNormal()
                 .eval(F);
  normalizeWithJac(normal.y, normal.J);
#endif

  //-- force needs to align with normal -> project force along normal
  y.resize(1);
  y.scalar() = -scalarProduct(normal.y, force.y);
  if(!!J) J = - (~normal.y*force.J + ~force.y*normal.J);
}

void F_fex_POASurfaceDistance::phi2(arr& y, arr& J, const FrameL& F){
  if(order>0){  Feature::phi2(y, J, F);  return;  }
  CHECK_EQ(F.N, 2, "");
  rai::ForceExchange* ex = getContact(F.elem(0), F.elem(1));
  rai::Frame *f=0;
  if(leftRight == rai::_left) f = F.elem(0);
  if(leftRight == rai::_right) f = F.elem(1);

  //-- get POA
  arr poa, Jpoa;
  ex->kinPOA(poa, Jpoa);

  //-- evaluate functional
  CHECK(f->shape, "");
  shared_ptr<ScalarFunction> func = f->shape->functional();
  CHECK(func, "");
  arr g;
  double d = (*func)(g, NoArr, poa);

  //-- evaluate Jacobian of POA if f1 moves
  arr Jp;
  f->C.jacobian_pos(Jp, f, poa);

  //-- value & Jacobian
  y.resize(1);
  y.scalar() = d;
  J = ~g * (Jpoa - Jp);
}

void F_fex_POASurfaceNormal::phi2(arr& y, arr& J, const FrameL& F){
  if(order>0){  Feature::phi2(y, J, F);  return;  }
  CHECK_EQ(F.N, 2, "");
  rai::ForceExchange* ex = getContact(F.elem(0), F.elem(1));
  rai::Frame *f=0;
  if(leftRight == rai::_left) f = F.elem(0);
  if(leftRight == rai::_right) f = F.elem(1);

  //-- get POA
  arr poa, Jpoa;
  ex->kinPOA(poa, Jpoa);

  //-- evaluate functional with Hessian
  CHECK(f->shape, "");
  shared_ptr<ScalarFunction> func = f->shape->functional();
  CHECK(func, "");
  arr g,H;
  (*func)(g, H, poa);

  //-- evaluate Jacobian of POA if f1 moves
  arr Jp;
  f->C.jacobian_pos(Jp, f, poa);
  arr Jang;
  f->C.jacobian_angular(Jang, f);

  //-- value & Jacobian
  y = g;
  J = H * (Jpoa - Jp);
  J += crossProduct(Jang, g);
}

void F_fex_POASurfaceNormalsOppose::phi2(arr& y, arr& J, const FrameL& F) {
  Value n1 = F_fex_POASurfaceNormal(rai::_left)
             .eval(F);
  Value n2 = F_fex_POASurfaceNormal(rai::_right)
             .eval(F);

  y = n1.y + n2.y;
  if(!!J) J = n1.J + n2.J;
}

void F_fex_POASurfaceAvgNormal::phi2(arr& y, arr& J, const FrameL& F) {
  Value n1 = F_fex_POASurfaceNormal(rai::_left)
             .eval(F);
  Value n2 = F_fex_POASurfaceNormal(rai::_right)
             .eval(F);

  y = 0.5*(n2.y - n1.y); //normals should oppose, so this should be the avg normal; normal always points 'against obj1'
  if(!!J) J = 0.5*(n2.J - n1.J);
}

void F_fex_POAContactDistances::phi2(arr& y, arr& J, const FrameL& F) {
  if(order>0){  Feature::phi2(y, J, F);  return;  }
  CHECK_EQ(F.N, 2, "");
  rai::Frame *f1 = F.elem(0);
  rai::Frame *f2 = F.elem(1);

  rai::ForceExchange* ex = getContact(f1, f2);

  //-- POA inside objects (eventually on surface!)
  rai::Shape* s1 = f1->shape;
  rai::Shape* s2 = f2->shape;
  CHECK(s1 && s2, "");
  double r1=s1->radius();
  double r2=s2->radius();
  rai::Mesh* m1 = &s1->sscCore();  if(!m1->V.N) { m1 = &s1->mesh(); r1=0.; }
  rai::Mesh* m2 = &s2->sscCore();  if(!m2->V.N) { m2 = &s2->mesh(); r2=0.; }

  rai::Mesh M0;
  M0.setDot();
  rai::Transformation X0=0;
  arr pos, Jpos;
  ex->kinPOA(pos, Jpos);
  X0.pos = pos;

  PairCollision coll1(M0, *m1, X0, s1->frame.ensure_X(), 0., r1);
  PairCollision coll2(M0, *m2, X0, s2->frame.ensure_X(), 0., r2);

  arr Jp1, Jp2;
  f1->C.jacobian_pos(Jp1, f1, coll1.p1);
  f1->C.jacobian_pos(Jp2, f2, coll2.p2);

  arr y1, y2, J1, J2;
  coll1.kinDistance(y1, J1, Jpos, Jp1);
  coll2.kinDistance(y2, J2, Jpos, Jp2);

  y.setBlockVector(y1, y2);
  J.setBlockMatrix(J1, J2);

  if(!!J) checkNan(J);
}

void F_fex_POA_isAtWitnesspoint::phi2(arr& y, arr& J, const FrameL& F) {
  CHECK_EQ(F.N, 2, "");
  rai::ForceExchange* ex = getContact(F.elem(0), F.elem(1));

  arr poa, Jpoa;
  ex->kinPOA(poa, Jpoa);

  Value wit = F_PairCollision((!use2ndObject ? F_PairCollision::_p1 : F_PairCollision::_p2), false)
              .eval(F);

  y = poa - wit.y;
  if(!!J) { J = Jpoa - wit.J; }
}

void F_fex_POAmovesContinuously::phi2(arr& y, arr& J, const FrameL& F) {
  CHECK_EQ(order, 1, "");
  arr cp1, Jcp1;
  arr cp2, Jcp2;
  getContact(F(0,0), F(0,1))->kinPOA(cp1, Jcp1);
  getContact(F(1,0), F(1,1))->kinPOA(cp2, Jcp2);

  y = cp1 - cp2;
  if(!!J) J = Jcp1 - Jcp2;
}

void F_fex_NormalForceEqualsNormalPOAmotion::phi2(arr& y, arr& J, const FrameL& F) {
  CHECK_EQ(order, 1, "");

  Value poavel = F_fex_POA()
                 .setOrder(1)
                 .eval(F);

  Value force = F_fex_Force()
                .eval(F[-1]);

  Value normal = F_PairCollision(F_PairCollision::_normal, true)
                 .eval(F[-1]);

  double forceScaling = 1e1;
  force.y *= forceScaling;
  force.J *= forceScaling;

  //-- force needs to align with normal -> project force along normal
  y.resize(1);
  y.scalar() = scalarProduct(normal.y, force.y - poavel.y);
  if(!!J) J = ~normal.y*(force.J - poavel.J) + ~(force.y - poavel.y) * normal.J;
}

void F_fex_POAzeroRelVel::phi2(arr& y, arr& J, const FrameL& F) {
  CHECK_EQ(order, 1, "");
  rai::ForceExchange* ex = getContact(F(1,0), F(1,1));
#if 0
  POA_rel_vel(y, J, Ktuple, con, true);
#else
  arr v1, Jv1, v2, Jv2;
  POA_vel(v1, Jv1, F, ex, false);
  POA_vel(v2, Jv2, F, ex, true);
  y = v1 - v2;
  if(!!J) J = Jv1 - Jv2;
  if(normalOnly) {
    Value normal = F_PairCollision(F_PairCollision::_normal, true)
                   .eval(F[-1]);
    if(!!J) J = ~normal.y*J + ~y*normal.J;
    y = ARR(scalarProduct(normal.y, y));
  }
#endif
}

void F_fex_ElasticVel::phi2(arr& y, arr& J, const FrameL& F) {
  CHECK_EQ(order, 2, "");
  CHECK_EQ(F.d0, 3, "");
  CHECK_EQ(F.d1, 2, "");
  rai::Frame *f1 = F(1,0);
  rai::Frame *f2 = F(1,1);

  rai::ForceExchange* ex = getContact(f1, f2);
  arr v0, Jv0, v1, Jv1;
  POA_rel_vel(v0, Jv0, F({0,1}), ex, false);
  POA_rel_vel(v1, Jv1, F({1,2}), ex, true);

  //-- from the geometry we need normal
  Value normal = F_PairCollision (F_PairCollision::_normal, false)
                 .eval(F[-2]);

  arr y1, y2, J1, J2;
  f1->C.kinematicsZero(y1, J1, 3);
  //tangential vel
  if(stickiness==1.) {
    y1 = v1 - normal.y*scalarProduct(normal.y, v1);
    if(!!J) J1 = Jv1 - (normal.y*~normal.y*Jv1 + normal.y*~v1*normal.J + scalarProduct(normal.y, v1)*normal.J);
  } else if(stickiness>0.) {
    CHECK_LE(stickiness, 1., "");
    double alpha=1.-stickiness;
    y1 = (v1-alpha*v0) - normal.y*scalarProduct(normal.y, v1-alpha*v0);
    if(!!J) J1 = (Jv1-alpha*Jv0) - (normal.y*~normal.y*(Jv1-alpha*Jv0) + normal.y*~(v1-alpha*v0)*normal.J + scalarProduct(normal.y, (v1-alpha*v0))*normal.J);
  }

  //normal vel
  if(elasticity>0.) {
    y2.resize(1) = scalarProduct(normal.y, v1 + elasticity*v0);
    if(!!J) J2 = ~normal.y*(Jv1+elasticity*Jv0) + ~(v1+elasticity*v0)*normal.J;
  } else if(elasticity==0.) {
    y2.resize(1) = scalarProduct(normal.y, v1);
    if(!!J) J2 = ~normal.y*(Jv1) + ~(v1)*normal.J;
  }

  y.setBlockVector(y1, y2);
  J.setBlockMatrix(J1, J2);
}

void F_fex_NormalVelIsComplementary::phi2(arr& y, arr& J, const FrameL& F) {
  CHECK_EQ(F.d0, 2, "");
  rai::ForceExchange* ex = getContact(F(0,0), F(0,1));

  //-- get the pre and post V:
  arr /*v0, Jv0, */v1, Jv1;
//  POA_rel_vel(v0, Jv0, Ktuple, con, false);
  POA_rel_vel(v1, Jv1, F, ex, true);

  //-- get the force
  arr force, Jforce;
  ex->kinForce(force, Jforce);

  y.resize(1);
  y(0) = scalarProduct(force, v1);
  if(!!J) J = ~force * Jv1 + ~v1 * Jforce;
}

