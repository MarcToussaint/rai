/*  ------------------------------------------------------------------
    Copyright (c) 2011-2024 Marc Toussaint
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

#define RAI_USE_FUNCTIONAL_NORMALS

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

  rai::PairCollision coll(M0, *m, X0, s->frame.ensure_X(), 0., r);

  arr Jp;
  K.jacobian_pos(Jp, &s->frame, coll.p1);

  coll.kinDistance(y, J, Jpos, Jp);
}

void POA_rel_vel2(arr& y, arr& J, const FrameL& F, rai::ForceExchange* ex, bool after_or_before) {
  CHECK_EQ(F.d0, 3, "");
  CHECK_EQ(F.d1, 2, "");
  CHECK_EQ(F(1, 0), &ex->a, "");
  CHECK_EQ(F(1, 1), &ex->b, "");

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
arr POA_rel_vel(const FrameL& F, rai::ForceExchange* ex, bool after_or_before) {
  CHECK_EQ(F.d0, 2, "");
  CHECK_EQ(F.d1, 2, "");
  if(after_or_before) {
    CHECK_EQ(F(0, 0), &ex->a, "");
    CHECK_EQ(F(0, 1), &ex->b, "");
  } else {
    CHECK_EQ(F(1, 0), &ex->a, "");
    CHECK_EQ(F(1, 1), &ex->b, "");
  }

  arr cp, cpJ;
  ex->kinPOA(cp, cpJ);
  cp.J() = cpJ;

  // p1, p2 are the CENTERS! of the frame a and b
  // v1, v2 are the CENTER velocities of the frame a and b
  arr p1 = F_Position().eval({&ex->a});
  arr p2 = F_Position().eval({&ex->b});
  arr v1, v2;
  v1 = F_Position().setOrder(1).eval({F(0, 0), F(1, 0)});
  v2 = F_Position().setOrder(1).eval({F(0, 1), F(1, 1)});
  arr w1, w2;
  w1 = F_AngVel().eval({{2, 1}, {F(0, 0), F(1, 0)}});
  w2 = F_AngVel().eval({{2, 1}, {F(0, 1), F(1, 1)}});

  arr vc1 = v1 - crossProduct(w1, cp - p1);
//  arr Jvc1 = v1.J() - skew(w1) * (cpJ - p1.J()) + skew(cp-p1) * w1.J();
  arr vc2 = v2 - crossProduct(w2, cp - p2);
//  arr Jvc2 = v2.J() - skew(w2) * (cpJ - p2.J()) + skew(cp-p2) * w2.J();

  arr y = vc1 - vc2;
  return y;
}

//3-dim feature: the POA velocities (V)
arr POA_vel(const FrameL& F, rai::ForceExchange* ex, bool b_or_a) {
  CHECK_GE(F.d0, 2, "");
  CHECK_GE(F.d1, 2, "");
  CHECK_EQ(F(1, 0), &ex->a, "");
  CHECK_EQ(F(1, 1), &ex->b, "");

  FrameL ff = {F(0, 0), F(1, 0)};
  if(b_or_a) ff = {F(0, 1), F(1, 1)};

  //POA
  arr poa, Jpoa;
  ex->kinPOA(poa, Jpoa);
  if(!!Jpoa) poa.J() = Jpoa;

  arr p = F_Position() .eval({ff(1)});
  arr v = F_LinVel() .eval(ff);
  arr w = F_AngVel() .eval(ff);

  arr y = v - crossProduct(w, poa - p);
  return y;
//  if(!!J){
//    arr Jtmp = v.J() - skew(w) * (Jpoa - p.J()) + skew(poa-p) * w.J();
//    CHECK_ZERO(maxDiff(J, Jtmp), 1e-10, "");
//  }
}

void shapeFunction(double& x, double& dx);

//===========================================================================

void F_fex_POA::phi2(arr& y, arr& J, const FrameL& F) {
  if(order>0) {  Feature::phi2(y, J, F);  return;  }
  CHECK_GE(F.N, 2, "");
  rai::ForceExchange* ex = getContact(F.elem(0), F.elem(1));
  ex->kinPOA(y, J);
}

void F_fex_Force::phi2(arr& y, arr& J, const FrameL& F) {
  if(order>0) {  Feature::phi2(y, J, F);  return;  }
  CHECK_EQ(F.N, 2, "");
  rai::ForceExchange* ex = getContact(F.elem(0), F.elem(1), false);
  if(!ex) { F.elem(0)->C.kinematicsZero(y, J, dim_phi2(F)); return; }
  ex->kinForce(y, J);
}

void F_fex_Torque::phi2(arr& y, arr& J, const FrameL& F) {
  if(order>0) {  Feature::phi2(y, J, F);  return;  }
  CHECK_EQ(F.N, 2, "");
  rai::ForceExchange* ex = getContact(F.elem(0), F.elem(1));
  ex->kinTorque(y, J);
}

void F_fex_Wrench::phi2(arr& y, arr& J, const FrameL& F) {
  if(order>0) {  Feature::phi2(y, J, F);  return;  }
  CHECK_EQ(F.N, 2, "");
  rai::ForceExchange* ex = getContact(F.elem(0), F.elem(1));
  arr y1, y2, J1, J2;
  ex->kinForce(y1, J1);
  ex->kinTorque(y2, J2);
  y.setBlockVector(y1, y2);
  J.setBlockMatrix(J1, J2);
}

//===========================================================================

void F_HingeXTorque::phi2(arr& y, arr& J, const FrameL& F) {
  if(order>0) {  Feature::phi2(y, J, F);  return;  }
  CHECK_EQ(F.N, 2, "");
  rai::Frame* f1 = F.elem(0);
  rai::Frame* f2 = F.elem(1);
  CHECK(f2->joint, "second frame needs to be a joint");
  CHECK_EQ(f2->joint->type, rai::JT_hingeX, "second frame needs to be a joint")
  rai::ForceExchange* ex = getContact(f1, f2);
  arr y2, J2;
  ex->kinTorque(y2, J2);

  auto axis = F_Vector(Vector_x)
              .eval({f2});

  y.resize(1) = scalarProduct(y2, axis);
  if(!!J) J = ~y2 * axis.J() + ~axis * J2;
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
    CHECK(a->inertia, "can't accumulate gravity force for zero-mass object '" <<a->name <<"'")
    double mass=1.;
    if(a->inertia) mass = a->inertia->mass;
    force(2) += gravity * mass;
  }

  //-- collect contacts and signs FOR ALL shapes attached to this link
  rai::Array<rai::ForceExchange*> contacts;
  arr signs;
  FrameL linkF;
  linkF.append(a);
  a->getRigidSubFrames(linkF, false);
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

void F_GravityAcceleration::phi2(arr& y, arr& J, const FrameL& F) {
  CHECK_EQ(F.N, 1, "");

  rai::Frame* a = F.scalar();
  a->C.kinematicsZero(y, J, 6);
  if(!impulseInsteadOfAcceleration) {
    y(2) -= gravity;
  } else {
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
  arr acc = F_LinAngVel()
            .setImpulseInsteadOfAcceleration()
            .setOrder(2)
            .eval(F);

#if 1
  //-- collect total contact forces (actually impulse) without gravity
  arr fo = F_TotalForce(true) // ignore gravity
           .eval({F.elem(-2)}); // ! THIS IS THE MID TIME SLICE !

  if(useGravity) {
    //-- collect gravity change-of-velocities -> MULTIPLIES WITH TAU! (this is where tau optimization has major effect!)
    arr grav = F_GravityAcceleration()
               .setImpulseInsteadOfAcceleration()
               .eval({F.elem(-1)}); //END TIME SLICE!

    //-- subtract nominal gravity change-of-velocity from object change-of-velocity
    acc -= grav;
    //acc.J() -= grav.J(); AUTODIFF
  }
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
  y = acc + fo % one_over_mass;
  grabJ(y, J);
  //if(!!J) J = acc.J() + one_over_mass % fo.J(); AUTODIFF
#else
  y = mass_diag % acc + fo; //THIS IS ACTUALLY AN IMPULSE EQUATION: COLLECTED FORCES fo ARE INTERPRETED AS IMPULSE (and that's why gravity should not be mixed in)
  if(!!J) J = mass_diag % acc.J() + fo.J();
#endif

}

//===========================================================================

void F_NewtonEuler_DampedVelocities::phi2(arr& y, arr& J, const FrameL& F) {
  CHECK_EQ(order, 1, "");

  //get linear and angular velocities
  arr vel = F_LinAngVel()
            .setOrder(1)
            .eval(F);

  if(useGravity) {
    //-- collect gravity change-of-velocities -> MULTIPLIES WITH TAU! (this is where tau optimization has major effect!)
    arr grav = F_GravityAcceleration()
               .setImpulseInsteadOfAcceleration()
               .eval({F.elem(-1)}); //END TIME SLICE!
    vel -= grav;
    vel.J() -= grav.J();
  }

  //collect mass info (assume diagonal inertia matrix!!)
  rai::Frame* a = F.elem(-2);
  CHECK(a->inertia, "F_NewtonEuler needs inertia defined for '" <<a->name <<"'");
  CHECK(a->inertia->matrix.isDiagonal(), "can only handle diagonal");
  arr mass_diag(6);
  mass_diag({0, 2}) = a->inertia->mass;
  mass_diag({3, 5}) = a->inertia->matrix.getDiag();

  //collect total contact forces
  arr fo = F_TotalForce(true)
           .eval({F.elem(-2)});

  double friction = .1;
  a->ats->get<double>(friction, "friction");
#if 1
  arr one_over_mass = ones(6);
  one_over_mass /= mass_diag;
//  one_over_mass *= 1e1;
  y = friction*vel + one_over_mass % fo;
  if(!!J) J = friction*vel.J() + one_over_mass % fo.J();
#else
  y = (friction*mass_diag) % vel + fo; //THIS IS ACTUALLY AN IMPULSE EQUATION: COLLECTED FORCES fo ARE INTERPRETED AS IMPULSE (and that's why gravity should not be mixed in)
  if(!!J) J = (friction*mass_diag) % vel.J() + fo.J();
#endif
}

//===========================================================================

void F_Energy::phi2(arr& y, arr& J, const FrameL& F) {
  if(order==2) {
    diffInsteadOfVel=true;
    Feature::phi2(y, J, F);
    diffInsteadOfVel=false;
    return;
  }

  CHECK_EQ(order, 1, "");

  double E=0.;
  arr p, v, w;

  F.elem(-1)->C.kinematicsZero(y, J, 1);

  arr grav = {0., 0., gravity};

  for(uint i=0; i<F.d1; i++) {
    double mass=1.;
    arr Imatrix = diag(.1, 3);
    rai::Frame* a = F(1, i);
    if(a->inertia) {
      mass = a->inertia->mass;
      Imatrix = 2.*conv_mat2arr(a->inertia->matrix);
      //      rai::Quaternion &rot = f->X.rot;
      //      I=(rot).getMatrix() * f->inertia->matrix * (-rot).getMatrix();
    }

    p = F_Position()
        .eval({F(1, i)});

    v = F_Position()
        .setOrder(1)
        .eval({F(0, i), F(1, i)});

//    w = TM_AngVel()
//        .eval({F(0,i), F(1,i)});

    E += .5*mass*sumOfSqr(v);
    E += mass * scalarProduct(grav, p); //p(2)=height //(a->X*a->inertia->com).z;
//      E += .5*m*sumOfSqr(w); //(w*(I*w));

    if(!!J) {
      J += (mass*~v) * v.J();
      J += (mass*~grav) * p.J();
    }
  }

  y = arr{E};
}

//===========================================================================

FrameL getShapesAbove(rai::Frame* a) {
  FrameL aboves;
  if(a->shape) aboves.append(a);
  for(rai::Frame* b:a->children) aboves.append(getShapesAbove(b));
  return aboves;
}

//===========================================================================

arr F_fex_ForceIsNormal::phi(const FrameL& F) {
  rai::ForceExchange* ex = getContact(F.elem(0), F.elem(1), false);
  if(!ex) { arr y; F.elem(0)->C.kinematicsZero(y, y.J(), dim_phi2(F)); return y; }

  //-- from the contact we need force
  arr force = F_fex_Force()
              .eval(F);

  //-- from the geometry we need normal
#ifdef RAI_USE_FUNCTIONAL_NORMALS
  arr normal = F_fex_POASurfaceAvgNormal()
               .eval(F);
  op_normalize(normal);
#else
  Value normal = F_PairCollision(F_PairCollision::_normal, true)
                 .eval(F);
#endif

  //-- force needs to align with normal -> project force along normal
  arr y = force - normal*(~normal * force);
  return y;
}

arr F_fex_ForceInFrictionCone::phi(const FrameL& F) {
  rai::ForceExchange* ex = getContact(F.elem(0), F.elem(1), false);
  if(!ex) { arr y; F.elem(0)->C.kinematicsZero(y, y.J(), dim_phi2(F)); return y; }

  //-- from the contact we need force
  arr force = F_fex_Force() .eval(F);
  op_normalize(force); //optional

  //-- from the geometry we need normal
  arr normal = F_fex_POASurfaceAvgNormal() .eval(F);
  op_normalize(normal);

  //-- friction cone
#if 0
  arr nf = normal*(~normal * force);
  arr a = force - nf;
  arr a2 = ~a * a;
  arr b2 = ~force * nf;
  return a2 - (mu*mu)*b2;
#else //works only with normalized force (see above)
  return mu - (~normal * force);
#endif
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
  rai::ForceExchange* ex = getContact(F.elem(0), F.elem(1), false);
  if(!ex) { F.elem(0)->C.kinematicsZero(y, J, dim_phi2(F)); return; }

  //-- from the contact we need force
  arr force = F_fex_Force()
              .eval(F);

  //-- from the geometry we need normal
#ifdef RAI_USE_FUNCTIONAL_NORMALS
  arr normal = F_fex_POASurfaceAvgNormal()
               .eval(F);
  op_normalize(normal);
#else
  Value normal = F_PairCollision(F_PairCollision::_normal, true)
                 .eval(F);
#endif

  //-- force needs to align with normal -> project force along normal
  y = - (~normal * force); //-scalarProduct(normal, force);
  grabJ(y, J);
//  if(!!J) J = - (~normal*force.J() + ~force*normal.J()); AUTODIFF
}

void F_fex_POASurfaceDistance::phi2(arr& y, arr& J, const FrameL& F) {
  if(order>0) {  Feature::phi2(y, J, F);  return;  }
  CHECK_EQ(F.N, 2, "");
  rai::ForceExchange* ex = getContact(F.elem(0), F.elem(1), false);
  if(!ex) { F.elem(0)->C.kinematicsZero(y, J, dim_phi2(F)); return; }
  rai::Frame* f=0;
  if(leftRight == rai::_left) f = F.elem(0);
  if(leftRight == rai::_right) f = F.elem(1);

  //-- get POA
  arr poa, Jpoa;
  ex->kinPOA(poa, Jpoa);

  //-- evaluate functional
  CHECK(f->shape, "the frame '" <<f->name <<"' needs to have a shape");
  shared_ptr<ScalarFunction> func = f->shape->functional();
  CHECK(func, "the frame '" <<f->name <<"' needs to have a functional shape");
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

void F_fex_POASurfaceNormal::phi2(arr& y, arr& J, const FrameL& F) {
  if(order>0) {  Feature::phi2(y, J, F);  return;  }
  CHECK_EQ(F.N, 2, "");
  rai::ForceExchange* ex = getContact(F.elem(0), F.elem(1), false);
  if(!ex) { F.elem(0)->C.kinematicsZero(y, J, dim_phi2(F)); return; }
  rai::Frame* f=0;
  if(leftRight == rai::_left) f = F.elem(0);
  if(leftRight == rai::_right) f = F.elem(1);

  //-- get POA
  arr poa, Jpoa;
  ex->kinPOA(poa, Jpoa);

  //-- evaluate functional with Hessian
  CHECK(f->shape, "");
  shared_ptr<ScalarFunction> func = f->shape->functional();
  CHECK(func, "");
  arr g, H;
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
  arr n1 = F_fex_POASurfaceNormal(rai::_left)
           .eval(F);
  arr n2 = F_fex_POASurfaceNormal(rai::_right)
           .eval(F);

  y = n1 + n2;
  if(!!J) J = n1.J() + n2.J();
}

void F_fex_POASurfaceAvgNormal::phi2(arr& y, arr& J, const FrameL& F) {
  arr n1 = F_fex_POASurfaceNormal(rai::_left)
           .eval(F);
  arr n2 = F_fex_POASurfaceNormal(rai::_right)
           .eval(F);

  y = 0.5*(n2 - n1); //normals should oppose, so this should be the avg normal; normal always points 'against obj1'
  grabJ(y, J);
//  if(!!J) J = 0.5*(n2.J() - n1.J()); AUTODIFF
}

void F_fex_POAContactDistances::phi2(arr& y, arr& J, const FrameL& F) {
  if(order>0) {  Feature::phi2(y, J, F);  return;  }
  CHECK_EQ(F.N, 2, "");
  rai::Frame* f1 = F.elem(0);
  rai::Frame* f2 = F.elem(1);

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

  rai::PairCollision coll1(M0, *m1, X0, s1->frame.ensure_X(), 0., r1);
  rai::PairCollision coll2(M0, *m2, X0, s2->frame.ensure_X(), 0., r2);

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

  arr wit = F_PairCollision((!use2ndObject ? F_PairCollision::_p1 : F_PairCollision::_p2), false)
            .eval(F);

  y = poa - wit;
  if(!!J) { J = Jpoa - wit.J(); }
}

void F_fex_NormalForceEqualsNormalPOAmotion::phi2(arr& y, arr& J, const FrameL& F) {
  CHECK_EQ(order, 1, "");

  arr poavel = F_fex_POA()
               .setOrder(1)
               .eval(F);

  arr force = F_fex_Force()
              .eval(F[-1]);

  arr normal = F_PairCollision(F_PairCollision::_normal, true)
               .eval(F[-1]);

  double forceScaling = 1e1;
  force *= forceScaling;
  force.J() *= forceScaling;

  //-- force needs to align with normal -> project force along normal
  y.resize(1);
  y.scalar() = scalarProduct(normal, force - poavel);
  if(!!J) J = ~normal*(force.J() - poavel.J()) + ~(force - poavel) * normal.J();
}

void F_fex_POA_PositionRel::phi2(arr& y, arr& J, const FrameL& F) {
  if(order>0) {  Feature::phi2(y, J, F);  return;  }
  CHECK_EQ(F.N, 2, "");
  rai::ForceExchange* ex = getContact(F.elem(0), F.elem(1));

  //rai::Frame *f1 = F.elem(0);
  rai::Frame* f2 = F.elem(1);
  if(b_or_a) { /*f1=F.elem(1);*/ f2=F.elem(0); }

  arr y1, y2, J1, J2;
  ex->kinPOA(y1, J1);
  f2->C.kinematicsPos(y2, J2, f2);
  arr Rinv = ~(f2->ensure_X().rot.getArr());
  y = Rinv * (y1 - y2);
  if(!!J) {
    arr A;
    f2->C.jacobian_angular(A, f2);
    J = Rinv * (J1 - J2 - crossProduct(A, y1 - y2));
  }
}

arr F_fex_POAzeroRelVel::phi(const FrameL& F) {
  CHECK_EQ(order, 1, "");
  rai::ForceExchange* ex = getContact(F(1, 0), F(1, 1));
#if 1
  arr y = POA_rel_vel(F, ex, false);
  return y;
#else
  arr v1 = POA_vel(F, ex, false);
  arr v2 = POA_vel(F, ex, true);
  arr y = v1 - v2;
  if(normalOnly) {
    arr normal = F_PairCollision(F_PairCollision::_normal, true)
                 .eval(F[-1]);
//    if(!!J) J = ~normal*J + ~y*normal.J();
    y = ~normal * y;
  }
  return y;
#endif
}

arr F_fex_ElasticVel::phi(const FrameL& F) {
  CHECK_EQ(order, 2, "");
  CHECK_EQ(F.d0, 3, "");
  CHECK_EQ(F.d1, 2, "");
  rai::Frame* f1 = F(1, 0);
  rai::Frame* f2 = F(1, 1);

  rai::ForceExchange* ex = getContact(f1, f2);
  arr v0 = POA_rel_vel(F({0, 1}), ex, false);
  arr v1 = POA_rel_vel(F({1, 2}), ex, true);

  //-- from the geometry we need normal
  arr normal = F_PairCollision(F_PairCollision::_normal, false)
               .eval(F[-2]);

  arr y1, y2;
//  f1->C.kinematicsZero(y1, J1, 3);
  //tangential vel
  if(stickiness==1.) {
    y1 = v1 - normal*(~normal * v1);
//    if(!!J) J1 = Jv1 - (normal*~normal*Jv1 + normal*~v1*normal.J() + scalarProduct(normal, v1)*normal.J());
  } else if(stickiness>0.) {
    CHECK_LE(stickiness, 1., "");
    double alpha=1.-stickiness;
    y1 = (v1-alpha*v0) - normal*(~normal * (v1-alpha*v0));
//    if(!!J) J1 = (Jv1-alpha*Jv0) - (normal*~normal*(Jv1-alpha*Jv0) + normal*~(v1-alpha*v0)*normal.J() + scalarProduct(normal, (v1-alpha*v0))*normal.J());
  }

  //normal vel
  if(elasticity>0.) {
    y2 = ~normal * (v1 + elasticity*v0);
//    if(!!J) J2 = ~normal*(Jv1+elasticity*Jv0) + ~(v1+elasticity*v0)*normal.J();
  } else if(elasticity==0.) {
    y2 = ~normal * v1;
//    if(!!J) J2 = ~normal*(Jv1) + ~(v1)*normal.J();
  }

  arr y;
  y.setBlockVector(y1, y2);
  return y;
//  J.setBlockMatrix(J1, J2);
}

arr F_fex_NormalVelIsComplementary::phi(const FrameL& F) {
  CHECK_EQ(F.d0, 2, "");
  rai::ForceExchange* ex = getContact(F(0, 0), F(0, 1));

  //-- get the pre and post V:
//  POA_rel_vel(v0, Jv0, Ktuple, con, false);
  arr v1 = POA_rel_vel(F, ex, true);

  //-- get the force
  arr force, Jforce;
  ex->kinForce(force, Jforce);
  force.J() = Jforce;

  arr y = ~force * v1;
  return y;
}

//===========================================================================

arr F_PushRadiusPrior::phi(const FrameL& F) {
//  CHECK_EQ(F.N, 4, "");

  rai::Frame* stick = F.elem(0);
  rai::Frame* obj = F.elem(1);
  rai::Frame* target = (F.d1==3?F.elem(2):0);

  //poa
  arr p;
  if(rai::getContact(stick, obj, false)) {
    p = F_fex_POA() .eval({stick, obj});
  } else {
    p = F_Position() .eval({stick});
  }

  //object center
  arr c = F_Position() .eval({obj});

  arr dir;
  if(F.N==3) { //target is given as 3rd frame
    CHECK_EQ(order, 0, "");
    dir = -c;
    dir += F_Position() .eval({target});
  } else { //target is implicit in object velocity
    CHECK_EQ(order, 1, "");
    //object velocity
    dir = F_Position() .setOrder(1) .eval({F(0, 1), F(1, 1)});
  }

  op_normalize(dir, 1e-3);

  arr y = rad * dir - (c-p);
  return y;
}

arr F_PushAligned::phi(const FrameL& F) {
  CHECK_EQ(F.N, 3, "");

  //poa
  arr a;
  if(rai::getContact(F.elem(0), F.elem(1), false)) {
    a = F_fex_POA() .eval({F.elem(0), F.elem(1)});
  } else {
    a = F_Position() .eval({F.elem(0)});
  }
  arr b = F_Position() .eval({F.elem(1)});
  arr c = F_Position() .eval({F.elem(2)});

  arr y;
  op_crossProduct(y, a-b, c-b);
  return y;
}

arr F_PushSide::phi(const FrameL& F) {
  CHECK_EQ(F.N, 3, "");

  //poa
  arr a;
  if(rai::getContact(F.elem(0), F.elem(1), false)) {
    a = F_fex_POA() .eval({F.elem(0), F.elem(1)});
  } else {
    a = F_Position() .eval({F.elem(0)});
  }
  arr b = F_Position() .eval({F.elem(1)});
  arr c = F_Position() .eval({F.elem(2)});

  arr ab = (a-b);
  arr cb = (c-b);
//  op_normalize(ab, 1e-4);
  op_normalize(cb, 1e-4);

  arr y = ~ab * cb;
  return y;
}
