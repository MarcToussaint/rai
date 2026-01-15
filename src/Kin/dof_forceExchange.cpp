/*  ------------------------------------------------------------------
    Copyright (c) 2011-2024 Marc Toussaint
    email: toussaint@tu-berlin.de

    This code is distributed under the MIT License.
    Please see <root-path>/LICENSE for details.
    --------------------------------------------------------------  */

#include "dof_forceExchange.h"
#include "../Gui/opengl.h"
#include "../Geo/pairCollision.h"

rai::ForceExchangeDof::ForceExchangeDof(rai::Frame& a, rai::Frame& b, ForceExchangeType _type, const ForceExchangeDof* copy)
  : a(a), b(b), type(_type), scale(1.) {
  CHECK(&a != &b, "");
  CHECK_EQ(&a.C, &b.C, "contact between frames of different configuration!");
  frame=&a;
  dim = getDimFromType();
  a.C.reset_q();
  a.forces.append(this);
  b.forces.append(this);
  a.C.otherDofs.append(this);
  setZero();
  if(copy) {
    copyParameters(copy);
    if(copy->mimic) NIY;

    scale=copy->scale;
    type = copy->type;
    force_to_torque = copy->force_to_torque;
    poa = copy->poa;
    force = copy->force;
    torque = copy->torque;
  }
}

rai::ForceExchangeDof::~ForceExchangeDof() {
  a.C.reset_q();
  a.forces.removeValue(this);
  b.forces.removeValue(this);
  a.C.otherDofs.removeValue(this);
}

void rai::ForceExchangeDof::setZero() {
  a.C._state_q_isGood=false;
  force.resize(3).setZero();
  torque.resize(3).setZero();
  if(type==FXT_poa || type==FXT_poaOnly) {
//    poa = .5*a.getPosition() + .5*b.getPosition();
    poa = b.getPosition();
  } else if(type==FXT_forceZ) {
    force.resize(1).setZero();
    torque.resize(1).setZero();
  } else {
    poa = b.getPosition();
  }
  if(__coll) { delete __coll; __coll=0; }
}

uint rai::ForceExchangeDof::getDimFromType() {
  if(type==FXT_forceZ) return 1;
  else if(type==FXT_force) return 3;
  else if(type==FXT_poaOnly) return 3;
  else return 6;
}

void rai::ForceExchangeDof::setDofs(const arr& q, uint n) {
  if(type==FXT_poa) {
    poa = q({n, n+2+1});
    force = q({n+3, n+5+1});
    torque.resize(3).setZero();
  } else if(type==FXT_poaOnly) {
    poa = q({n, n+2+1});
    force.clear();
    torque.clear();
  } else if(type==FXT_wrench) {
    poa = b.getPosition();
    force = q({n, n+2+1});
    torque = q({n+3, n+5+1});
  } else if(type==FXT_force) {
    poa = b.getPosition();
    force = q({n, n+2+1});
    torque.resize(3).setZero();
  } else if(type==FXT_forceZ) {
    poa = b.getPosition();
    force.resize(1) = q(n);
    torque.resize(1).setZero();
  } else NIY;
  if(scale!=1.) {
    force *= scale;
    torque *= scale;
  }
  if(__coll) { delete __coll; __coll=0; }
}

arr rai::ForceExchangeDof::calcDofsFromConfig() const {
  arr q;
  if(type==FXT_poa) {
    q.resize(6);
    q.setVectorBlock(poa, 0);
    q.setVectorBlock(force/scale, 3);
  } else if(type==FXT_poaOnly) {
    q = poa;
  } else if(type==FXT_wrench) {
    q.resize(6);
    q.setVectorBlock(force/scale, 0);
    q.setVectorBlock(torque/scale, 3);
  } else if(type==FXT_force) {
    q = force/scale;
  } else if(type==FXT_forceZ) {
    q.resize(1).first() = force.scalar();
  } else NIY;
  return q;
}

void rai::ForceExchangeDof::setRandom(uint timeSlices_d1, int verbose) {
  setZero();
//  q0 = calcDofsFromConfig();
  Dof::setRandom(timeSlices_d1, verbose);
}

void rai::ForceExchangeDof::kinPOA(arr& y, arr& J) const {
  a.C.kinematicsZero(y, J, 3);

  if(type==FXT_poa) {
    y = poa;
    if(!!J && active) for(uint i=0; i<3; i++) J.elem(i, qIndex+0+i) = 1.;
  } else if(type==FXT_poaOnly) {
    y = poa;
    if(!!J && active) for(uint i=0; i<3; i++) J.elem(i, qIndex+0+i) = 1.;
  } else if(type==FXT_wrench || type==FXT_force || type==FXT_forceZ) {
    //use b as the POA!! why??
    // b.C.kinematicsPos(y, J, &b);
    a.C.kinematicsPos(y, J, &a);
  } else NIY;
}

void rai::ForceExchangeDof::kinForce(arr& y, arr& J) const {
  a.C.kinematicsZero(y, J, 3);

  if(type==FXT_poa) {
    y = force;
    if(!!J && active) for(uint i=0; i<3; i++) J.elem(i, qIndex+3+i) = scale;
  } else if(type==FXT_poaOnly) {
    //is zero already
  } else if(type==FXT_wrench || type==FXT_force || type==FXT_force) {
    y = force;
    if(!!J && active) for(uint i=0; i<3; i++) J.elem(i, qIndex+0+i) = scale;
  } else if(type==FXT_forceZ) {
    arr z, Jz;
    b.C.kinematicsVec(z, Jz, &b, Vector_z);
    y = force.scalar() * z;
    if(!!J && active) {
      for(uint i=0; i<3; i++) J.elem(i, qIndex) += scale * z.elem(i);
      J += force.scalar()*Jz;
    }
  } else NIY;
}

void rai::ForceExchangeDof::kinTorque(arr& y, arr& J) const {
  a.C.kinematicsZero(y, J, 3);

  if(type==FXT_poa || type==FXT_force || type==FXT_poaOnly) {
    //zero: POA is zero-momentum point
  } else if(type==FXT_forceZ) {
    arr z, Jz;
    b.C.kinematicsVec(z, Jz, &b, Vector_z);
    y = force_to_torque * force.scalar() * z;
    if(!!J) {
      for(uint i=0; i<3; i++) J.elem(i, qIndex) += (force_to_torque*scale) * z.elem(i);
      J += (force_to_torque*force.scalar()) * Jz;
    }
  } else if(type==FXT_wrench) {
    y = torque;
    if(!!J) for(uint i=0; i<3; i++) J.elem(i, qIndex+3+i) = scale;
  } else NIY;
}

rai::PairCollision_CvxCvx* rai::ForceExchangeDof::coll() {
  if(!__coll) {
    rai::Shape* s1 = a.shape.get();
    rai::Shape* s2 = b.shape.get();
    CHECK(s1 && s2, "");
    double r1=s1->coll_cvxRadius;
    double r2=s2->coll_cvxRadius;
    arr m1 = s1->sscCore();
    arr m2 = s2->sscCore();
    __coll = new PairCollision_CvxCvx(m1, m2, a.ensure_X(), b.ensure_X(), r1, r2);
  }
  return __coll;
}

arr gnuplot(const double x) {
  double r = std::sqrt(x);
  double g = x * x * x;
  double b = std::sin(x * 2 * RAI_PI);

  return arr{r, g, b};
}


void rai::ForceExchangeDof::write(Graph& g) const {
  g.add("a", a.name);
  g.add("b", b.name);
  double d = 0.;
  if(__coll) d = -(__coll->distance-__coll->rad1-__coll->rad2);
  g.add("force", force);
  g.add("torque", torque);
  g.add("poa", poa);
  g.add("d", d);
  g.add("compl", sumOfSqr(d * force));
//  <<" type=" <<a_type <<'-' <<b_type <<" dist=" <<getDistance() /*<<" pDist=" <<get_pDistance()*/ <<" y=" <<y <<" l=" <<lagrangeParameter;
}

rai::ForceExchangeDof* rai::getContact(rai::Frame* a, rai::Frame* b, bool raiseErrorIfNonExist) {
  for(rai::ForceExchangeDof* c : a->forces) if(&c->a==a && &c->b==b) return c;
  if(raiseErrorIfNonExist) HALT("can't retrieve contact " <<a->name <<"--" <<b->name);
  return nullptr;
}
