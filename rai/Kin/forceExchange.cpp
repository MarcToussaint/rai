/*  ------------------------------------------------------------------
    Copyright (c) 2011-2020 Marc Toussaint
    email: toussaint@tu-berlin.de

    This code is distributed under the MIT License.
    Please see <root-path>/LICENSE for details.
    --------------------------------------------------------------  */

#include "forceExchange.h"
#include "../Gui/opengl.h"
#include "../Geo/pairCollision.h"

rai::ForceExchange::ForceExchange(rai::Frame& a, rai::Frame& b, rai::ForceExchange* copyContact)
  : a(a), b(b), type(FXT_torque) {
  CHECK(&a != &b, "");
  CHECK_EQ(&a.C, &b.C, "contact between frames of different configuration!");
  a.C.reset_q();
  a.forces.append(this);
  b.forces.append(this);
  a.C.forces.append(this);
  setZero();
  if(copyContact) {
    poa = copyContact->poa;
    force = copyContact->force;
    torque = copyContact->torque;
  }
}

rai::ForceExchange::~ForceExchange() {
  a.C.reset_q();
  a.forces.removeValue(this);
  b.forces.removeValue(this);
  a.C.forces.removeValue(this);
}

void rai::ForceExchange::setZero() {
//  a_rel.setZero(); b_rel.setZero(); a_norm.setZero(); b_norm.setZero(); a_rad=b_rad=0.; a_type=b_type=1;
  force.resize(3).setZero();
  torque.resize(3).setZero();
  poa = (.5*(a.ensure_X().pos + b.ensure_X().pos)).getArr();
  if(__coll) { delete __coll; __coll=0; }
}

void rai::ForceExchange::calc_F_from_q(const arr& q, uint n) {
  if(type==FXT_poa){
    poa = q({n, n+2});
    force = q({n+3, n+5});
    torque.resize(3).setZero();
  }else if(type==FXT_torque){
    poa = b.getPosition();
    force = q({n, n+2});
    torque = q({n+3, n+5});
  }
  if(__coll) { delete __coll; __coll=0; }
}

arr rai::ForceExchange::calc_q_from_F() const {
  arr q(6);
  if(type==FXT_poa){
    q.setVectorBlock(poa, 0);
    q.setVectorBlock(force, 3);
  }else if(type==FXT_torque){
    q.setVectorBlock(force, 0);
    q.setVectorBlock(torque, 3);
  }
  return q;
}

void rai::ForceExchange::kinPOA(arr& y, arr& J) const {
  a.C.kinematicsZero(y, J, 3);

  if(type==FXT_poa){
    y = poa;
    if(!!J) for(uint i=0; i<3; i++) J.elem(i, qIndex+0+i) = 1.;
  }else if(type==FXT_torque){
    //use b as the POA!!
    b.C.kinematicsPos(y, J, &b);
  }
}

void rai::ForceExchange::kinForce(arr& y, arr& J) const {
  a.C.kinematicsZero(y, J, 3);

  if(type==FXT_poa){
    y = force;
    if(!!J) for(uint i=0; i<3; i++) J.elem(i, qIndex+3+i) = 1.;
  }else if(type==FXT_torque){
    y = force;
    if(!!J) for(uint i=0; i<3; i++) J.elem(i, qIndex+0+i) = 1.;
  }
}

void rai::ForceExchange::kinTorque(arr& y, arr& J) const {
  a.C.kinematicsZero(y, J, 3);

  if(type==FXT_poa){
    //zero: POA is zero-momentum point
  }else if(type==FXT_torque){
    y = torque;
    if(!!J) for(uint i=0; i<3; i++) J.elem(i, qIndex+3+i) = 1.;
  }
}

PairCollision* rai::ForceExchange::coll() {
  if(!__coll) {
    rai::Shape* s1 = a.shape;
    rai::Shape* s2 = b.shape;
    CHECK(s1 && s2, "");
    double r1=s1->size(-1);
    double r2=s2->size(-1);
    rai::Mesh* m1 = &s1->sscCore();  if(!m1->V.N) { m1 = &s1->mesh(); r1=0.; }
    rai::Mesh* m2 = &s2->sscCore();  if(!m2->V.N) { m2 = &s2->mesh(); r2=0.; }
    __coll = new PairCollision(*m1, *m2, s1->frame.ensure_X(), s2->frame.ensure_X(), r1, r2);
  }
  return __coll;
}

void rai::ForceExchange::glDraw(OpenGL& gl) {
#ifdef RAI_GL
  glLoadIdentity();
  glLineWidth(3.f);
  glDrawDiamond(poa(0), poa(1), poa(2), .02, .02, .02);
  glBegin(GL_LINES);
  glColor(1., 0., 1., 1.);
  glVertex3dv(poa.p);
  glVertex3dv((poa+torque).p);
  glColor(1., 1., 1., 1.);
  glVertex3dv(poa.p);
  glVertex3dv((poa+force).p);
  glEnd();
  glLineWidth(1.f);

  glBegin(GL_LINES);
  glVertex3dv(&a.ensure_X().pos.x);
  glVertex3dv(poa.p);
  glColor(.8, .5, .8, 1.);
  glVertex3dv(poa.p);
  glVertex3dv(&b.ensure_X().pos.x);
  glEnd();

  glLoadIdentity();

//    f.pos=.5*(posA+posB);
//    f.getAffineMatrixGL(GLmatrix);
//    glLoadMatrixd(GLmatrix);
//    glDrawText(STRING(a <<'-' <<b <<':' <<d), 0.,0.,0.);
#endif
}

void rai::ForceExchange::write(std::ostream& os) const {
  os <<a.name <<'-' <<b.name;
  double d = 0.;
  if(__coll) {
    d = -(__coll->distance-__coll->rad1-__coll->rad2);
  }
  os <<" f=" <<force <<" d=" <<d <<"   compl=" <<sumOfSqr(d * force);
//  <<" type=" <<a_type <<'-' <<b_type <<" dist=" <<getDistance() /*<<" pDist=" <<get_pDistance()*/ <<" y=" <<y <<" l=" <<lagrangeParameter;
}

