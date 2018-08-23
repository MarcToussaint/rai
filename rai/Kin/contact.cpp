/*  ------------------------------------------------------------------
    Copyright (c) 2017 Marc Toussaint
    email: marc.toussaint@informatik.uni-stuttgart.de

    This code is distributed under the MIT License.
    Please see <root-path>/LICENSE for details.
    --------------------------------------------------------------  */

#include "contact.h"
#include <Gui/opengl.h>
#include <Geo/pairCollision.h>

rai::Contact::Contact(rai::Frame &a, rai::Frame &b, rai::Contact *copyContact)
  : a(a), b(b) {
  CHECK(&a != &b,"");
  CHECK_EQ(&a.K, &b.K, "contact between frames of different configuration!");
  a.K.reset_q();
  a.contacts.append(this);
  b.contacts.append(this);
  a.K.contacts.append(this);
  setZero();
  if(copyContact){
//    a_rel = copyContact->a_rel;
//    b_rel = copyContact->b_rel;
//    a_norm = copyContact->a_norm;
//    b_norm = copyContact->b_norm;
//    a_rad = copyContact->a_rad;
//    b_rad = copyContact->b_rad;
//    a_type = copyContact->a_type;
//    b_type = copyContact->b_type;
    force = copyContact->force;
    soft = copyContact->soft;
  }
}

rai::Contact::~Contact() {
  a.K.reset_q();
  a.contacts.removeValue(this);
  b.contacts.removeValue(this);
  a.K.contacts.removeValue(this);
}

void rai::Contact::setZero(){
//  a_rel.setZero(); b_rel.setZero(); a_norm.setZero(); b_norm.setZero(); a_rad=b_rad=0.; a_type=b_type=1;
  force.resize(3).setZero();
  position = (.5*(a.X.pos + b.X.pos)).getArr();
  if(__coll){ delete __coll; __coll=0; }
}

void rai::Contact::calc_F_from_q(const arr &q, uint n) {
  position = q({n,n+2});
  force = q({n+3,n+5});
  if(__coll){ delete __coll; __coll=0; }
}

arr rai::Contact::calc_q_from_F() const {
  arr q(6);
  q.setVectorBlock(position,0);
  q.setVectorBlock(force, 3);
  return q;
}

PairCollision *rai::Contact::coll(){
  if(!__coll){
    rai::Shape *s1 = a.shape;
    rai::Shape *s2 = b.shape;
    CHECK(s1 && s2,"");
    double r1=s1->size(3);
    double r2=s2->size(3);
    rai::Mesh *m1 = &s1->sscCore();  if(!m1->V.N) { m1 = &s1->mesh(); r1=0.; }
    rai::Mesh *m2 = &s2->sscCore();  if(!m2->V.N) { m2 = &s2->mesh(); r2=0.; }
    __coll = new PairCollision(*m1, *m2, s1->frame.X, s2->frame.X, r1, r2);
  }
  return __coll;
}

void rai::Contact::setFromPairCollision(PairCollision &col){
//  a_rel = a.X / rai::Vector(col.p1 - col.rad1 * col.normal);
//  b_rel = b.X / rai::Vector(col.p2 + col.rad2 * col.normal);
//  a_norm = a.X.rot / rai::Vector(-col.normal);
//  b_norm = b.X.rot / rai::Vector(col.normal);
//  a_type = col.simplex1.d0;
//  b_type = col.simplex2.d0;
//  a_rad = col.rad1; //a.shape->size(3);
//  b_rad = col.rad2; //b.shape->size(3);
  y = -(col.distance-col.rad1-col.rad2);
}

#if 0
double rai::Contact::getDistance() const {
  TM_ContactNegDistance map(*this);
  arr y;
  map.phi(y, NoArr, a.K);
  return -y.scalar();
}

Feature *rai::Contact::getTM_ContactNegDistance() const {
  return new TM_ContactNegDistance(*this);
}

void rai::TM_ContactNegDistance::phi(arr &y, arr &J, const rai::KinematicWorld &K) {
#if 0
  if(C.a_pts.nd==1 && C.b_pts.nd==1) {
    arr ap,bp, normal, Jap,Jbp;
    K.kinematicsPos(ap, Jap, &C.a, C.a_pts);
    K.kinematicsPos(bp, Jbp, &C.b, C.b_pts);
    
    double distance = euclideanDistance(ap, bp);
    
    normal = p1-p2;
    double l = length(normal);
    if(l<1e-20) { y.resize(1).setZero(); if(&J) J.resize(1,Jap.N).setZero(); return; }
    normal /= l;
    
    y.resize(1).scalar() = -distance+C.a_rad+C.b_rad;
    if(&J) {
      J = Jp2 - Jp1;
      J = ~normal*J;
    }
  }
  if(C.a_pts.nd==1 && C.b_pts.nd==3) {
    arr ap, bp, normal, bps(3,3), Jap,Jn;
    
    K.kinematicsPos(ap, Jap, &C.a, C.a_pts);
    K.kinematicsPos(bps[0], NoArr, &C.b, C.b_pts[0]);
    K.kinematicsPos(bps[1], NoArr, &C.b, C.b_pts[1]);
    K.kinematicsPos(bps[2], NoArr, &C.b, C.b_pts[2]);
    coll_1on3(bp, normal, ap, bps);
    K.kinematicsVec(normal, Jn, C.b, C.b_norm);
    
    double distance = euclideanDistance(ap, bp);
    
    arr normal = p1-p2;
    double l = length(normal);
    if(l<1e-20) { y.resize(1).setZero(); if(&J) J.resize(1,Jap.N).setZero(); return; }
    normal /= l;
    
    y.resize(1).scalar() = -distance+C.a_rad+C.b_rad;
    if(&J) {
      J = Jp2 - Jp1;
      J = ~normal*J;
    }
  }
#endif
  if(C.a_type==2 && C.b_type!=2) {
    HALT("not checked");
    arr ap,an,bp, Jap, Jan, Jbp;
    K.kinematicsPos(ap, Jap, &C.a, C.a_rel);
    K.kinematicsVec(an, Jan, &C.a, C.a_norm);
    K.kinematicsPos(bp, Jbp, &C.b, C.b_rel);
    
    y.resize(1);
    y = scalarProduct(bp-ap, an) - (C.a_rad+C.b_rad);
    y *= -1.;
    if(&J) {
      J = ~(bp-ap)*Jan + ~an*(Jbp-Jap);
      J *= -1.;
    }
  } else if(C.b_type==2 && C.a_type!=2) {
    HALT("not checked");
    arr ap,bn,bp, Jap, Jbn, Jbp;
    K.kinematicsPos(ap, Jap, &C.a, C.a_rel);
    K.kinematicsVec(bn, Jbn, &C.b, C.b_norm);
    K.kinematicsPos(bp, Jbp, &C.b, C.b_rel);
    
    y.resize(1);
    y = scalarProduct(bp-ap, bn) - (C.a_rad+C.b_rad);
    y *= -1.;
    if(&J) {
      J = ~(bp-ap)*Jbn + ~bn*(Jbp-Jap);
      J *= -1.;
    }
  } else {
    arr ap, an, bp, bn, Jap, Jan, Jbp, Jbn;
    K.kinematicsPos(ap, (&J?Jap:NoArr), K.frames(C.a.ID), C.a_rel);
    K.kinematicsVec(an, (&J?Jan:NoArr), K.frames(C.a.ID), C.a_norm);
    K.kinematicsPos(bp, (&J?Jbp:NoArr), K.frames(C.b.ID), C.b_rel);
    K.kinematicsVec(bn, (&J?Jbn:NoArr), K.frames(C.b.ID), C.b_norm);
    
    y = ARR(.5*scalarProduct(bp-ap, an-bn) - (C.a_rad+C.b_rad));
    y *= -1.;
    if(&J) {
      J = ~(bp-ap)*(Jan-Jbn) + ~(an-bn)*(Jbp-Jap);
      J *= -.5;
    }
  }
}
#endif

void rai::Contact::glDraw(OpenGL& gl) {
#ifdef RAI_GL
  glLoadIdentity();
  glColor(1., 0., 1., 1.);
  glLineWidth(3.f);
  glDrawDiamond(position(0), position(1), position(2), .02, .02, .02);
  glBegin(GL_LINES);
  glVertex3dv(position.p);
  glVertex3dv((position+force).p);
  glEnd();
  glLineWidth(1.f);
  glLoadIdentity();
  
//    f.pos=.5*(posA+posB);
//    f.getAffineMatrixGL(GLmatrix);
//    glLoadMatrixd(GLmatrix);
//    glDrawText(STRING(a <<'-' <<b <<':' <<d), 0.,0.,0.);
#endif
}

void rai::Contact::write(std::ostream &os) const {
  os <<a.name <<'-' <<b.name;
  os <<" f=" <<force <<" d=" <<y <<"   compl=" <<sumOfSqr(y * force);
//  <<" type=" <<a_type <<'-' <<b_type <<" dist=" <<getDistance() /*<<" pDist=" <<get_pDistance()*/ <<" y=" <<y <<" l=" <<lagrangeParameter;
}
