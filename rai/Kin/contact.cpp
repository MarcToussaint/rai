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
    position = copyContact->position;
    force = copyContact->force;
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
  double d = 0.;
  if(__coll){
    d = -(__coll->distance-__coll->rad1-__coll->rad2);
  }
  os <<" f=" <<force <<" d=" <<d <<"   compl=" <<sumOfSqr(d * force);
//  <<" type=" <<a_type <<'-' <<b_type <<" dist=" <<getDistance() /*<<" pDist=" <<get_pDistance()*/ <<" y=" <<y <<" l=" <<lagrangeParameter;
}
