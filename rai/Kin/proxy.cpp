/*  ------------------------------------------------------------------
    Copyright (c) 2011-2020 Marc Toussaint
    email: toussaint@tu-berlin.de

    This code is distributed under the MIT License.
    Please see <root-path>/LICENSE for details.
    --------------------------------------------------------------  */

#include "proxy.h"
#include "kin.h"
#include "frame.h"
#include "../Gui/opengl.h"

//===========================================================================
//
// Proxy
//

void rai::Proxy::copy(const rai::Configuration& C, const rai::Proxy& p) {
  collision.reset();
  if(!!C) {
    a = C.frames(p.a->ID); CHECK(a, "");
    b = C.frames(p.b->ID); CHECK(b, "");
  } else a=b=0;
  posA = p.posA;
  posB = p.posB;
  normal = p.normal;
  d = p.d;
  colorCode = p.colorCode;
//  if(p.collision) collision = p.collision;
}

void rai::Proxy::calc_coll() {
  rai::Shape* s1 = a->shape;
  rai::Shape* s2 = b->shape;
  CHECK(s1 && s2, "");

  double r1=0.; if(s1->size().N) r1=s1->size().last();
  double r2=0.; if(s2->size().N) r2=s2->size().last();
  rai::Mesh* m1 = &s1->sscCore();  if(!m1->V.N) { m1 = &s1->mesh(); r1=0.; }
  rai::Mesh* m2 = &s2->sscCore();  if(!m2->V.N) { m2 = &s2->mesh(); r2=0.; }

  if(collision) collision.reset();
  collision = make_shared<PairCollision>(*m1, *m2, s1->frame.ensure_X(), s2->frame.ensure_X(), r1, r2);

  d = collision->distance-collision->rad1-collision->rad2;
  normal = collision->normal;
  posA = collision->p1;
  posB = collision->p2;
  if(collision->rad1>0.) posA -= collision->rad1*normal;
  if(collision->rad2>0.) posB += collision->rad2*normal;
}

typedef rai::Array<rai::Proxy*> ProxyL;

void rai::Proxy::glDraw(OpenGL& gl) {
#ifdef RAI_GL
  if(collision) {
    glLoadIdentity();
    collision->glDraw(gl);
  } else {
    glLoadIdentity();
    if(!colorCode) {
      if(d>0.) glColor(.2, .8, .2);
      else glColor(1, 0, 0);
    } else glColor(colorCode);
    glBegin(GL_LINES);
    glVertex3dv(posA.p());
    glVertex3dv(posB.p());
    glEnd();
    glDisable(GL_CULL_FACE);
    rai::Transformation f;
    f.pos=posA;
    f.rot.setDiff(rai::Vector(0, 0, 1), posA-posB);
    double GLmatrix[16];
    f.getAffineMatrixGL(GLmatrix);
    glLoadMatrixd(GLmatrix);
    glDrawDisk(.02);

    f.pos=posB;
    f.getAffineMatrixGL(GLmatrix);
    glLoadMatrixd(GLmatrix);
    glDrawDisk(.02);

#if 0 //write text
    f.pos=.5*(posA+posB);
    f.getAffineMatrixGL(GLmatrix);
    glLoadMatrixd(GLmatrix);
    glDrawText(STRING(a->name <<'-' <<b->name <<':' <<d), 0., 0., 0.);
#endif

    glEnable(GL_CULL_FACE);
  }
#endif
}

void rai::Proxy::write(std::ostream& os, bool brief) const {
  os <<" ("
     <<a->name <<")-("
     <<b->name
     <<") [" <<a->ID <<',' <<b->ID <<"] \td=" <<d;
  if(!brief)
    os <<" |A-B|=" <<(posB-posA).length()
       //        <<" d^2=" <<(posB-posA).lengthSqr()
       <<" v=" <<(posB-posA)
       <<" normal=" <<normal
       <<" posA=" <<posA
       <<" posB=" <<posB;
}

