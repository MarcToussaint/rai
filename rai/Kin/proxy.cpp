/*  ------------------------------------------------------------------
    Copyright (c) 2019 Marc Toussaint
    email: marc.toussaint@informatik.uni-stuttgart.de

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

rai::Proxy::Proxy() {
}

rai::Proxy::~Proxy() {
  del_coll();
}

void rai::Proxy::copy(const rai::Configuration& K, const rai::Proxy& p) {
  del_coll();
  if(!!K) {
    a = K.frames(p.a->ID); CHECK(a, "");
    b = K.frames(p.b->ID); CHECK(b, "");
  } else a=b=0;
  posA = p.posA;
  posB = p.posB;
  normal = p.normal;
  d = p.d;
  colorCode = p.colorCode;
}

void rai::Proxy::calc_coll(const Configuration& K) {
  CHECK_EQ(&a->C, &K, "");
  CHECK_EQ(&b->C, &K, "");
  rai::Shape* s1 = a->shape;
  rai::Shape* s2 = b->shape;
  CHECK(s1 && s2, "");

  double r1=0.; if(s1->size().N) r1=s1->size().last();
  double r2=0.; if(s2->size().N) r2=s2->size().last();
  rai::Mesh* m1 = &s1->sscCore();  if(!m1->V.N) { m1 = &s1->mesh(); r1=0.; }
  rai::Mesh* m2 = &s2->sscCore();  if(!m2->V.N) { m2 = &s2->mesh(); r2=0.; }

  if(coll) coll.reset();
  coll = std::make_shared<PairCollision>(*m1, *m2, s1->frame.ensure_X(), s2->frame.ensure_X(), r1, r2);

  d = coll->distance-coll->rad1-coll->rad2;
  posA = coll->p1;
  posB = coll->p2;
  normal = coll->normal;
}

typedef rai::Array<rai::Proxy*> ProxyL;

void rai::Proxy::glDraw(OpenGL& gl) {
#ifdef RAI_GL
  if(coll) {
    glLoadIdentity();
    coll->glDraw(gl);
  } else {
    glLoadIdentity();
    if(!colorCode) {
      if(d>0.) glColor(.8, .2, .2);
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

