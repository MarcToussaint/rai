/*  ------------------------------------------------------------------
    Copyright (c) 2011-2020 Marc Toussaint
    email: toussaint@tu-berlin.de

    This code is distributed under the MIT License.
    Please see <root-path>/LICENSE for details.
    --------------------------------------------------------------  */

#include "dataStructures.h"
#include "pcl.h"

#ifdef RAI_PCL

void glDrawPrimitives(void* classP) {
  ((DisplayPrimitives*)classP)->glDraw(NoOpenGL);
}

void DisplayPrimitives::glDraw(struct OpenGL& gl) {
  G.glDraw(gl);
  double tmp[16];
  for(Primitive* p:P) {
    if(!p->X.isZero()) {
      p->X.getAffineMatrixGL(tmp);
      glLoadMatrixd(tmp);
    }
    p->glDraw();
  }
}

void Plane::glDraw() {
  rai::Vector p(nx, ny, nz);
  rai::Vector axis = Vector_z ^ p;
  double angle = acos(p.z / p.length());
  p *= -c/(p*p);
  glLoadIdentity();
  glTranslatef(p.x, p.y, p.z);
  glRotated(180.*angle/RAI_PI, axis.x, axis.y, axis.z);
  glDisable(GL_CULL_FACE);
  glDrawRect(0, 0, 0, 1.);
  glLoadIdentity();
}

void PclCloudView::glDraw() {
  NIY; //conv_PclCloud_ArrCloud(pts,cols,cloud);
  NIY; //glDrawPointCloud(pts,cols);
}

void ArrCloudView::glDraw() {
  arr p=pts.get(), c=cols.get();
  NIY; //glDrawPointCloud(p,c);
}

#endif
