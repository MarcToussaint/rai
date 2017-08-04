#include "proxy.h"

#include <Gui/opengl.h>

//===========================================================================
//
// Proxy
//

mlr::Proxy::Proxy() {
  colorCode = 0;
}

#ifdef MLR_GL
void mlr::Proxy::glDraw(OpenGL& gl){
  glLoadIdentity();
  if(!colorCode){
    if(d>0.) glColor(.8,.2,.2);
    else glColor(1,0,0);
  }else glColor(colorCode);
  glBegin(GL_LINES);
  glVertex3dv(posA.p());
  glVertex3dv(posB.p());
  glEnd();
  mlr::Transformation f;
  f.pos=posA;
  f.rot.setDiff(mlr::Vector(0, 0, 1), posA-posB);
  double GLmatrix[16];
  f.getAffineMatrixGL(GLmatrix);
  glLoadMatrixd(GLmatrix);
  glDisable(GL_CULL_FACE);
  glDrawDisk(.02);
  glEnable(GL_CULL_FACE);

  f.pos=posB;
  f.getAffineMatrixGL(GLmatrix);
  glLoadMatrixd(GLmatrix);
  glDrawDisk(.02);
}
#endif
