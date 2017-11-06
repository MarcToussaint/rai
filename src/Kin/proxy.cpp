#include "proxy.h"

#include <Gui/opengl.h>
#include "kin.h"
#include "frame.h"

//===========================================================================
//
// Proxy
//

mlr::Proxy::Proxy(KinematicWorld &_K) : K(_K) {
  K.proxies.append(this);
}

mlr::Proxy::~Proxy() {
  if(coll) delete coll;
  if(this==K.proxies.last()) K.proxies.resizeCopy(K.proxies.N-1);
  else K.proxies.removeValue(this);
}


void mlr::Proxy::calc_coll(){
  mlr::Shape *s1 = K.frames(a)->shape;
  mlr::Shape *s2 = K.frames(b)->shape;
  coll = new PairCollision(s1->sscCore(), s2->sscCore(), s1->frame.X, s2->frame.X, s1->size(3), s2->size(3));
}

#ifdef MLR_GL
void mlr::Proxy::glDraw(OpenGL& gl){
  if(coll){
    glLoadIdentity();
    coll->glDraw(gl);
  }else{
    glLoadIdentity();
    if(!colorCode){
      if(d>0.) glColor(.8,.2,.2);
      else glColor(1,0,0);
    }else glColor(colorCode);
    glBegin(GL_LINES);
    glVertex3dv(posA.p());
    glVertex3dv(posB.p());
    glEnd();
    glDisable(GL_CULL_FACE);
    mlr::Transformation f;
    f.pos=posA;
    f.rot.setDiff(mlr::Vector(0, 0, 1), posA-posB);
    double GLmatrix[16];
    f.getAffineMatrixGL(GLmatrix);
    glLoadMatrixd(GLmatrix);
    glDrawDisk(.02);

    f.pos=posB;
    f.getAffineMatrixGL(GLmatrix);
    glLoadMatrixd(GLmatrix);
    glDrawDisk(.02);
    glEnable(GL_CULL_FACE);
  }
}
#endif

