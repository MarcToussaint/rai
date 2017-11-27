#include "contact.h"
#include <Gui/opengl.h>

double mlr::Contact::getDistance(){
  TM_ContactNegDistance map(*this);
  arr y;
  map.phi(y, NoArr, a.K);
  return -y.scalar();
}

TaskMap *mlr::Contact::getTM_ContactNegDistance() const{
  return new TM_ContactNegDistance(*this);
}

void mlr::TM_ContactNegDistance::phi(arr &y, arr &J, const mlr::KinematicWorld &K, int t){
  if(C.a_type==2 && C.b_type!=2){
    HALT("not checked");
    arr ap,an,bp, Jap, Jan, Jbp;
    K.kinematicsPos(ap, Jap, &C.a, C.a_rel);
    K.kinematicsVec(an, Jan, &C.a, C.a_norm);
    K.kinematicsPos(bp, Jbp, &C.b, C.b_rel);

    y.resize(1);
    y = scalarProduct(bp-ap, an) - (C.a_rad+C.b_rad);
    y *= -1.;
    if(&J){
      J = ~(bp-ap)*Jan + ~an*(Jbp-Jap);
      J *= -1.;
    }
  }
  else if(C.b_type==2 && C.a_type!=2){
    HALT("not checked");
    arr ap,bn,bp, Jap, Jbn, Jbp;
    K.kinematicsPos(ap, Jap, &C.a, C.a_rel);
    K.kinematicsVec(bn, Jbn, &C.b, C.b_norm);
    K.kinematicsPos(bp, Jbp, &C.b, C.b_rel);

    y.resize(1);
    y = scalarProduct(bp-ap, bn) - (C.a_rad+C.b_rad);
    y *= -1.;
    if(&J){
      J = ~(bp-ap)*Jbn + ~bn*(Jbp-Jap);
      J *= -1.;
    }
  }
  else{
    arr ap, an, bp, bn, Jap, Jan, Jbp, Jbn;
    K.kinematicsPos(ap, (&J?Jap:NoArr), K.frames(C.a.ID), C.a_rel);
    K.kinematicsVec(an, (&J?Jan:NoArr), K.frames(C.a.ID), C.a_norm);
    K.kinematicsPos(bp, (&J?Jbp:NoArr), K.frames(C.b.ID), C.b_rel);
    K.kinematicsVec(bn, (&J?Jbn:NoArr), K.frames(C.b.ID), C.b_norm);


    y = ARR( .5*scalarProduct(bp-ap, an-bn) - (C.a_rad+C.b_rad) );
    y *= -1.;
    if(&J){
      J = ~(bp-ap)*(Jan-Jbn) + ~(an-bn)*(Jbp-Jap);
      J *= -.5;
    }
  }
}

#ifdef MLR_GL
void mlr::Contact::glDraw(OpenGL& gl){
  mlr::Vector pa = a.X * a_rel;
  mlr::Vector pb = b.X * b_rel;
  mlr::Vector n = .5*((b.X.rot * b_norm) - (a.X.rot * a_norm));

  glLoadIdentity();
  glColor(1., 0., 0., 1.);
  glLineWidth(3.f);
  glDrawProxy(pa.getArr(), pb.getArr(), .05, 0, n.getArr(), a_rad, b_rad);
  glLineWidth(2.f);
  glLoadIdentity();

//    f.pos=.5*(posA+posB);
//    f.getAffineMatrixGL(GLmatrix);
//    glLoadMatrixd(GLmatrix);
//    glDrawText(STRING(a <<'-' <<b <<':' <<d), 0.,0.,0.);
}
#endif
