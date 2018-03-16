#include "contact.h"
#include <Gui/opengl.h>
#include <Geo/pairCollision.h>

double mlr::Contact::getDistance() const{
  TM_ContactNegDistance map(*this);
  arr y;
  map.phi(y, NoArr, a.K);
  return -y.scalar();
}

TaskMap *mlr::Contact::getTM_ContactNegDistance() const{
  return new TM_ContactNegDistance(*this);
}

void mlr::TM_ContactNegDistance::phi(arr &y, arr &J, const mlr::KinematicWorld &K, int t){
  if(C.a_pts.nd==1 && C.b_pts.nd==1){
    arr ap,bp, normal, Jap,Jbp;
    K.kinematicsPos(ap, Jap, &C.a, C.a_pts);
    K.kinematicsPos(bp, Jbp, &C.b, C.b_pts);

    double distance = euclideanDistance(ap, bp);

    normal = p1-p2;
    double l = length(normal);
    if(l<1e-20){ y.resize(1).setZero(); if(&J) J.resize(1,Jap.N).setZero(); return; }
    normal /= l;

    y.resize(1).scalar() = -distance+C.a_rad+C.b_rad;
    if(&J){
      J = Jp2 - Jp1;
      J = ~normal*J;
    }
  }
  if(C.a_pts.nd==1 && C.b_pts.nd==3){
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
    if(l<1e-20){ y.resize(1).setZero(); if(&J) J.resize(1,Jap.N).setZero(); return; }
    normal /= l;

    y.resize(1).scalar() = -distance+C.a_rad+C.b_rad;
    if(&J){
      J = Jp2 - Jp1;
      J = ~normal*J;
    }
  }
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

void mlr::Contact::glDraw(OpenGL& gl){
#ifdef MLR_GL
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
#endif
}

void mlr::Contact::write(std::ostream &os) const{
  os <<a.name <<'-' <<b.name <<" type=" <<a_type <<'-' <<b_type <<" dist=" <<getDistance() <<" pDist=" <<get_pDistance() <<" y=" <<y <<" l=" <<lagrangeParameter;
}
