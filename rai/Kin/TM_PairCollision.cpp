/*  ------------------------------------------------------------------
    Copyright 2016 Marc Toussaint
    email: marc.toussaint@informatik.uni-stuttgart.de
    
    This program is free software: you can redistribute it and/or modify
    it under the terms of the GNU General Public License as published by
    the Free Software Foundation, either version 3 of the License, or (at
    your option) any later version. This program is distributed without
    any warranty. See the GNU General Public License for more details.
    You should have received a COPYING file of the full GNU General Public
    License along with this program. If not, see
    <http://www.gnu.org/licenses/>
    --------------------------------------------------------------  */


#include "TM_PairCollision.h"
#include "frame.h"
#include <Geo/pairCollision.h>

TM_PairCollision::TM_PairCollision(int _i, int _j, bool negative, bool neglectRadii)
  : i(_i), j(_j), negScalar(negative), neglectRadii(neglectRadii){
}

TM_PairCollision::TM_PairCollision(const mlr::KinematicWorld& K, const char* s1, const char* s2, bool negative, bool neglectRadii) : negScalar(negative), neglectRadii(neglectRadii){
  CHECK(s1 && s2,"");
  mlr::Frame *s;
  s=K.getFrameByName(s1); CHECK(s,"shape name '" <<s1 <<"' does not exist"); i=s->ID;
  s=K.getFrameByName(s2); CHECK(s,"shape name '" <<s2 <<"' does not exist"); j=s->ID;
}


void TM_PairCollision::phi(arr& y, arr& J, const mlr::KinematicWorld& K, int t){
  mlr::Shape *s1 = i<0?NULL: K.frames(i)->shape;
  mlr::Shape *s2 = j<0?NULL: K.frames(j)->shape;
  CHECK(s1 && s2,"");
  CHECK(s1->type()==mlr::ST_mesh || s1->type()==mlr::ST_ssCvx || s1->type()==mlr::ST_ssBox,"");
  CHECK(s2->type()==mlr::ST_mesh || s2->type()==mlr::ST_ssCvx || s2->type()==mlr::ST_ssBox,"");
  const mlr::Mesh *m1, *m2;
  if(s1->type()==mlr::ST_mesh) m1=&s1->mesh(); else m1=&s1->sscCore();
  if(s2->type()==mlr::ST_mesh) m2=&s2->mesh(); else m2=&s2->sscCore();
  CHECK(m1->V.N,"");
  CHECK(m2->V.N,"");

#if 1

  PairCollision coll(*m1, *m2, s1->frame.X, s2->frame.X, s1->size(3), s2->size(3));

  if(neglectRadii) coll.rad1=coll.rad2=0.;

  arr Jp1, Jp2, Jx1, Jx2;
  if(&J){
    K.jacobianPos(Jp1, &s1->frame, coll.p1);
    K.jacobianPos(Jp2, &s2->frame, coll.p2);
    K.axesMatrix(Jx1, &s1->frame);
    K.axesMatrix(Jx2, &s2->frame);
  }

  if(!negScalar){
    coll.kinVector(y, J, Jp1, Jp2, Jx1, Jx2);
  }else{
#if 0
    uint n=K.getJointStateDimension();
    arr JS1(coll.simplex1.d0, 3, n), JS2(coll.simplex2.d0, 3, n);
    for(uint i=0;i<coll.simplex1.d0;i++) K.jacobianPos(JS1[i](), &s1->frame, coll.simplex1[i]);
    for(uint i=0;i<coll.simplex2.d0;i++) K.jacobianPos(JS2[i](), &s2->frame, coll.simplex2[i]);
    coll.kinDistance2(y, J, JS1, JS2);
#else
    K.jacobianPos(Jp1, &s1->frame, coll.p1);
    K.jacobianPos(Jp2, &s2->frame, coll.p2);
    coll.kinDistance(y, J, Jp1, Jp2);
#endif
    y *= -1.;
    if(&J) J *= -1.;
    if(&J) checkNan(J);
  }

#else

  mlr::Vector p1, p2, e1, e2;
  GJK_point_type pt1, pt2;

  double d2 = GJK_sqrDistance(*m1, *m2, s1->frame.X, s2->frame.X, p1, p2, e1, e2, pt1, pt2);
  //  if(d2<1e-10) LOG(-1) <<"zero distance";
  arr y1, J1, y2, J2;

  double penetrating = false;
  if(d2<1e-10){
    penetrating = true;
    arr dir, pos;
    //THIS IS PRETTY SLOW... (make the support function depend on transform?)
    mlr::Mesh M1(*m1); s1->frame.X.applyOnPointArray(M1.V);
    mlr::Mesh M2(*m2); s2->frame.X.applyOnPointArray(M2.V);
    double penetration = GJK_libccd_penetration(dir, pos, M1, M2);
//    cout <<"penetration=" <<penetration <<endl;
    mlr::Vector cen = pos; //.5*(p1+p2);
    p1 = cen + .5*penetration*mlr::Vector(dir);
    p2 = cen - .5*penetration*mlr::Vector(dir);
    pt1=GJK_vertex; pt2=GJK_face;
  }

  W.kinematicsPos(y1, (&J?J1:NoArr), &s1->frame, s1->frame.X.rot/(p1-s1->frame.X.pos));
  W.kinematicsPos(y2, (&J?J2:NoArr), &s2->frame, s2->frame.X.rot/(p2-s2->frame.X.pos));
  v = y1 - y2;
  if(&J){
    J = J1 - J2;
    if(exact){
      if((pt1==GJK_vertex && pt2==GJK_face) || (pt1==GJK_face && pt2==GJK_vertex)){
        arr vec, Jv, n = v/length(v);
        J = n*(~n*J);
        if(pt1==GJK_vertex) W.kinematicsVec(vec, Jv, &s2->frame, s2->frame.X.rot/(p1-p2));
        if(pt2==GJK_vertex) W.kinematicsVec(vec, Jv, &s1->frame, s1->frame.X.rot/(p1-p2));
        J += Jv;
      }
      if(pt1==GJK_edge && pt2==GJK_edge){
        arr vec, Jv, n, a, b;
        n = v/length(v);
        J = n*(~n*J);

        W.kinematicsVec(vec, Jv, &s1->frame, s1->frame.X.rot/e1);
        a=conv_vec2arr(e1);
        b=conv_vec2arr(e2);
        double ab=scalarProduct(a,b);
        J += (a-b*ab) * (1./(1.-ab*ab)) * (~v*(b*~b -eye(3,3))) * Jv;

        W.kinematicsVec(vec, Jv, &s2->frame, s2->frame.X.rot/e2);
        a=conv_vec2arr(e2);
        b=conv_vec2arr(e1);
        J += (a-b*ab) * (1./(1.-ab*ab)) * (~v*(b*~b -eye(3,3))) * Jv;
      }
      if((pt1==GJK_vertex && pt2==GJK_edge) || (pt1==GJK_edge && pt2==GJK_vertex)){
        arr vec, Jv, n;
        if(pt1==GJK_vertex) n=conv_vec2arr(e2); else n=conv_vec2arr(e1);
        J = J - n*(~n*J);
        if(pt1==GJK_vertex) W.kinematicsVec(vec, Jv, &s2->frame, s2->frame.X.rot/(p1-p2));
        if(pt2==GJK_vertex) W.kinematicsVec(vec, Jv, &s1->frame, s1->frame.X.rot/(p1-p2));
        J += n*(~n*Jv);
      }
    }
  }

  //reduce by radii
  double rad=0.;
  if(s1->type()==mlr::ST_ssCvx) rad += s1->size(3);
  if(s2->type()==mlr::ST_ssCvx) rad += s2->size(3);
  double l2=sumOfSqr(v), l=sqrt(l2);
  double fac = (l-rad)/l;
  if(&J){
    arr d_fac = (1.-(l-rad)/l)/l2 *(~v)*J;
    J = J*fac + v*d_fac;
  }
  v *= fac;

  if(negScalar){
    if(penetrating){
      if(&J) J = ~(y/l)*J;
      y = ARR(l);
    }else{
      if(&J) J = ~(y/(-l))*J;
      y = ARR(-l);
    }
  }
//  CHECK_ZERO(l2-d2, 1e-6,"");
#endif
}

mlr::String TM_PairCollision::shortTag(const mlr::KinematicWorld &G){
  return STRING("TM_PairCollision"<<(i<0?"WORLD":G.frames(i)->name) <<':' <<(j<0?"WORLD":G.frames(j)->name));
}

