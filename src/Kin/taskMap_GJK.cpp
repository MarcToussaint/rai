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


#include "taskMap_GJK.h"

TaskMap_GJK::TaskMap_GJK(const mlr::Shape* s1, const mlr::Shape* s2, bool exact, bool negative) : exact(exact), negScalar(negative){
  CHECK(s1 && s2,"");
  i = s1->index;
  j = s2->index;
}

TaskMap_GJK::TaskMap_GJK(const mlr::KinematicWorld& W, const char* s1, const char* s2, bool exact, bool negative) : exact(exact), negScalar(negative){
  CHECK(s1 && s2,"");
  mlr::Shape *s;
  s=W.getShapeByName(s1); CHECK(s,"shape name '" <<s1 <<"' does not exist"); i=s->index;
  s=W.getShapeByName(s2); CHECK(s,"shape name '" <<s2 <<"' does not exist"); j=s->index;
}

TaskMap_GJK::TaskMap_GJK(const mlr::KinematicWorld& W, const Graph& specs, bool exact) : exact(exact){
  Node *it;
  if((it=specs["sym2"])){ auto name=it->get<mlr::String>(); auto *s=W.getShapeByName(name); CHECK(s,"shape name '" <<name <<"' does not exist"); i=s->index; }
  if((it=specs["sym3"])){ auto name=it->get<mlr::String>(); auto *s=W.getShapeByName(name); CHECK(s,"shape name '" <<name <<"' does not exist"); j=s->index; }
//  if((it=specs["vec1"])) vec1 = mlr::Vector(it->get<arr>());  else vec1.setZero();
//  if((it=specs["vec2"])) vec2 = mlr::Vector(it->get<arr>());  else vec2.setZero();
}

void TaskMap_GJK::phi(arr& v, arr& J, const mlr::KinematicWorld& W, int t){
  mlr::Shape *s1 = i<0?NULL: W.shapes(i);
  mlr::Shape *s2 = j<0?NULL: W.shapes(j);
  CHECK(s1 && s2,"");
  CHECK(s1->sscCore.V.N,"");
  CHECK(s2->sscCore.V.N,"");
  mlr::Vector p1, p2, e1, e2;
  GJK_point_type pt1, pt2;

  GJK_sqrDistance(s1->sscCore, s2->sscCore, s1->X, s2->X, p1, p2, e1, e2, pt1, pt2);
  //  if(d2<1e-10) LOG(-1) <<"zero distance";
  arr y1, J1, y2, J2;

  W.kinematicsPos(y1, (&J?J1:NoArr), s1->body, s1->body->X.rot/(p1-s1->body->X.pos));
  W.kinematicsPos(y2, (&J?J2:NoArr), s2->body, s2->body->X.rot/(p2-s2->body->X.pos));
  v = y1 - y2;
  if(&J){
    J = J1 - J2;
    if(exact){
      if((pt1==GJK_vertex && pt2==GJK_face) || (pt1==GJK_face && pt2==GJK_vertex)){
        arr vec, Jv, n = v/length(v);
        J = n*(~n*J);
        if(pt1==GJK_vertex) W.kinematicsVec(vec, Jv, s2->body, s2->body->X.rot/(p1-p2));
        if(pt2==GJK_vertex) W.kinematicsVec(vec, Jv, s1->body, s1->body->X.rot/(p1-p2));
        J += Jv;
      }
      if(pt1==GJK_edge && pt2==GJK_edge){
        arr vec, Jv, n, a, b;
        n = v/length(v);
        J = n*(~n*J);

        W.kinematicsVec(vec, Jv, s1->body, s1->body->X.rot/e1);
        a=conv_vec2arr(e1);
        b=conv_vec2arr(e2);
        double ab=scalarProduct(a,b);
        J += (a-b*ab) * (1./(1.-ab*ab)) * (~v*(b*~b -eye(3,3))) * Jv;

        W.kinematicsVec(vec, Jv, s2->body, s2->body->X.rot/e2);
        a=conv_vec2arr(e2);
        b=conv_vec2arr(e1);
        J += (a-b*ab) * (1./(1.-ab*ab)) * (~v*(b*~b -eye(3,3))) * Jv;
      }
      if((pt1==GJK_vertex && pt2==GJK_edge) || (pt1==GJK_edge && pt2==GJK_vertex)){
        arr vec, Jv, n;
        if(pt1==GJK_vertex) n=conv_vec2arr(e2); else n=conv_vec2arr(e1);
        J = J - n*(~n*J);
        if(pt1==GJK_vertex) W.kinematicsVec(vec, Jv, s2->body, s2->body->X.rot/(p1-p2));
        if(pt2==GJK_vertex) W.kinematicsVec(vec, Jv, s1->body, s1->body->X.rot/(p1-p2));
        J += n*(~n*Jv);
      }
    }
  }
  //reduce by radii
  double l2=sumOfSqr(v), l=sqrt(l2);
  double fac = (l-s1->size(3)-s2->size(3))/l;
  if(&J){
    arr d_fac = (1.-(l-s1->size(3)-s2->size(3))/l)/l2 *(~v)*J;
    J = J*fac + v*d_fac;
  }
  v *= fac;

  if(negScalar){
    if(&J) J = ~(v/(-l))*J;
    v = ARR(-l);
  }
//  CHECK_ZERO(l2-d2, 1e-6,"");
}

