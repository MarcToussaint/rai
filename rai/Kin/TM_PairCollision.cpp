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

TM_PairCollision::TM_PairCollision(const mlr::KinematicWorld& K, const char* s1, const char* s2, bool negative, bool neglectRadii)
  : i(initIdArg(K, s1)), j(initIdArg(K, s2)),
    negScalar(negative), neglectRadii(neglectRadii){
  CHECK(i>=0,"shape name '" <<s1 <<"' does not exist");
  CHECK(j>=0,"shape name '" <<s2 <<"' does not exist");
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

  PairCollision coll(*m1, *m2, s1->frame.X, s2->frame.X, s1->size(3), s2->size(3));

  if(neglectRadii) coll.rad1=coll.rad2=0.;


  if(!negScalar){
    arr Jp1, Jp2, Jx1, Jx2;
    if(&J){
      K.jacobianPos(Jp1, &s1->frame, coll.p1);
      K.jacobianPos(Jp2, &s2->frame, coll.p2);
      K.axesMatrix(Jx1, &s1->frame);
      K.axesMatrix(Jx2, &s2->frame);
    }
    coll.kinVector(y, J, Jp1, Jp2, Jx1, Jx2);
  }else{
    arr Jp1, Jp2;
    K.jacobianPos(Jp1, &s1->frame, coll.p1);
    K.jacobianPos(Jp2, &s2->frame, coll.p2);
    coll.kinDistance(y, J, Jp1, Jp2);
    y *= -1.;
    if(&J) J *= -1.;
    if(&J) checkNan(J);
  }
}

mlr::String TM_PairCollision::shortTag(const mlr::KinematicWorld &G){
  return STRING("TM_PairCollision"<<(i<0?"WORLD":G.frames(i)->name) <<':' <<(j<0?"WORLD":G.frames(j)->name));
}

