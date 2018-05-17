/*  ------------------------------------------------------------------
    Copyright (c) 2017 Marc Toussaint
    email: marc.toussaint@informatik.uni-stuttgart.de

    This code is distributed under the MIT License.
    Please see <root-path>/LICENSE for details.
    --------------------------------------------------------------  */

#include "TM_PairCollision.h"
#include "frame.h"
#include <Geo/pairCollision.h>

TM_PairCollision::TM_PairCollision(int _i, int _j, Type _type, bool _neglectRadii)
  : i(_i), j(_j), type(_type), neglectRadii(_neglectRadii) {
}

TM_PairCollision::TM_PairCollision(const rai::KinematicWorld& K, const char* s1, const char* s2, Type _type, bool neglectRadii)
  : i(initIdArg(K, s1)), j(initIdArg(K, s2)),
    type(_type), neglectRadii(neglectRadii) {
  CHECK(i>=0,"shape name '" <<s1 <<"' does not exist");
  CHECK(j>=0,"shape name '" <<s2 <<"' does not exist");
}

TM_PairCollision::~TM_PairCollision(){
  if(coll) delete coll;
}

void TM_PairCollision::phi(arr& y, arr& J, const rai::KinematicWorld& K) {
  rai::Shape *s1 = i<0?NULL: K.frames(i)->shape;
  rai::Shape *s2 = j<0?NULL: K.frames(j)->shape;
  CHECK(s1 && s2,"");
#if 1
  double r1=s1->size(3);
  double r2=s2->size(3);
  rai::Mesh *m1 = &s1->sscCore();  if(!m1->V.N) { m1 = &s1->mesh(); r1=0.; }
  rai::Mesh *m2 = &s2->sscCore();  if(!m2->V.N) { m2 = &s2->mesh(); r2=0.; }
#else
  CHECK(s1->type()==rai::ST_mesh || s1->type()==rai::ST_ssCvx || s1->type()==rai::ST_ssBox,"");
  CHECK(s2->type()==rai::ST_mesh || s2->type()==rai::ST_ssCvx || s2->type()==rai::ST_ssBox,"");
  const rai::Mesh *m1, *m2;
  if(s1->type()==rai::ST_mesh) m1=&s1->mesh(); else m1=&s1->sscCore();
  if(s2->type()==rai::ST_mesh) m2=&s2->mesh(); else m2=&s2->sscCore();
  CHECK(m1->V.N,"");
  CHECK(m2->V.N,"");
#endif
  
  if(coll) delete coll;
  coll = new PairCollision(*m1, *m2, s1->frame.X, s2->frame.X, r1, r2);
  
  if(neglectRadii) coll->rad1=coll->rad2=0.;
  
  if(type==_negScalar) {
    arr Jp1, Jp2;
    K.jacobianPos(Jp1, &s1->frame, coll->p1);
    K.jacobianPos(Jp2, &s2->frame, coll->p2);
    coll->kinDistance(y, J, Jp1, Jp2);
    y *= -1.;
    if(&J) J *= -1.;
    if(&J) checkNan(J);
  }

  if(type==_vector) {
    arr Jp1, Jp2, Jx1, Jx2;
    if(&J) {
      K.jacobianPos(Jp1, &s1->frame, coll->p1);
      K.jacobianPos(Jp2, &s2->frame, coll->p2);
      K.axesMatrix(Jx1, &s1->frame);
      K.axesMatrix(Jx2, &s2->frame);
    }
    coll->kinVector(y, J, Jp1, Jp2, Jx1, Jx2);
  }

  if(type==_normal) {
    arr Jp1, Jp2, Jx1, Jx2;
    if(&J) {
      K.jacobianPos(Jp1, &s1->frame, coll->p1);
      K.jacobianPos(Jp2, &s2->frame, coll->p2);
      K.axesMatrix(Jx1, &s1->frame);
      K.axesMatrix(Jx2, &s2->frame);
    }
    coll->kinNormal(y, J, Jp1, Jp2, Jx1, Jx2);
  }
}

rai::String TM_PairCollision::shortTag(const rai::KinematicWorld &G) {
  return STRING("TM_PairCollision_"<<(i<0?"WORLD":G.frames(i)->name) <<':' <<(j<0?"WORLD":G.frames(j)->name));
}

