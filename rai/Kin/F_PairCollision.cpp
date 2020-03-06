/*  ------------------------------------------------------------------
    Copyright (c) 2019 Marc Toussaint
    email: marc.toussaint@informatik.uni-stuttgart.de

    This code is distributed under the MIT License.
    Please see <root-path>/LICENSE for details.
    --------------------------------------------------------------  */

#include "F_PairCollision.h"
#include "frame.h"
#include "../Geo/pairCollision.h"

TM_PairCollision::TM_PairCollision(int _i, int _j, Type _type, bool _neglectRadii)
  : i(_i), j(_j), type(_type), neglectRadii(_neglectRadii) {
}

TM_PairCollision::TM_PairCollision(const rai::Configuration& K, const char* s1, const char* s2, Type _type, bool neglectRadii)
  : i(initIdArg(K, s1)), j(initIdArg(K, s2)),
    type(_type), neglectRadii(neglectRadii) {
  CHECK_GE(i, 0, "shape name '" <<s1 <<"' does not exist");
  CHECK_GE(j, 0, "shape name '" <<s2 <<"' does not exist");
}

TM_PairCollision::~TM_PairCollision() {
}

void TM_PairCollision::phi(arr& y, arr& J, const rai::Configuration& C) {
  rai::Shape* s1 = i<0?nullptr: C.frames(i)->shape;
  rai::Shape* s2 = j<0?nullptr: C.frames(j)->shape;
  CHECK(s1 && s2, "");
  double r1=s1->radius();
  double r2=s2->radius();
  rai::Mesh* m1 = &s1->sscCore();  if(!m1->V.N) { m1 = &s1->mesh(); r1=0.; }
  rai::Mesh* m2 = &s2->sscCore();  if(!m2->V.N) { m2 = &s2->mesh(); r2=0.; }
  if(!m1->V.N) m1->V = zeros(1, 3);
  if(!m2->V.N) m2->V = zeros(1, 3);

  coll.reset();
  coll = make_unique<PairCollision>(*m1, *m2, s1->frame.ensure_X(), s2->frame.ensure_X(), r1, r2);

  if(neglectRadii) coll->rad1=coll->rad2=0.;

  if(type==_negScalar) {
    arr Jp1, Jp2;
    C.jacobian_pos(Jp1, &s1->frame, coll->p1);
    C.jacobian_pos(Jp2, &s2->frame, coll->p2);
    coll->kinDistance(y, J, Jp1, Jp2);
    y *= -1.;
    if(!!J) J *= -1.;
    if(!!J) checkNan(J);
  } else {
    arr Jp1, Jp2, Jx1, Jx2;
    if(!!J) {
      C.jacobian_pos(Jp1, &s1->frame, coll->p1);
      C.jacobian_pos(Jp2, &s2->frame, coll->p2);
      C.jacobian_angular(Jx1, &s1->frame);
      C.jacobian_angular(Jx2, &s2->frame);
    }
    if(type==_vector) coll->kinVector(y, J, Jp1, Jp2, Jx1, Jx2);
    if(type==_normal) coll->kinNormal(y, J, Jp1, Jp2, Jx1, Jx2);
    if(type==_center) coll->kinCenter(y, J, Jp1, Jp2, Jx1, Jx2);
    if(type==_p1) coll->kinPointP1(y, J, Jp1, Jp2, Jx1, Jx2);
    if(type==_p2) coll->kinPointP2(y, J, Jp1, Jp2, Jx1, Jx2);
  }
}

rai::String TM_PairCollision::shortTag(const rai::Configuration& G) {
  return STRING("PairCollision-"<<(i<0?"WORLD":G.frames(i)->name) <<'-' <<(j<0?"WORLD":G.frames(j)->name));
}

rai::Graph TM_PairCollision::getSpec(const rai::Configuration& K) {
  return rai::Graph({ {"feature", "dist"}, {"o1", K.frames(i)->name}, {"o2", K.frames(j)->name}});
}
