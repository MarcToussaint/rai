/*  ------------------------------------------------------------------
    Copyright (c) 2011-2024 Marc Toussaint
    email: toussaint@tu-berlin.de

    This code is distributed under the MIT License.
    Please see <root-path>/LICENSE for details.
    --------------------------------------------------------------  */

#include "proxy.h"
#include "kin.h"
#include "frame.h"
#include "../Gui/opengl.h"

//===========================================================================
//
// Proxy
//

void rai::Proxy::copy(const rai::Configuration& C, const rai::Proxy& p) {
  collision.reset();
  if(!!C) {
    a = C.frames.elem(p.a->ID); CHECK(a, "");
    b = C.frames.elem(p.b->ID); CHECK(b, "");
  } else a=b=0;
  posA = p.posA;
  posB = p.posB;
  normal = p.normal;
  d = p.d;
  colorCode = p.colorCode;
//  if(p.collision) collision = p.collision;
}

void rai::Proxy::calc_coll() {
  CHECK(a && b, "ill-defined proxies!");
  rai::Shape* s1 = a->shape.get();
  rai::Shape* s2 = b->shape.get();
  CHECK(s1 && s2, "");

  double r1 = s1->coll_cvxRadius;
  double r2 = s2->coll_cvxRadius;
  arr& m1 = s1->sscCore();
  arr& m2 = s2->sscCore();

  if(collision) collision.reset();
  collision = make_shared<PairCollision_CvxCvx>(m1, m2, a->ensure_X(), b->ensure_X(), r1, r2);

  d = collision->distance-collision->rad1-collision->rad2;
  normal = collision->normal;
  posA = collision->p1;
  posB = collision->p2;
  if(collision->rad1>0.) posA -= collision->rad1*normal;
  if(collision->rad2>0.) posB += collision->rad2*normal;
}

typedef rai::Array<rai::Proxy*> ProxyL;

void rai::Proxy::write(std::ostream& os, bool brief) const {
  os <<" ("
     <<a->name <<")-("
     <<b->name
     <<") [" <<a->ID <<',' <<b->ID <<"] \td=" <<d;
  if(!brief)
    os <<" |A-B|=" <<(posB-posA).length()
       //        <<" d^2=" <<(posB-posA).lengthSqr()
       <<" v=" <<(posB-posA)
       <<" normal=" <<normal
       <<" posA=" <<posA
       <<" posB=" <<posB;
}

