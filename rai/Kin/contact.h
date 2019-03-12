/*  ------------------------------------------------------------------
    Copyright (c) 2017 Marc Toussaint
    email: marc.toussaint@informatik.uni-stuttgart.de

    This code is distributed under the MIT License.
    Please see <root-path>/LICENSE for details.
    --------------------------------------------------------------  */

#pragma once

#include <Core/util.h>
#include "frame.h"
#include "feature.h"

struct PairCollision;

namespace rai {

///Description of a Contact
struct Contact : NonCopyable, GLDrawer {
  Frame &a, &b;

private:
  PairCollision *__coll=0;
public:

  uint qIndex=UINT_MAX;
  arr position;
  arr force;

  Contact(Frame &a, Frame &b, Contact *copyContact=NULL);
  ~Contact();

  void setZero();
  uint qDim() { return 6; }
  void calc_F_from_q(const arr& q, uint n);
  arr calc_q_from_F() const;

  PairCollision *coll();

  void glDraw(OpenGL&);
  void write(ostream& os) const;
};
stdOutPipe(Contact)

struct TM_ContactNegDistance : Feature {
  const Contact& C;
  
  TM_ContactNegDistance(const Contact& contact) : C(contact) {}
  
  void phi(arr& y, arr& J, const rai::KinematicWorld& K);
  virtual uint dim_phi(const rai::KinematicWorld& K) { return 1; }
  virtual rai::String shortTag(const rai::KinematicWorld& K) { return STRING("ContactNegDistance-"<<C.a.name<<'-'<<C.b.name); }
};

} //rai
