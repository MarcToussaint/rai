/*  ------------------------------------------------------------------
    Copyright (c) 2011-2020 Marc Toussaint
    email: toussaint@tu-berlin.de

    This code is distributed under the MIT License.
    Please see <root-path>/LICENSE for details.
    --------------------------------------------------------------  */

#pragma once

#include "frame.h"
#include "feature.h"
#include "../Core/util.h"

struct PairCollision;

namespace rai {

///Description of a ForceExchange
struct ForceExchange : NonCopyable, GLDrawer {
  Frame& a, &b;

 private:
  PairCollision* __coll=0;
 public:

  uint qIndex=UINT_MAX;
  arr position;
  arr force;

  ForceExchange(Frame& a, Frame& b, ForceExchange* copyContact=nullptr);
  ~ForceExchange();

  void setZero();
  uint qDim() { return 6; }
  void calc_F_from_q(const arr& q, uint n);
  arr calc_q_from_F() const;

  PairCollision* coll();

  void glDraw(OpenGL&);
  void write(ostream& os) const;
};
stdOutPipe(ForceExchange)

struct TM_ContactNegDistance : Feature {
  const ForceExchange& C;

  TM_ContactNegDistance(const ForceExchange& contact) : C(contact) {}

  void phi(arr& y, arr& J, const rai::Configuration& K);
  virtual uint dim_phi(const rai::Configuration& K) { return 1; }
  virtual rai::String shortTag(const rai::Configuration& K) { return STRING("ContactNegDistance-"<<C.a.name<<'-'<<C.b.name); }
};

} //rai
