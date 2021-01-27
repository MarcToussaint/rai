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

//===========================================================================

enum ForceExchangeType { FXT_none=-1, FXT_poa=0, FXT_torque=1 };

///Description of a ForceExchange
struct ForceExchange : NonCopyable, GLDrawer {
  Frame &a, &b;
  uint qIndex=UINT_MAX;
  ForceExchangeType type;
  double scale=1.;
 private:
  PairCollision* __coll=0;
 public:

  arr poa;
  arr force;
  arr torque;

  ForceExchange(Frame& a, Frame& b, ForceExchangeType _type, ForceExchange* copyContact=nullptr);
  ~ForceExchange();

  void setZero();
  uint qDim() { return 6; }
  void calc_F_from_q(const arr& q, uint n);
  arr calc_q_from_F() const;

  virtual double sign(Frame *f) const { if(&a==f) return 1.; return -1.; }
  virtual void kinPOA(arr& y, arr& J) const;
  virtual void kinForce(arr& y, arr& J) const;
  virtual void kinTorque(arr& y, arr& J) const;

  PairCollision* coll();

  void glDraw(OpenGL&);
  void write(ostream& os) const;
};
stdOutPipe(ForceExchange)

//===========================================================================

rai::ForceExchange* getContact(rai::Frame* a, rai::Frame* b, bool raiseErrorIfNonExist=true);

} //rai
