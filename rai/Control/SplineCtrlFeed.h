#pragma once

#include "CtrlMsgs.h"

#include "../Core/thread.h"
#include "../Algo/spline.h"

namespace rai {

struct SplineCtrlReference : ReferenceFeed {
  Var<Spline> spline;

  void initialize(const arr& q_real, const arr& qDot_real);
  void waitForInitialized();

  void getReference(arr& q_ref, arr& qDot_ref, arr& qDDot_ref, const arr& q_real, const arr& qDot_real, double time);

  void append(const arr& x, const arr& t, bool prependLast=true);
  void override(const arr& x, const arr& t);

  void moveTo(const arr& x, double t, bool append=true);

};

} //namespace
