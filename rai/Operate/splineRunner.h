/*  ------------------------------------------------------------------
    Copyright (c) 2011-2020 Marc Toussaint
    email: toussaint@tu-berlin.de

    This code is distributed under the MIT License.
    Please see <root-path>/LICENSE for details.
    --------------------------------------------------------------  */

#include "../Algo/spline.h"

struct SplineRunner {
  rai::Spline refSpline; // reference spline constructed from ref
  arr refPoints, refTimes; // the knot points and times of the spline
  double phase=0.; // current phase in the spline

  void set(const arr& x, const arr& t, const arr& x0, bool append);
  arr run(double dt, arr& qref_dot=NoArr);
  double timeToGo();
  void stop();
};
