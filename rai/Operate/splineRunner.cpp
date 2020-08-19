/*  ------------------------------------------------------------------
    Copyright (c) 2011-2020 Marc Toussaint
    email: toussaint@tu-berlin.de

    This code is distributed under the MIT License.
    Please see <root-path>/LICENSE for details.
    --------------------------------------------------------------  */

#include "splineRunner.h"

void SplineRunner::set(const arr& x, const arr& t, const arr& x0, bool append) {
  if(!refTimes.N) append=false;

  if(append) {
    refPoints.append(x);
    double last = refTimes.last();
    refTimes.append(t+last);
    refSpline.set(2, refPoints, refTimes);
  } else {
    refPoints = x;
    refTimes = t;
    if(refTimes(0)>0.) { //the given reference does not have a knot for NOW -> copy the current joint state
      refTimes.prepend(0.);
      refPoints.prepend(x0);
    }
    refSpline.set(2, refPoints, refTimes);
    phase=0.;
  }
}

arr SplineRunner::run(double dt, arr& qref_dot) {
  if(refSpline.points.N) {
    //read out the new reference
    phase += dt;
    double maxPhase = refSpline.times.last();
    arr q_ref = refSpline.eval(phase);
    if(!!qref_dot) qref_dot = refSpline.eval(phase, 1);
    if(phase>maxPhase) { //clear spline buffer
      q_ref = refPoints[-1];
      stop();
    }
    return q_ref;
  }
  return {};
}

double SplineRunner::timeToGo() {
  double maxPhase = 0.;
  if(refSpline.points.N) maxPhase = refSpline.times.last();
  return maxPhase - phase;
}

void SplineRunner::stop() {
  phase=0.;
  refPoints.clear();
  refTimes.clear();
  refSpline.clear();
}
