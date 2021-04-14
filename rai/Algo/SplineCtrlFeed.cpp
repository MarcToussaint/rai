#include "SplineCtrlFeed.h"

namespace rai{

void SplineCtrlReference::initialize(const arr& q_real, const arr& qDot_real, double ctrlTime) {
  spline.set()->set(2, ~q_real, {ctrlTime});
}

void SplineCtrlReference::getReference(arr& q_ref, arr& qDot_ref, arr& qDDot_ref, const arr& q_real, const arr& qDot_real, double ctrlTime){
  if(!spline.get()->points.N) initialize(q_real, qDot_real, ctrlTime);
  spline.get() -> eval(q_ref, qDot_ref, qDDot_ref, ctrlTime);
}

void SplineCtrlReference::waitForInitialized(){
  while(!spline.get()->times.N) spline.waitForNextRevision();
}

void SplineCtrlReference::append(const arr& x, const arr& t, double ctrlTime, bool prependLast){
  waitForInitialized();
  arr _x(x), _t(t);
  auto splineSet = spline.set();
  if(prependLast){
    _x.prepend(splineSet->points[-1]);
    _t.prepend(0.);
  }
  if(ctrlTime > splineSet->end()){ //previous spline is done... create new one
    splineSet->set(2, _x, _t+ctrlTime);
  }else{ //previous spline still active... append
    splineSet->append(_x, _t);
  }
}

void SplineCtrlReference::overrideSmooth(const arr& x, const arr& t, double ctrlTime){
  CHECK(t.first()>.1, "that's too harsh!");
  waitForInitialized();
  arr x_now, xDot_now;
  arr _x(x), _t(t);
  auto splineSet = spline.set();
  splineSet->eval(x_now, xDot_now, NoArr, ctrlTime);
  _x.prepend(x_now);
  _t.prepend(0.);
  splineSet->set(2, _x, _t+ctrlTime, xDot_now);
}

void SplineCtrlReference::overrideHard(const arr& x, const arr& t, const arr& xDot0, double nowTime){
  waitForInitialized();
  //only saftey checks: evaluate the current spline and time
  auto splineSet = spline.set();
  {
      arr x_now, xDot_now;
      splineSet->eval(x_now, xDot_now, NoArr, nowTime);
      CHECK_LE(t.first(), .0, "");
      CHECK_GE(t.first(), -.2, "you first time knot is more than 200msec ago!");
      CHECK_LE(maxDiff(x[0],x_now), .1, "your first point knot is too far from the current spline");
      CHECK_LE(maxDiff(xDot0,xDot_now), .5, "your initial velocity is too far from the current spline");
  }

  splineSet->set(2, x, t+nowTime, xDot0);
}



} //namespace
