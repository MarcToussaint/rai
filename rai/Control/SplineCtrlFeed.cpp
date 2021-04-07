#include "SplineCtrlFeed.h"

namespace rai{

void SplineCtrlReference::initialize(const arr& q_real, const arr& qDot_real) {
  spline.set()->set(2, ~q_real, {rai::realTime()});
}

void SplineCtrlReference::getReference(arr& q_ref, arr& qDot_ref, arr& qDDot_ref, const arr& q_real, const arr& qDot_real, double time){
  if(!spline.get()->points.N) initialize(q_real, qDot_real);
  spline.get() -> eval(q_ref, qDot_ref, qDDot_ref, time);
}

void SplineCtrlReference::waitForInitialized(){
  while(!spline.get()->times.N) spline.waitForNextRevision();
}

void SplineCtrlReference::append(const arr& x, const arr& t, bool prependLast){
  waitForInitialized();
  double now = rai::realTime();
  arr _x(x), _t(t);
  auto splineSet = spline.set();
  if(prependLast){
    _x.prepend(splineSet->points[-1]);
    _t.prepend(0.);
  }
  if(now > splineSet->end()){ //previous spline is done... create new one
    splineSet->set(2, _x, _t+now);
  }else{ //previous spline still active... append
    splineSet->append(_x, _t);
  }
}

void SplineCtrlReference::override(const arr& x, const arr& t){
  CHECK(t.first()>.1, "that's too harsh!");
  waitForInitialized();
  double now = rai::realTime();
  arr x_now, xDot_now;
  arr _x(x), _t(t);
  auto splineSet = spline.set();
  splineSet->eval(x_now, xDot_now, NoArr, now);
  _x.prepend(x_now);
  _t.prepend(0.);
  splineSet->set(2, _x, _t+now, xDot_now);
}

void SplineCtrlReference::overrideHardRealTime(const arr& x, const arr& t, const arr& xDot0){
  waitForInitialized();
  //only saftey checks: evaluate the current spline and time
  auto splineSet = spline.set();
  {
      double now = rai::realTime();
      arr x_now, xDot_now;
      splineSet->eval(x_now, xDot_now, NoArr, now);
      CHECK_GE(now, t.first(), "");
      CHECK_LE(now-t.first(), .2, "you first time knot is more than 200msec ago!");
      CHECK_LE(maxDiff(x[0],x_now), .1, "your first point knot is too far from the current spline");
      CHECK_LE(maxDiff(xDot0,xDot_now), .1, "your initial velocity is too far from the current spline");
  }

  splineSet->set(2, x, t, xDot0);
}

void SplineCtrlReference::moveTo(const arr& x, double t, bool append){
  if(append) this->append(~x, {t}, true);
  else  override(~x, {t});
}

} //namespace
