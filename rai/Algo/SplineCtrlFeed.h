#pragma once

#include "spline.h"

#include "../Core/thread.h"

namespace rai {

struct ReferenceFeed {
  /// callback called by a robot control loop
  virtual void getReference(arr& q_ref, arr& qDot_ref, arr& qDDot_ref, const arr& q_real, const arr& qDot_real, double ctrlTime) = 0;
};

struct SplineCtrlReference : ReferenceFeed {
  Var<Spline> spline;

  /// initializes to constant (q_real, zero-vel) spline
  void initialize(const arr& q_real, const arr& qDot_real, double time);
  void waitForInitialized();

  /// callback called by a robot control loop; at first time initializes the spline
  void getReference(arr& q_ref, arr& qDot_ref, arr& qDDot_ref, const arr& q_real, const arr& qDot_real, double ctrlTime);

  /// append new knots to the spline; if prependLast, the currently last spline remains a zero-vel double know (holds) before starting the appended
  void append(const arr& x, const arr& t, double ctrlTime, bool prependLast);
  /// override the spline, but use the current spline's current pos/vel as start knot of the new spline; the first time knot needs to be >.1 sec
  void overrideSmooth(const arr& x, const arr& t, double ctrlTime);
  /// fully override the spline with new knots x and t, as well as initial vel xDot0; for safety, the first x needs to be close to the current spline's current pos
  void overrideHard(const arr& x, const arr& t, double ctrlTime);

  //simple helper for single goal spline
  void moveTo(const arr& x, double t, double ctrlTime, bool append){
    if(append) this->append(~x, {t}, ctrlTime, true);
    else  overrideSmooth(~x, {t}, ctrlTime);
  }

  //info:
  double getEndTime() { return spline.get()->end(); }
  void eval(arr& x, arr& xDot, arr& xDDot, double t) { spline.get()->eval(x, xDot, xDDot, t); }

  void report(double ctrlTime);
};

struct CubicSplineCtrlReference : ReferenceFeed {
  Var<CubicSpline> spline;

  /// initializes to constant (q_real, zero-vel) spline
  void initialize(const arr& q_real, const arr& qDot_real, double time);
  void waitForInitialized();

  /// callback called by a robot control loop; at first time initializes the spline
  void getReference(arr& q_ref, arr& qDot_ref, arr& qDDot_ref, const arr& q_real, const arr& qDot_real, double ctrlTime);

  /// append new knots to the spline; if prependLast, the currently last spline remains a zero-vel double know (holds) before starting the appended
  void append(const arr& x, const arr& v, const arr& t, double ctrlTime);
  /// override the spline, but use the current spline's current pos/vel as start knot of the new spline; the first time knot needs to be >.1 sec
  void overrideSmooth(const arr& x, const arr& v, const arr& t, double ctrlTime);
  /// fully override the spline with new knots x and t, as well as initial vel xDot0; for safety, the first x needs to be close to the current spline's current pos
  void overrideHard(const arr& x, const arr& v, const arr& t, double ctrlTime);

  //simple helper for single goal spline
  void moveTo(const arr& x, double t, double ctrlTime, bool append){
    if(append) this->append(~x, zeros(1, x.N), {t}, ctrlTime);
    else  overrideSmooth(~x, zeros(1, x.N), {t}, ctrlTime);
  }

  //info:
  double getEndTime() { waitForInitialized(); return spline.get()->end(); }
  void eval(arr& x, arr& xDot, arr& xDDot, double t) { spline.get()->eval(x, xDot, xDDot, t); }

  void report(double ctrlTime);
};

} //namespace
