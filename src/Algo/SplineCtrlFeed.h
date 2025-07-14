/*  ------------------------------------------------------------------
    Copyright (c) 2011-2024 Marc Toussaint
    email: toussaint@tu-berlin.de

    This code is distributed under the MIT License.
    Please see <root-path>/LICENSE for details.
    --------------------------------------------------------------  */

#pragma once

#include "spline.h"

#include "../Core/thread.h"

namespace rai {

struct ReferenceFeed {
  /// callback called by a robot control loop
  virtual void getReference(arr& q_ref, arr& qDot_ref, arr& qDDot_ref, const arr& q_real, const arr& qDot_real, double ctrlTime) = 0;
};

struct BSplineCtrlReference : ReferenceFeed {
  Var<BSpline> spline;
  uint degree=3;

  /// initializes to constant (q_real, zero-vel) spline
  void initialize(const arr& q_real, const arr& qDot_real, double time);
  void waitForInitialized();

  /// callback called by a robot control loop; at first time initializes the spline
  void getReference(arr& q_ref, arr& qDot_ref, arr& qDDot_ref, const arr& q_real, const arr& qDot_real, double ctrlTime);

  /// append new knots to the spline; the currently last spline remains a zero-vel double know (holds) before starting the appended
  void append(const arr& x, const arr& t, double ctrlTime);
  /// override the spline, but use the current spline's current pos/vel as start knot of the new spline; the first time knot needs to be >.1 sec
  void overwriteSmooth(const arr& x, const arr& t, double ctrlTime);
  /// fully override the spline with new knots x and t, as well as initial vel xDot0; for safety, the first x needs to be close to the current spline's current pos
  void overwriteHard(const arr& x, const arr& t, double ctrlTime);

  //info:
  double getEndTime() { return spline.get()->end(); }
  arr getEndPoint() { return spline.get()->ctrlPoints[-1].copy(); }
  void eval(arr& x, arr& xDot, arr& xDDot, double t) { spline.get()->eval3(x, xDot, xDDot, t); }

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
  void overwriteSmooth(const arr& x, const arr& v, const arr& t, double ctrlTime);
  /// fully override the spline with new knots x and t, as well as initial vel xDot0; for safety, the first x needs to be close to the current spline's current pos
  void overwriteHard(const arr& x, const arr& v, const arr& t, double ctrlTime);

  //info:
  double getEndTime() { waitForInitialized(); return spline.get()->end(); }
  void eval(arr& x, arr& xDot, arr& xDDot, double t) { spline.get()->eval(x, xDot, xDDot, t); }

  void report(double ctrlTime);
};

} //namespace
