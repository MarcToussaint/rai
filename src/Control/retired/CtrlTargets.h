/*  ------------------------------------------------------------------
    Copyright (c) 2011-2024 Marc Toussaint
    email: toussaint@tu-berlin.de

    This code is distributed under the MIT License.
    Please see <root-path>/LICENSE for details.
    --------------------------------------------------------------  */

#pragma once

#include "CtrlObjective.h"

#include "../Algo/spline.h"

//===========================================================================

/// a CtrlMovingTarget continuously updates the 'target' (zero-point) of the Feature of a CtrlObjective -- this allows to realize a MotionProfile or following a reference path or perception based moving target
struct CtrlMovingTarget {
  bool isTransient=false;
  virtual ~CtrlMovingTarget() {}
  virtual ActStatus step(double tau, CtrlObjective* o, const arr& y_real) = 0; //step forward, updating the target based on y_real
  virtual void resetGoal(const arr& goal) {}
  virtual void setTimeScale(double d) = 0;
  virtual void resetState() = 0;
  virtual void reportState(ostream& os) const {
    os <<" (nil)"; //\ty_ref=" <<feat->target <<" \ty-residual=" <<y_buffer;
  }

  virtual arr getResidual(const arr& y_real) { return arr{}; }
};

//===========================================================================

struct CtrlTarget_Const : CtrlMovingTarget {
  CtrlTarget_Const() {}
  virtual ActStatus step(double tau, CtrlObjective* o, const arr& y_real);
  virtual void setTimeScale(double d) {}
  virtual void resetState() {}
};

//===========================================================================

struct CtrlTarget_MaxCarrot : CtrlMovingTarget {
  double maxDistance, goalDistance=0.;
  arr goal;
  uint countInGoalRange=0;
  CtrlTarget_MaxCarrot(CtrlObjective& co, double maxDistance, const arr& _goal=NoArr);
  virtual ActStatus step(double tau, CtrlObjective* o, const arr& y_real);
  virtual void resetGoal(const arr& _goal) { goal=_goal; }
  virtual void setTimeScale(double d) {}
  virtual void resetState() { countInGoalRange=0; }
  virtual arr getResidual(const arr& y_real) { if(goal.N) return y_real-goal; return y_real; }
  virtual void reportState(ostream& os) const {
    os <<" [MaxCarrot: isTransient:" <<isTransient <<" goalDistance:" <<goalDistance <<" maxDistance:" <<maxDistance <<" #countInGoalRange:" <<countInGoalRange <<"]";
  }
};

//===========================================================================

struct CtrlTarget_PathCarrot: CtrlMovingTarget {
  double stepMax;
  rai::BSpline spline;
  double endTime;
  double time=0.;
  uint countInRange=0;
  uint countBlocked=0;
  CtrlTarget_PathCarrot(const arr& path, double stepMax, double _endTime=1.);
  CtrlTarget_PathCarrot(const arr& path, double stepMax, const arr& times);
  virtual ActStatus step(double tau, CtrlObjective* o, const arr& y_real);
  virtual void setTimeScale(double d) { endTime = d; }
  virtual void resetState() { time=0.; countInRange=0.; }
  virtual void reportState(ostream& os) const {
    os <<" [PathCarrot time: " <<time <<'/' <<endTime <<" #conv:" <<countInRange <<" #lag:" <<countBlocked <<"]";
  }
};

//===========================================================================

struct CtrlTarget_ConstVel : CtrlMovingTarget {
  CtrlTarget_ConstVel() {}
  virtual ActStatus step(double tau, CtrlObjective* o, const arr& y_real);
  virtual void setTimeScale(double d) {}
  virtual void resetState() {}
};

//===========================================================================

struct CtrlTarget_Sine : CtrlMovingTarget {
  arr y_start, y_target, y_err;
  double t, T;
  CtrlTarget_Sine(const arr& y_target, double duration) : y_target(y_target), t(0.), T(duration) {}
  virtual ActStatus step(double tau, CtrlObjective* o, const arr& y_real);
  virtual void setTimeScale(double d) { T=d; }
  virtual void resetState() { y_start.clear(); y_err.clear(); t=0.; }
};

//===========================================================================

struct CtrlTarget_Bang : CtrlMovingTarget {
  arr y_target;           ///< position target of this motion generator
  double maxVel;          ///< parameters
  double tolerance;
  CtrlTarget_Bang();
  CtrlTarget_Bang(const arr& _y_target, double _maxVel);

  virtual void setTimeScale(double d) { HALT("doesn't make sense"); }
  virtual ActStatus step(double tau, CtrlObjective* o, const arr& y_real);
  virtual void resetState() {}
};

//===========================================================================

struct CtrlTarget_PD : CtrlMovingTarget {
  arr y_ref, v_ref;
  arr y_target, v_target;
  double kp, kd;
  double maxVel, maxAcc;
  bool flipTargetSignOnNegScalarProduct;
  bool makeTargetModulo2PI;
  double tolerance;
  CtrlTarget_PD();
  CtrlTarget_PD(const arr& _y_target, double decayTime, double dampingRatio, double maxVel=-1., double maxAcc=-1.);
  CtrlTarget_PD(const rai::Graph& params);

  virtual void setTimeScale(double d) { setGainsAsNatural(d, .9); }
  virtual ActStatus step(double tau, CtrlObjective* o, const arr& y_real);
  virtual void resetState() { y_ref.clear(); v_ref.clear(); }

  void setTarget(const arr&, const arr& _v=NoArr) { NIY; }
  void setGains(double _kp, double _kd);
  void setGainsAsNatural(double decayTime, double dampingRatio); ///< the decayTime is the to decay to 10% of the initial offset/error

  arr getDesiredAcceleration();
  void getDesiredLinAccLaw(arr& Kp_y, arr& Kd_y, arr& a0_y);

  double error();
  bool isConverged(double tolerance);
};

