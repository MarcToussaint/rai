#pragma once

#include "CtrlObjective.h"

//===========================================================================

struct CtrlTarget_Const : CtrlTarget {
  CtrlTarget_Const() {}
  virtual ActStatus step(arr& target, double tau, const arr& y_real);
  virtual void setTimeScale(double d) {}
  virtual void resetState() {}
};

//===========================================================================

struct CtrlTarget_MaxCarrot : CtrlTarget {
  double maxDistance;
  arr goal;
  uint countInRange=0;
  CtrlTarget_MaxCarrot(CtrlObjective& co, double maxDistance, const arr& _goal=NoArr) : maxDistance(maxDistance){ if(!!_goal) goal=_goal; }
  virtual ActStatus step(arr& target, double tau, const arr& y_real);
  virtual void setTimeScale(double d) {}
  virtual void resetState() {}
};

//===========================================================================

struct CtrlTarget_ConstVel : CtrlTarget {
  CtrlTarget_ConstVel() {}
  virtual ActStatus step(arr& target, double tau, const arr& y_real);
  virtual void setTimeScale(double d) {}
  virtual void resetState() {}
};

//===========================================================================

struct CtrlTarget_Sine : CtrlTarget {
  arr y_start, y_target, y_err;
  double t, T;
  CtrlTarget_Sine(const arr& y_target, double duration) : y_target(y_target), t(0.), T(duration) {}
  virtual ActStatus step(arr& target, double tau, const arr& y_real);
  virtual void setTimeScale(double d) { T=d; }
  virtual void resetState() { y_start.clear(); y_err.clear(); t=0.; }
};

//===========================================================================

struct CtrlTarget_Bang : CtrlTarget {
  arr y_target;           ///< position target of this motion generator
  double maxVel;          ///< parameters
  double tolerance;
  CtrlTarget_Bang();
  CtrlTarget_Bang(const arr& _y_target, double _maxVel);

  virtual void setTimeScale(double d) { HALT("doesn't make sense"); }
  virtual ActStatus step(arr& target, double tau, const arr& y_real);
  virtual void resetState() {}
};

//===========================================================================

struct CtrlTarget_PD : CtrlTarget {
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
  virtual ActStatus step(arr& target, double tau, const arr& y_real);
  virtual void resetState() { y_ref.clear(); v_ref.clear(); }

  void setTarget(const arr&, const arr& _v=NoArr){ NIY; }
  void setGains(double _kp, double _kd);
  void setGainsAsNatural(double decayTime, double dampingRatio); ///< the decayTime is the to decay to 10% of the initial offset/error


  arr getDesiredAcceleration();
  void getDesiredLinAccLaw(arr& Kp_y, arr& Kd_y, arr& a0_y);

  double error();
  bool isConverged(double tolerance);
};

//===========================================================================

struct CtrlTarget_Path: CtrlTarget {
  rai::Spline spline;
  double endTime;
  double time;
  CtrlTarget_Path(const arr& path, double endTime);
  CtrlTarget_Path(const arr& path, const arr& times);
  virtual ActStatus step(arr& target, double tau, const arr& y_real);
  virtual void setTimeScale(double d) { endTime = d; }
  virtual void resetState() { NIY }
};
