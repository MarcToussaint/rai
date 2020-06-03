#pragma once

#include "CtrlObjective.h"

//===========================================================================

struct CtrlReference_Const : CtrlTarget {
  CtrlReference_Const() {}
  virtual ActStatus step(arr& target, double tau, const arr& y_real, const arr& v_real);
  virtual void setTimeScale(double d) {}
  virtual void resetState() {}
};

//===========================================================================

struct CtrlReference_MaxCarrot : CtrlTarget {
  double maxDistance;
  arr goal;
  CtrlReference_MaxCarrot(CtrlObjective& co, double maxDistance, const arr& _goal=NoArr) : maxDistance(maxDistance){ if(!!_goal) goal=_goal; }
  virtual ActStatus step(arr& target, double tau, const arr& y_real, const arr& v_real);
  virtual void setTimeScale(double d) {}
  virtual void resetState() {}
};

//===========================================================================

struct CtrlReference_ConstVel : CtrlTarget {
  CtrlReference_ConstVel() {}
  virtual ActStatus step(arr& target, double tau, const arr& y_real, const arr& v_real);
  virtual void setTimeScale(double d) {}
  virtual void resetState() {}
};

//===========================================================================

struct CtrlReference_Sine : CtrlTarget {
  arr y_start, y_target, y_err;
  double t, T;
  CtrlReference_Sine(const arr& y_target, double duration) : y_target(y_target), t(0.), T(duration) {}
  virtual ActStatus step(arr& target, double tau, const arr& y_real, const arr& v_real);
  virtual void setTimeScale(double d) { T=d; }
  virtual void resetState() { y_start.clear(); y_err.clear(); t=0.; }
};

//===========================================================================

struct CtrlReference_Bang : CtrlTarget {
  arr y_target;           ///< position target of this motion generator
  double maxVel;          ///< parameters
  double tolerance;
  CtrlReference_Bang();
  CtrlReference_Bang(const arr& _y_target, double _maxVel);

  virtual void setTimeScale(double d) { HALT("doesn't make sense"); }
  virtual ActStatus step(arr& target, double tau, const arr& y_real, const arr& v_real);
  virtual void resetState() {}
};

//===========================================================================

struct CtrlReference_PD : CtrlTarget {
  arr y_ref, v_ref;
  arr y_target, v_target;
  double kp, kd;
  double maxVel, maxAcc;
  bool flipTargetSignOnNegScalarProduct;
  bool makeTargetModulo2PI;
  double tolerance;
  CtrlReference_PD();
  CtrlReference_PD(const arr& _y_target, double decayTime, double dampingRatio, double maxVel=-1., double maxAcc=-1.);
  CtrlReference_PD(const rai::Graph& params);

  virtual void setTimeScale(double d) { setGainsAsNatural(d, .9); }
  virtual ActStatus step(arr& target, double tau, const arr& y_real, const arr& v_real);
  virtual void resetState() { y_ref.clear(); v_ref.clear(); }

  void setGains(double _kp, double _kd);
  void setGainsAsNatural(double decayTime, double dampingRatio); ///< the decayTime is the to decay to 10% of the initial offset/error


  arr getDesiredAcceleration();
  void getDesiredLinAccLaw(arr& Kp_y, arr& Kd_y, arr& a0_y);

  double error();
  bool isConverged(double tolerance);
};

//===========================================================================

struct CtrlReference_Path: CtrlTarget {
  rai::Spline spline;
  double endTime;
  double time;
  CtrlReference_Path(const arr& path, double endTime);
  CtrlReference_Path(const arr& path, const arr& times);
  virtual ActStatus step(arr& target, double tau, const arr& y_real, const arr& v_real);
  virtual void setTimeScale(double d) { endTime = d; }
  virtual void resetState() { NIY }
};
