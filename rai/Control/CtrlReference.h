#pragma once

#include "CtrlObjective.h"

//===========================================================================

struct CtrlReference_Const : CtrlReference {
  arr y_target;
  bool flipTargetSignOnNegScalarProduct;
  CtrlReference_Const(const arr& y_target, bool flip=false) : y_target(y_target), flipTargetSignOnNegScalarProduct(flip) {}
  virtual ActStatus step(double tau, const arr& y_real, const arr& v_real);
  virtual void setTarget(const arr& ytarget, const arr& vtarget=NoArr) { y_target = ytarget; }
  virtual void setTimeScale(double d) {}
  virtual void resetState() {}
  virtual bool isDone() { return false; }
};

//===========================================================================

struct CtrlReference_MaxCarrot : CtrlReference {
  arr y_target;
  double maxDistance;
  CtrlReference_MaxCarrot(const arr& y_target, double maxDistance) : y_target(y_target), maxDistance(maxDistance) {}
  virtual ActStatus step(double tau, const arr& y_real, const arr& v_real);
  virtual void setTarget(const arr& ytarget, const arr& vtarget=NoArr){ y_target = ytarget; }
  virtual void setTimeScale(double d) {}
  virtual void resetState() {}
  virtual bool isDone() { return false; }
};

//===========================================================================

struct CtrlReference_ConstVel : CtrlReference {
  arr v_target;
  CtrlReference_ConstVel(const arr& _v_target) : v_target(_v_target) {}
  virtual ActStatus step(double tau, const arr& y_real, const arr& v_real);
  virtual void setTarget(const arr& ytarget, const arr& vtarget=NoArr) { CHECK(!!vtarget, ""); v_target = vtarget; }
  virtual void setTimeScale(double d) {}
  virtual void resetState() {}
  virtual bool isDone() { return false; }
};

//===========================================================================

struct CtrlReference_Sine : CtrlReference {
  arr y_start, y_target, y_err;
  double t, T;
  CtrlReference_Sine(const arr& y_target, double duration) : y_target(y_target), t(0.), T(duration) {}
  virtual ActStatus step(double tau, const arr& y_real, const arr& v_real);
  virtual void setTarget(const arr& ytarget, const arr& vtarget=NoArr);
  virtual void setTimeScale(double d) { T=d; }
  virtual void resetState() { y_start.clear(); y_err.clear(); t=0.; }
  virtual bool isDone();
};

//===========================================================================

struct CtrlReference_Bang : CtrlReference {
  arr y_target;           ///< position target of this motion generator
  double maxVel;          ///< parameters
  double tolerance;
  CtrlReference_Bang();
  CtrlReference_Bang(const arr& _y_target, double _maxVel);

  virtual void setTarget(const arr& ytarget, const arr& vtarget=NoArr);
  virtual void setTimeScale(double d) { HALT("doesn't make sense"); }
  virtual ActStatus step(double tau, const arr& y_real, const arr& v_real);
  virtual void resetState() {}
  virtual bool isDone() { return false; }
};

//===========================================================================

struct CtrlReference_PD : CtrlReference {
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

  virtual void setTarget(const arr& ytarget, const arr& vtarget=NoArr);
  virtual void setTimeScale(double d) { setGainsAsNatural(d, .9); }
  virtual ActStatus step(double tau, const arr& y_real, const arr& v_real);
  virtual void resetState() { y_ref.clear(); v_ref.clear(); }

  void setGains(double _kp, double _kd);
  void setGainsAsNatural(double decayTime, double dampingRatio); ///< the decayTime is the to decay to 10% of the initial offset/error


  arr getDesiredAcceleration();
  void getDesiredLinAccLaw(arr& Kp_y, arr& Kd_y, arr& a0_y);

  double error();
  bool isConverged(double tolerance);
  virtual bool isDone() { return isConverged(tolerance); }
};

//===========================================================================

struct CtrlReference_Path: CtrlReference {
  rai::Spline spline;
  double endTime;
  double time;
  CtrlReference_Path(const arr& path, double endTime);
  CtrlReference_Path(const arr& path, const arr& times);
  virtual ActStatus step(double tau, const arr& y_real, const arr& v_real);
  virtual void setTarget(const arr& ytarget, const arr& vtarget=NoArr) { HALT("can't directly set target of a path"); }
  virtual void setTimeScale(double d) { endTime = d; }
  virtual void resetState() { NIY }
  virtual bool isDone() { return time>=endTime; }
};
