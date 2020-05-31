/*  ------------------------------------------------------------------
    Copyright (c) 2019 Marc Toussaint
    email: marc.toussaint@informatik.uni-stuttgart.de

    This code is distributed under the MIT License.
    Please see <root-path>/LICENSE for details.
    --------------------------------------------------------------  */

#pragma once

#include "../Algo/spline.h"
#include "../Optim/optimization.h"
#include "../Core/thread.h"
#include "../Kin/kin.h"

struct CtrlObjective;
struct CtrlReference;
typedef rai::Array<std::shared_ptr<CtrlObjective>> CtrlObjectiveL;

//===========================================================================

/// a CtrlReference continuously updates the refence (zero-point) of a CtrlObjective (e.g. MotionProfile, reference path)
struct CtrlReference {
  arr y_ref, v_ref;

  virtual ~CtrlReference() {}
  virtual ActStatus step(double tau, const arr& y_real, const arr& v_real) = 0; //step forward, updating the reference based on y_real
  virtual void setTimeScale(double d) = 0;
  virtual void resetState() = 0;
};

//===========================================================================

/** In the given task space, a task can represent: 1) a pos/vel ctrl task
 *  and/or 2) a compliance and/or 3) a force limit control */
struct CtrlObjective {
  std::shared_ptr<Feature> feat;       ///< this defines the task space
  const rai::Enum<ObjectiveType> type;  ///< element of {sumOfSqr, inequality, equality}
  rai::String name;  ///< just for easier reporting

  //-- the reference (zero point in feature space (target in KOMO)) can be continuously changed by motion primitives or other means
  std::shared_ptr<CtrlReference> ref;  ///< non-nullptr iff this is a pos/vel task

  //-- parameters that influence how CtrlMethods treat this objective
  bool active;       ///< also non-active tasks are updated (states evaluated), but don't enter the TaskControlMethods
  double scale;
  double kp, kd;     ///< gains
  arr C;             ///< feature space compliance matrix (TODO: needed?)

  //-- buffers that store the last evaluation of the feature, andfeature values -- these are always kept up-to-date (in update)
  ActStatus status;
  arr y, J_y;           ///< update() will evaluate these for a given kinematic configuration
  arr f;                ///< measured generalized force in this task space


  CtrlObjective() : type(OT_sos), active(true), scale(1.), kp(1.), kd(1.), status(AS_init) {}
//  CtrlObjective(char* _name, const ptr<Feature>& _feat, const ptr<CtrlReference>& _ref, double _kp, double _kd, const arr& _C);
  ~CtrlObjective() {}

  arr update_y(const rai::Configuration& C); //returns the CHANGE in y (to estimate velocity)
  void resetState();

  void setRef(const ptr<CtrlReference>& _ref);
  void setTarget(const arr& y_target);
  void setTimeScale(double d);

  void reportState(ostream& os);
};

//===========================================================================

struct CtrlProblem : NonCopyable {
  rai::Configuration& C;   ///< original world; which is the blueprint for all time-slice worlds (almost const: only makeConvexHulls modifies it)
  double tau;
  double maxVel=1.;
  double maxAcc=1.;

  rai::Array<ptr<CtrlObjective>> objectives;    ///< list of objectives

  CtrlProblem(rai::Configuration& _C, double _tau) : C(_C), tau(_tau) {}
  CtrlObjective* addPDTask(CtrlObjectiveL& tasks, const char* name, double decayTime, double dampingRatio, ptr<Feature> map);
  ptr<CtrlObjective> addObjective(const ptr<Feature>& f, ObjectiveType type, const ptr<CtrlReference>& _ref);
  ptr<CtrlObjective> addObjective(const FeatureSymbol& feat, const StringA& frames,
                                  ObjectiveType type, const arr& scale=NoArr, const arr& target=NoArr, int order=-1);

  void update(rai::Configuration& C);
  void report(ostream& os=std::cout);
  arr solve();

};

//===========================================================================

void naturalGains(double& Kp, double& Kd, double decayTime, double dampingRatio);
void getForceControlCoeffs(arr& f_des, arr& u_bias, arr& K_I, arr& J_ft_inv, const arr& f_ref, double f_alpha, const CtrlObjective& co, const rai::Configuration& world);
