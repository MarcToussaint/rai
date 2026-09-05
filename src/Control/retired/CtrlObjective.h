/*  ------------------------------------------------------------------
    Copyright (c) 2011-2024 Marc Toussaint
    email: toussaint@tu-berlin.de

    This code is distributed under the MIT License.
    Please see <root-path>/LICENSE for details.
    --------------------------------------------------------------  */

#pragma once

#include "../Core/thread.h"
#include "../Kin/kin.h"
#include "../Optim/NLP.h"

struct CtrlObjective;
struct CtrlMovingTarget;
struct CtrlSolver;
typedef rai::Array<CtrlObjective*> CtrlObjectiveL;

//===========================================================================

/** In the given task space, a task can represent: 1) a pos/vel ctrl task
 *  and/or 2) a compliance and/or 3) a force limit control */
struct CtrlObjective {
  std::shared_ptr<Feature> feat;    ///< this defines the task space
  ObjectiveType type;               ///< element of {sumOfSqr, inequality, equality}
  double transientStep; //TODO -> arr times;
  rai::String name;                 ///< just for easier reporting

  //-- the reference (zero point in feature space (target in KOMO)) can be continuously changed by motion primitives or other means
  std::shared_ptr<CtrlMovingTarget> movingTarget;  ///< non-nullptr iff this is a pos/vel task

  //-- parameters that influence how CtrlMethods treat this objective
  bool active;     //TODO REMOVE  ///< also non-active tasks are updated (states evaluated), but don't enter the TaskControlMethods
//  double kp, kd;     ///< gains
//  arr C;             ///< feature space compliance matrix (TODO: needed?)

  //-- buffers that store the last evaluation of the feature, andfeature values -- these are always kept up-to-date (in update)
  ActStatus status;  ///< discrete status based on the reference
  arr y_buffer;
//  arr y, J_y;        ///< update() will evaluate these for a given kinematic configuration
//  arr f;             ///< measured generalized force in this task space

  CtrlObjective() : type(OT_sos), transientStep(-1.), active(true), /*kp(1.), kd(1.),*/ status(AS_init) {}
//  CtrlObjective(char* _name, const shared_ptr<Feature>& _feat, const shared_ptr<CtrlReference>& _ref, double _kp, double _kd, const arr& _C);
  ~CtrlObjective() {}

  arr getResidual(CtrlSolver& cp);
  arr getValue(CtrlSolver& cp);

//  arr update_y(const ConfigurationL& Ctuple); //returns the CHANGE in y (to estimate velocity)
  void resetState();

  void setRef(const std::shared_ptr<CtrlMovingTarget>& _ref);
  void setTarget(const arr& y_target);
  void setTimeScale(double d);

  void reportState(ostream& os) const;
};
