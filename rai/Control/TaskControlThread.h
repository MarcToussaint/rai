/*  ------------------------------------------------------------------
    Copyright (c) 2011-2020 Marc Toussaint
    email: toussaint@tu-berlin.de

    This code is distributed under the MIT License.
    Please see <root-path>/LICENSE for details.
    --------------------------------------------------------------  */

#pragma once

#include "ctrlMsg.h"
#include "control.h"
#include "RTControllerSimulation.h"
#include "gravityCompensation.h"
#include "../Core/thread.h"

/// The task controller generates the message send to the RT_Controller
/// the problem is defined by the list of CtrlObjectives
struct TaskControlThread : Thread {

  Var<rai::Configuration> ctrl_config;
  Var<CtrlMsg> ctrl_ref;
  Var<CtrlMsg> ctrl_state;
  Var<CtrlObjectiveL> ctrl_tasks;

  arr q_real, qdot_real, torques_real; //< real state
  arr q_model, qdot_model; //< model state
  arr q0; //< homing pose
  arr Hmetric;

  arr Kp_base, Kd_base; //< Kp, Kd parameters defined in the model file
  double kp_factor, kd_factor, ki_factor;
  bool useSwift;
  bool requiresInitialSync; //< whether the step() should reinit the state from the ros message
  int verbose;

  TaskControlThread(const Var<rai::Configuration>& _ctrl_config,
                    const Var<CtrlMsg>& _ctrl_ref,
                    const Var<CtrlMsg>& _ctrl_state,
                    const Var<CtrlObjectiveL>& _ctrl_tasks);
  ~TaskControlThread();

  arr whatsTheForce(const ptr<CtrlObjective>& t);

  void step();
};

#if 0 //draft
struct TaskControlUserInterface {
  Var<rai::Configuration> ctrl_config;
  Var<CtrlObjectiveL> ctrl_tasks;

  TaskControlUserInterface(const Var<rai::Configuration>& _ctrl_config, const Var<CtrlObjectiveL>& _ctrl_tasks);

  //add your wish function
};

#endif

ptr<CtrlObjective> addCtrlObjective(Var<CtrlObjectiveL>& ctrlTasks,
                                    Var<rai::Configuration>& ctrl_config,
                                    const char* name, const ptr<Feature>& map,
                                    const ptr<CtrlMovingTarget>& ref);

ptr<CtrlObjective> addCtrlObjective(Var<CtrlObjectiveL>& ctrlTasks,
                                    Var<rai::Configuration>& ctrl_config,
                                    const char* name, FeatureSymbol fs, const StringA& frames,
                                    const ptr<CtrlMovingTarget>& ref);

ptr<CtrlObjective> addCtrlObjective(Var<CtrlObjectiveL>& ctrlTasks,
                                    Var<rai::Configuration>& ctrl_config,
                                    const char* name, FeatureSymbol fs, const StringA& frames,
                                    double duration);

ptr<CtrlObjective> addCompliance(Var<CtrlObjectiveL>& ctrlTasks,
                                 Var<rai::Configuration>& ctrl_config,
                                 const char* name, FeatureSymbol fs, const StringA& frames,
                                 const arr& compliance);

void removeCtrlObjective(Var<CtrlObjectiveL>& ctrlTasks, CtrlObjective* t);

