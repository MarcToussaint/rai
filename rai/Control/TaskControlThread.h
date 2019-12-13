/*  ------------------------------------------------------------------
    Copyright (c) 2019 Marc Toussaint
    email: marc.toussaint@informatik.uni-stuttgart.de

    This code is distributed under the MIT License.
    Please see <root-path>/LICENSE for details.
    --------------------------------------------------------------  */

#pragma once

#include <Core/thread.h>
#include <Control/ctrlMsg.h>
#include <Control/taskControl.h>
#include <Control/RTControllerSimulation.h>
#include <Control/gravityCompensation.h>

/// The task controller generates the message send to the RT_Controller
/// the problem is defined by the list of CtrlTasks
struct TaskControlThread : Thread {

  Var<rai::Configuration> ctrl_config;
  Var<CtrlMsg> ctrl_ref;
  Var<CtrlMsg> ctrl_state;
  Var<CtrlTaskL> ctrl_tasks;

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
                    const Var<CtrlTaskL>& _ctrl_tasks);
  ~TaskControlThread();

  arr whatsTheForce(const ptr<CtrlTask>& t);

  void step();
};

#if 0 //draft
struct TaskControlUserInterface {
  Var<rai::Configuration> ctrl_config;
  Var<CtrlTaskL> ctrl_tasks;

  TaskControlUserInterface(const Var<rai::Configuration>& _ctrl_config, const Var<CtrlTaskL>& _ctrl_tasks);

  //add your wish function
};

#endif

ptr<CtrlTask> addCtrlTask(Var<CtrlTaskL>& ctrlTasks,
                          Var<rai::Configuration>& ctrl_config,
                          const char* name, const ptr<Feature>& map,
                          const ptr<MotionProfile>& ref);

ptr<CtrlTask> addCtrlTask(Var<CtrlTaskL>& ctrlTasks,
                          Var<rai::Configuration>& ctrl_config,
                          const char* name, FeatureSymbol fs, const StringA& frames,
                          const ptr<MotionProfile>& ref);

ptr<CtrlTask> addCtrlTask(Var<CtrlTaskL>& ctrlTasks,
                          Var<rai::Configuration>& ctrl_config,
                          const char* name, FeatureSymbol fs, const StringA& frames,
                          double duration);

ptr<CtrlTask> addCompliance(Var<CtrlTaskL>& ctrlTasks,
                            Var<rai::Configuration>& ctrl_config,
                            const char* name, FeatureSymbol fs, const StringA& frames,
                            const arr& compliance);

void removeCtrlTask(Var<CtrlTaskL>& ctrlTasks, const ptr<CtrlTask>& t);

