/*  ------------------------------------------------------------------
    Copyright (c) 2017 Marc Toussaint
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
  
  //protected access points
  VAR(arr, ctrl_q_real)
  VAR(arr, ctrl_q_ref)
  
  VAR(CtrlMsg, ctrl_ref) //< the message send to the RTController
  VAR(CtrlMsg, ctrl_obs) //< the message received from the RTController
  VAR(rai::Array<CtrlTask*>, ctrlTasks)
  VAR(rai::KinematicWorld, modelWorld)
  VAR(bool, fixBase)
  VAR(double, IK_cost)
  
//private:
  rai::KinematicWorld realWorld;
  TaskControlMethods *taskController;
  arr q_real, qdot_real; //< real state
  arr q_model, qdot_model; //< model state
  arr q0; //< homing pose
  arr Kp_base, Kd_base; //< Kp, Kd parameters defined in the model file
  double kp_factor, kd_factor, ki_factor;
  bool useSwift;
  bool requiresInitialSync; //< whether the step() should reinit the state from the ros message
  bool verbose;
  
public:
  TaskControlThread(const rai::KinematicWorld& world);
  ~TaskControlThread();
  
  void open();
  void step();
  void close();
};
