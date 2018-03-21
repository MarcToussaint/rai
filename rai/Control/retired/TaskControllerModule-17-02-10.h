#pragma once

#include <Core/thread.h>
#include <Control/ctrlMsg.h>
#include <Control/taskControl.h>
#include <Control/RTControllerSimulation.h>
#include <Control/gravityCompensation.h>


/// The task controller generates the message send to the RT_Controller
/// the problem is defined by the list of CtrlTasks
struct TaskControlThread : Thread {
  struct sTaskControlThread *s;

  //protected access points
  VAR(arr, ctrl_q_real)
  VAR(arr, ctrl_q_ref)

  VAR(CtrlMsg, ctrl_ref) //< the message send to the RTController
  VAR(CtrlMsg, ctrl_obs) //< the message received from the RTController
  VAR(rai::Array<CtrlTask*>, ctrlTasks)
  VAR(rai::String, effects)
  VAR(rai::KinematicWorld, modelWorld)
  VAR(bool, fixBase)
  VAR(arr, pr2_odom)

  VAR(arr, qSign)

//private:
  rai::KinematicWorld realWorld;
  TaskControlMethods *taskController;
  arr q_real, qdot_real; //< real state
  arr q_model, qdot_model; //< model state
  arr q0; //< homing pose
  rai::String robot;
  bool oldfashioned;
  bool useRos;
  bool requiresInitialSync;
  bool syncModelStateWithReal; //< whether the step() should reinit the state from the ros message
  bool verbose;
  bool useDynSim;
  bool compensateGravity;
  bool compensateFTSensors;
  RTControllerSimulation* dynSim;

  GravityCompensation* gc;

  arr q_history, qdot_last, a_last, q_lowPass, qdot_lowPass, qddot_lowPass, aErrorIntegral, u_lowPass;
  arr model_error_g;

  arr qLastReading;

  arr fRInitialOffset;


public:
  TaskControlThread(const char* robot="none", const rai::KinematicWorld& world = NoWorld);
  ~TaskControlThread();

  void open();
  void step();
  void close();
};
