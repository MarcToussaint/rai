#pragma once

#include <Core/thread.h>
#include <Control/ctrlMsg.h>
#include <Control/taskController.h>
#include <Control/RTControllerSimulation.h>
#include <Control/gravityCompensation.h>


/// The task controller generates the message send to the RT_Controller
/// the problem is defined by the list of CtrlTasks
struct TaskControllerModule : Thread {
  struct sTaskControllerModule *s;

  //protected access points
  ACCESS(arr, ctrl_q_real)
  ACCESS(arr, ctrl_q_ref)

  ACCESS(CtrlMsg, ctrl_ref) //< the message send to the RTController
  ACCESS(CtrlMsg, ctrl_obs) //< the message received from the RTController
  ACCESS(mlr::Array<CtrlTask*>, ctrlTasks)
  ACCESS(mlr::String, effects)
  ACCESS(mlr::KinematicWorld, modelWorld)
  ACCESS(bool, fixBase)
  ACCESS(arr, pr2_odom)

  ACCESS(arr, qSign)

//private:
  mlr::KinematicWorld realWorld;
  TaskController *taskController;
  arr q_real, qdot_real; //< real state
  arr q_model, qdot_model; //< model state
  arr q0; //< homing pose
  mlr::String robot;
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
  TaskControllerModule(const char* robot="none", const mlr::KinematicWorld& world = NoWorld);
  ~TaskControllerModule();

  void open();
  void step();
  void close();
};
