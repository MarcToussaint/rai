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
  ACCESS(arr, ctrl_q_real)
  ACCESS(arr, ctrl_q_ref)

  ACCESS(CtrlMsg, ctrl_ref) //< the message send to the RTController
  ACCESS(CtrlMsg, ctrl_obs) //< the message received from the RTController
  ACCESS(mlr::Array<CtrlTask*>, ctrlTasks)
  ACCESS(mlr::KinematicWorld, modelWorld)
  ACCESS(bool, fixBase)
  ACCESS(arr, pr2_odom)
  ACCESS(double, IK_cost)

//private:
  mlr::KinematicWorld realWorld;
  TaskControlMethods *taskController;
  arr q_real, qdot_real; //< real state
  arr q_model, qdot_model; //< model state
  arr q0; //< homing pose
  mlr::String robot;
  bool useRos;
  bool useSwift;
  bool requiresInitialSync;
  bool syncModelStateWithReal; //< whether the step() should reinit the state from the ros message
  bool verbose;
  bool useDynSim;
  bool compensateGravity;
  bool compensateFTSensors;
  RTControllerSimulation* dynSim;

  GravityCompensation* gc;

  arr fRInitialOffset;


public:
  TaskControlThread(const char* robot="none", const mlr::KinematicWorld& world = NoWorld);
  ~TaskControlThread();

  void open();
  void step();
  void close();
};
