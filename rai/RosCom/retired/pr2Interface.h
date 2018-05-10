/*  ------------------------------------------------------------------
    Copyright (c) 2017 Marc Toussaint
    email: marc.toussaint@informatik.uni-stuttgart.de

    This code is distributed under the MIT License.
    Please see <root-path>/LICENSE for details.
    --------------------------------------------------------------  */

#ifndef PR2INTERFACE_H
#define PR2INTERFACE_H

#include <Core/thread.h>
#include <Core/array.h>
#include <RosCom/roscom.h>
#include <Kin/kin.h>
#include <RosCom/pr2DynamicSimulation.h>
#include <Control/taskSpaceController.h>
#include <RosCom/subscribeAlvarMarkers.h>

struct PR2Interface : Thread {
  VAR(CtrlMsg, ctrl_ref)
  VAR(CtrlMsg, ctrl_obs)
  VAR(AlvarMarkers, ar_pose_markers)
  
  rai::KinematicWorld* realWorld;
  rai::KinematicWorld* modelWorld;
  
  DynamicSimulation* dynamicSimulation;
  
  TaskSpaceController* controller;
  
  CtrlMsg ctrlMsg;
  
  bool logState = true;
  arr logT, logQRef, logQObs, logQDotRef, logQDotObs, logUObs, logU0, logKp, logKd, logFLObs, logFRObs, logKiFt, logJ_ft_inv, logFRef;
  std::map<rai::String, arr> logMap;
  
  arr lGripperRef, rGripperRef, torsoLiftRef;
  
  bool useROS = false;
  
  PR2Interface();
  ~PR2Interface() {threadCloseModules();}
  virtual void step();
  
  void initialize(rai::KinematicWorld* realWorld, rai::KinematicWorld* realWorldSimulation, rai::KinematicWorld* modelWorld, TaskSpaceController* controller = NULL);
  void initialize(rai::KinematicWorld* realWorld, rai::KinematicWorld* modelWorld, TaskSpaceController* controller = NULL);
  void startInterface();
  void sendCommand(const arr& u0, const arr& Kp, const arr& Kd, const arr& K_ft, const arr& J_ft_inv, const arr& fRef, const double& gamma);
  void goToPosition(arr pos, rai::String shape, double executionTime = 10.0, bool useMotionPlaner = true, rai::String name = "goToPosition");
  void goToTasks(rai::Array<LinTaskSpaceAccLaw*> laws, double executionTime = 10.0, bool useMotionPlanner = true);
  void goToTask(TaskMap* map, arr ref, double executionTime = 10.0, bool useMotionPlaner = true, rai::String name = "goToTask");
  void goToJointState(arr jointState, double executionTime = 10.0, bool useMotionPlaner = true, rai::String name = "goToJointState");
  void executeTrajectory(double executionTime);
  void moveTorsoLift(arr torsoLiftRef);
  void moveLGripper(arr lGripperRef);
  void moveRGripper(arr rGripperRef);
  void logStateSave(rai::String name = "noname", rai::String folder = "data");
  void clearLog();
};

void showTrajectory(const arr& traj, rai::KinematicWorld& _world, bool copyWorld = true, double delay = 0.03, rai::String text = "");

#endif // PR2INTERFACE_H
