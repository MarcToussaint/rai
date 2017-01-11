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
  ACCESS(CtrlMsg, ctrl_ref)
  ACCESS(CtrlMsg, ctrl_obs)
  ACCESS(AlvarMarkers, ar_pose_markers)

  mlr::KinematicWorld* realWorld;
  mlr::KinematicWorld* modelWorld;

  DynamicSimulation* dynamicSimulation;

  TaskSpaceController* controller;

  CtrlMsg ctrlMsg;

  bool logState = true;
  arr logT, logQRef, logQObs, logQDotRef, logQDotObs, logUObs, logU0, logKp, logKd, logFLObs, logFRObs, logKiFt, logJ_ft_inv, logFRef;
  std::map<mlr::String, arr> logMap;

  arr lGripperRef, rGripperRef, torsoLiftRef;

  bool useROS = false;

  PR2Interface();
  ~PR2Interface() {threadCloseModules();}
  virtual void step();

  void initialize(mlr::KinematicWorld* realWorld, mlr::KinematicWorld* realWorldSimulation, mlr::KinematicWorld* modelWorld, TaskSpaceController* controller = NULL);
  void initialize(mlr::KinematicWorld* realWorld, mlr::KinematicWorld* modelWorld, TaskSpaceController* controller = NULL);
  void startInterface();
  void sendCommand(const arr& u0, const arr& Kp, const arr& Kd, const arr& K_ft, const arr& J_ft_inv, const arr& fRef, const double& gamma);
  void goToPosition(arr pos, mlr::String shape, double executionTime = 10.0, bool useMotionPlaner = true, mlr::String name = "goToPosition");
  void goToTasks(mlr::Array<LinTaskSpaceAccLaw*> laws, double executionTime = 10.0, bool useMotionPlanner = true);
  void goToTask(TaskMap* map, arr ref, double executionTime = 10.0, bool useMotionPlaner = true, mlr::String name = "goToTask");
  void goToJointState(arr jointState, double executionTime = 10.0, bool useMotionPlaner = true, mlr::String name = "goToJointState");
  void executeTrajectory(double executionTime);
  void moveTorsoLift(arr torsoLiftRef);
  void moveLGripper(arr lGripperRef);
  void moveRGripper(arr rGripperRef);
  void logStateSave(mlr::String name = "noname", mlr::String folder = "data");
  void clearLog();
};

void showTrajectory(const arr& traj, mlr::KinematicWorld& _world, bool copyWorld = true, double delay = 0.03, mlr::String text = "");

#endif // PR2INTERFACE_H
