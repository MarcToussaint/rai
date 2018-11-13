#pragma once

#include "simulation.h"
#include "robotio.h"

struct SimulationThread : Thread, RobotAbstraction {

  Simulation SIM;
  double dt;
  bool pubSubToROS;
  struct SimulationThread_self* self=0;
  arr q0;

  SimulationThread(const rai::KinematicWorld& _K, double dt=.01, bool pubSubToROS=false);
  ~SimulationThread();

  //non threaded looping
  void loop();

  //--- Thread

  void step();
  void close(){}
  void open(){}

  //--- RobotIO

  //-- basic info
  virtual StringA getJointNames();
  virtual arr getHomePose();
  //-- execution
  virtual bool executeMotion(const StringA& joints, const arr& path, const arr& times, double timeScale=1., bool append=false);
  virtual void execGripper(const rai::String& gripperName, double position, double force=40.);
  virtual void attach(const char *a, const char *b);
  //-- feedback
  virtual arr getJointPositions(const StringA& joints={});

};
