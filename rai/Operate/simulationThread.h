/*  ------------------------------------------------------------------
    Copyright (c) 2019 Marc Toussaint
    email: marc.toussaint@informatik.uni-stuttgart.de

    This code is distributed under the MIT License.
    Please see <root-path>/LICENSE for details.
    --------------------------------------------------------------  */

#pragma once

#include "simulation.h"
#include "robotio.h"

struct SimulationThread : Thread, RobotAbstraction {

  Simulation SIM;
  double dt;
  bool pubSubToROS;
  struct SimulationThread_self* self=0;
  arr q0;

  SimulationThread(const rai::Configuration& _K, double dt=.01, bool pubSubToROS=false);
  ~SimulationThread();

  //non threaded looping
  void loop();

  //--- Thread

  void step();
  void close() {}
  void open() {}

  //--- RobotIO
  //-- basic info
  virtual StringA getJointNames();
  virtual arr getHomePose();
  //-- execution
  virtual bool executeMotion(const StringA& joints, const arr& path, const arr& times, double timeScale=1., bool append=false);
  virtual void execGripper(const rai::String& gripperName, double position, double force=40.);
  virtual void attach(const char* a, const char* b);
  //-- feedback
  virtual arr getJointPositions(const StringA& joints= {});

  //--- artificial intervention
  void addFile(const char* filename, const char* parentOfRoot=nullptr, const rai::Transformation& relOfRoot=0);
};
