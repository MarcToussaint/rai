#pragma once

#include "simulation.h"

struct SimulationThread : Thread{
  Simulation SIM;
  double dt;
  bool pubSubToROS;
  struct SimulationIO_self* self=0;
  Var<double> timeToGo;

  SimulationThread(const rai::KinematicWorld& K, double dt=.01, bool pubSubToROS=false);
  ~SimulationThread();

  void step();
  void close(){}
  void open(){}

  void loop();
};
