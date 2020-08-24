/*  ------------------------------------------------------------------
    Copyright (c) 2011-2020 Marc Toussaint
    email: toussaint@tu-berlin.de

    This code is distributed under the MIT License.
    Please see <root-path>/LICENSE for details.
    --------------------------------------------------------------  */

#pragma once

#include "../Core/thread.h"
#include "../Kin/kin.h"

enum RobotType { ROB_sim=0, ROB_pr2, ROB_baxter, ROB_kukaWSG };
enum SensorId { SEN_depth };

struct RobotAbstraction {
  Var<rai::Configuration> K;
  Var<arr> frameState;
  Var<arr> jointState;
  Var<double> timeToGo;

  virtual ~RobotAbstraction() {}
  //-- basic info
  virtual StringA getJointNames() = 0;
  virtual arr getHomePose() = 0;
  //-- execution
  virtual bool executeMotion(const StringA& joints, const arr& path, const arr& times, double timeScale=1., bool append=false) = 0;
  virtual void execGripper(const rai::String& gripperName, double position, double force=40.) = 0;
  virtual void attach(const char* a, const char* b) {}
  //-- feedback
  virtual arr getJointPositions(const StringA& joints= {}) = 0;
  //-- sensors
  virtual arr getSensor(SensorId sensor) { return NoArr; }
  virtual void getSensor(SensorId sensor, arr& data) { NIY; }

  void waitForCompletion();
};

struct RobotIO : RobotAbstraction {
  std::shared_ptr<RobotAbstraction> self;
  rai::Enum<RobotType> type;

  RobotIO(const rai::Configuration& _K, RobotType type);
  ~RobotIO();

  //-- just call virtuals
  bool executeMotion(const StringA& joints, const arr& path, const arr& times, double timeScale=1., bool append=false) {
    if(!path.N && !times.N) {
      LOG(-1) <<"you send an empty path to execute - perhaps the path computation didn't work";
      return false;
    }
    return self->executeMotion(joints, path, times, timeScale, append);
  }
  void execGripper(const rai::String& gripper, double position, double force=40.) { return self->execGripper(gripper, position, force); }
  arr getJointPositions(const StringA& joints= {}) { return self->getJointPositions(joints); }
  StringA getJointNames() { return self->getJointNames(); }
  arr getHomePose() { return self->getHomePose(); }
  void attach(const char* a, const char* b) { return self->attach(a, b); }
  virtual double timeToGo() { return self->timeToGo(); }
  arr getSensor(SensorId sensor) { self->getSensor(sensor); return arr(); }
  virtual void getSensor(SensorId sensor, arr& data) { self->getSensor(sensor, data); }

};

