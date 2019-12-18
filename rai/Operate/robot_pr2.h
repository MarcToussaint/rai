/*  ------------------------------------------------------------------
    Copyright (c) 2019 Marc Toussaint
    email: marc.toussaint@informatik.uni-stuttgart.de

    This code is distributed under the MIT License.
    Please see <root-path>/LICENSE for details.
    --------------------------------------------------------------  */

#include "robotio.h"

struct Robot_PR2 : RobotAbstraction {
  struct Robot_PR2_PathThread* self=0;

  Robot_PR2(const rai::Configuration& _K);
  ~Robot_PR2();

  virtual bool executeMotion(const StringA& joints, const arr& path, const arr& times, double timeScale=1., bool append=false);
  virtual void execGripper(const rai::String& gripper, double position, double force=40.);
  virtual arr getJointPositions(const StringA& joints);
  virtual arr getHomePose();
  virtual  StringA getJointNames();
  virtual double timeToGo();
};
