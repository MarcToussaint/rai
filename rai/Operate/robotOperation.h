/*  ------------------------------------------------------------------
    Copyright (c) 2019 Marc Toussaint
    email: marc.toussaint@informatik.uni-stuttgart.de

    This code is distributed under the MIT License.
    Please see <root-path>/LICENSE for details.
    --------------------------------------------------------------  */

#pragma once

#include <Kin/kin.h>

struct RobotOperation {
  std::shared_ptr<struct sRobotOperation> s;

  RobotOperation(const rai::Configuration& _K, double dt=.01, const char* rosNodeName="rai_node");
  ~RobotOperation();

  //-- real switch
  void sendToReal(bool activate);

  //-- basic info
  const StringA& getJointNames();
  arr getHomePose();

  //-- execution
  void move(const arr& path, const arr& times, bool append=true);   ///< core method to send motion (see detailed doc in cpp)
  void move(const arrA& poses, const arr& times, bool append=true); ///< wrapper of the above: send a sequence of poses using {q1, q2, q3}
  void moveHard(const arr& pose);
  double timeToGo();                                                ///< get the remaining time til the end of the spline
  void wait() { while(timeToGo()) rai::wait(.1); }

  //-- feedback
  arr getJointPositions(const StringA& joints= {});
  bool getGripperGrabbed(const std::string& whichArm="right");
  bool getGripperOpened(const std::string& whichArm="right");
  void sync(rai::Configuration& K);                                ///< copies current robot pose into K
};
