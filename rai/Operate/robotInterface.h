#pragma once

#include "simulation.h"
#include "robotio.h"

struct RobotInterface {
  std::shared_ptr<struct sRobotInterface> s;

  RobotInterface(const rai::KinematicWorld& _K, double dt=.01);
  ~RobotInterface();

  //-- basic info
  const StringA& getJointNames();
  arr getHomePose();

  //-- execution
  void move(const arr& path, const arr& times, bool append=true);   ///< core method to send motion (see detailed doc in cpp)
  void move(const arrA& poses, const arr& times, bool append=true); ///< wrapper of the above: send a sequence of poses using {q1, q2, q3}
  double timeToGo();                                                ///< get the remaining time til the end of the spline
  void wait(){ while(timeToGo()) rai::wait(.1); }

  //-- feedback
  arr getJointPositions(const StringA& joints={});
  void sync(rai::KinematicWorld& K);  ///< copies current robot pose into K
};
