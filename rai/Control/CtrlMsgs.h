/*  ------------------------------------------------------------------
    Copyright (c) 2017 Marc Toussaint
    email: marc.toussaint@informatik.uni-stuttgart.de

    This code is distributed under the MIT License.
    Please see <root-path>/LICENSE for details.
    --------------------------------------------------------------  */

#pragma once

#include "../Core/array.h"
#include "../Algo/SplineCtrlFeed.h"

namespace rai {

enum class ControlType { configRefs, projectedAcc };

//The control message send to the robot
struct CtrlCmdMsg {
  ControlType controlType=ControlType::configRefs;
  std::shared_ptr<ReferenceFeed> ref; // joint space references
//  arr q_ref, qDot_ref, qDDot_ref; // joint space references
  arr u_b; // open-loop/feed-forward torque term
  arr Kp, Kd; // gain matrices
  arr P_compliance;
};

// The state message comming back from the robot
struct CtrlStateMsg {
  double ctrlTime=0.;
  int stall=0; //now many iterations should we stall (not increment ctrl time)
  arr q, qDot; // actual joint state
  arr tauExternal; // external torques
  void initZero(uint n){ q.resize(n).setZero(); qDot.resize(n).setZero(); tauExternal.resize(n).setZero(); }
};

struct RobotAbstraction{
  Var<rai::CtrlCmdMsg> cmd;
  Var<rai::CtrlStateMsg> state;
  int writeData=0;
  RobotAbstraction() {}
  RobotAbstraction(const Var<rai::CtrlCmdMsg>& _cmd, const Var<rai::CtrlStateMsg>& _state) : cmd(_cmd), state(_state) {}
};


} //namespace
