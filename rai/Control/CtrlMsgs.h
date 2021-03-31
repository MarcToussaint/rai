/*  ------------------------------------------------------------------
    Copyright (c) 2017 Marc Toussaint
    email: marc.toussaint@informatik.uni-stuttgart.de

    This code is distributed under the MIT License.
    Please see <root-path>/LICENSE for details.
    --------------------------------------------------------------  */

#pragma once

#include <Core/array.h>

enum class ControlType { configRefs, projectedAcc };

//The control message send to the robot
struct CtrlCmdMsg {
  ControlType controlType=ControlType::configRefs;
  arr qRef, qDotRef; // joint space references
  arr qDDotRef; // joint acceleration feedforward reference
  arr u_b; // open-loop/feed-forward torque term
  arr Kp, Kd; // gain matrices
  arr P_compliance;
};

// The state message comming back from the robot
struct CtrlStateMsg {
  arr q, qDot; // actual joint state
  arr tauExternal; // external torques
};
