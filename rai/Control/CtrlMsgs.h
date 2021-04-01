/*  ------------------------------------------------------------------
    Copyright (c) 2017 Marc Toussaint
    email: marc.toussaint@informatik.uni-stuttgart.de

    This code is distributed under the MIT License.
    Please see <root-path>/LICENSE for details.
    --------------------------------------------------------------  */

#pragma once

#include <Core/array.h>

namespace rai {

enum class ControlType { configRefs, projectedAcc };

typedef std::function<void(arr& q_ref, arr& qDot_ref, arr& qDDot_ref, double time)> ReferenceFunction;

//The control message send to the robot
struct CtrlCmdMsg {
  ControlType controlType=ControlType::configRefs;
  ReferenceFunction ref; // joint space references
//  arr qRef, qDotRef, qDDotRef; // joint space references
  arr u_b; // open-loop/feed-forward torque term
  arr Kp, Kd; // gain matrices
  arr P_compliance;
};

// The state message comming back from the robot
struct CtrlStateMsg {
  double time=0.;
  arr q, qDot; // actual joint state
  arr tauExternal; // external torques
};

} //namespace
