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

struct ReferenceFeed {
  /// callback called by a robot control loop
  virtual void getReference(arr& q_ref, arr& qDot_ref, arr& qDDot_ref, const arr& q_real, const arr& qDot_real, double ctrlTime) = 0;
};

//The control message send to the robot
struct CtrlCmdMsg {
  ControlType controlType=ControlType::configRefs;
  shared_ptr<ReferenceFeed> ref; // joint space references
//  arr q_ref, qDot_ref, qDDot_ref; // joint space references
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
