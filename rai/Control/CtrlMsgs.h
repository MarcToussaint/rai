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
  virtual void getReference(arr& qRef, arr& qDotRef, arr& qDDotRef, const arr& q_real, const arr& qDot_real, double time) = 0;

};

//The control message send to the robot
struct CtrlCmdMsg {
  ControlType controlType=ControlType::configRefs;
  shared_ptr<ReferenceFeed> ref; // joint space references
//  ReferenceFunction ref; // joint space references
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
