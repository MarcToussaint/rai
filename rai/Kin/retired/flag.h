/*  ------------------------------------------------------------------
    Copyright (c) 2017 Marc Toussaint
    email: marc.toussaint@informatik.uni-stuttgart.de

    This code is distributed under the MIT License.
    Please see <root-path>/LICENSE for details.
    --------------------------------------------------------------  */

#pragma once

#include <Kin/kin.h>

enum FrameFlagType {
  FL_zeroVel=0,
  FL_zeroAcc,
  FL_gravityAcc,
  FL_zeroQVel,
  FL_zeroQAcc,
  FL_normalControlCosts,
  FL_impulseExchange,
  FL_qCtrlCostAcc,
  FL_qCtrlCostVel,
  FL_xPosAccCosts,
  FL_clear,
  FL_xPosVelCosts,
  FL_kinematic,
  FL_something
};

namespace rai {

struct Flag {
  rai::Enum<FrameFlagType> flag;
  uint frameId;
  uint stepOfApplication;
  bool persist=false;
  bool setTrue=true;
  
  Flag(FrameFlagType flag, uint frameId, uint stepOfApplication=0, bool persist=false, bool setTrue=true)
    : flag(flag), frameId(frameId), stepOfApplication(stepOfApplication), persist(persist), setTrue(setTrue) {}
  ~Flag() {}
  
  void apply(KinematicWorld& K);
  
  void write(std::ostream& os, KinematicWorld* K=NULL) const;
};

}
stdOutPipe(rai::Flag)
