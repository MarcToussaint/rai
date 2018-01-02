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
  FL_xPosAccCosts,
  FL_clear,
};

namespace mlr{

struct Flag {
  mlr::Enum<FrameFlagType> flag;
  uint frameId;
  uint stepOfApplication;
  bool persist=false;
  bool setTrue=true;

  Flag(FrameFlagType flag, uint frameId, uint stepOfApplication=0, bool persist=false, bool setTrue=true)
    : flag(flag), frameId(frameId), stepOfApplication(stepOfApplication), persist(persist), setTrue(setTrue){}
  ~Flag(){}

  void apply(KinematicWorld& K);

  void write(std::ostream& os, KinematicWorld* K=NULL) const;
};

}
stdOutPipe(mlr::Flag)
