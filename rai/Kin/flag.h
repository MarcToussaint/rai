#pragma once

#include <Kin/kin.h>

enum FrameFlagType {
  FT_zeroVel=0,
  FT_zeroAcc,
  FT_gravityAcc,
  FT_zeroQVel,
  FT_zeroQAcc,
  FT_noQControlCosts
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

  void write(std::ostream& os) const{
    os <<"FLAG '" <<flag<<"'"
      <<"  frame=" <<frameId
      <<"  stepOfApplication=" <<stepOfApplication
      <<"  persist=" <<persist
      <<"  setTrue=" <<setTrue;
  }
};

}
stdOutPipe(mlr::Flag)
