#include "flag.h"
#include "frame.h"

//===========================================================================

template<> const char* mlr::Enum<FrameFlagType>::names []={
  "FL_zeroVel",
  "FL_zeroAcc",
  "FL_gravityAcc",
  "FL_zeroQVel",
  "FL_zeroQAcc",
  "FL_noQControlCosts",
  "FL_impulseExchange",
  "FL_qCtrlCostAcc",
  "FL_xPosAccCosts",
  "FL_clear",
  NULL
};

//===========================================================================

void mlr::Flag::apply(mlr::KinematicWorld &K){
  mlr::Frame &a = *K.frames(frameId);
  if(flag.x==FL_clear){ a.flags=0; return; }
  if(setTrue) a.flags |= (1<<flag.x);
  else a.flags &= ~(1<<flag.x);
}

void mlr::Flag::write(std::ostream &os, mlr::KinematicWorld *K) const{
  os <<"FLAG '" <<flag<<"'"
    <<"  frame=" <<frameId;
  if(K) os <<"'" <<K->frames(frameId)->name <<"'";
  os <<"  stepOfApplication=" <<stepOfApplication
    <<"  persist=" <<persist
   <<"  setTrue=" <<setTrue;
}
