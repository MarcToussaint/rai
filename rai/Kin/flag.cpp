/*  ------------------------------------------------------------------
    Copyright (c) 2017 Marc Toussaint
    email: marc.toussaint@informatik.uni-stuttgart.de
    
    This code is distributed under the MIT License.
    Please see <root-path>/LICENSE for details.
    --------------------------------------------------------------  */

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
  "FL_xPosVelCosts",
  "FL_kinematic",
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
