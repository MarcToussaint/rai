/*  ------------------------------------------------------------------
    Copyright (c) 2011-2020 Marc Toussaint
    email: toussaint@tu-berlin.de

    This code is distributed under the MIT License.
    Please see <root-path>/LICENSE for details.
    --------------------------------------------------------------  */

#include "flag.h"
#include "frame.h"

//===========================================================================

template<> const char* rai::Enum<FrameFlagType>::names []= {
  "FL_zeroVel",
  "FL_zeroAcc",
  "FL_gravityAcc",
  "FL_zeroQVel",
  "FL_zeroQAcc",
  "FL_normalControlCosts",
  "FL_impulseExchange",
  "FL_qCtrlCostAcc",
  "FL_qCtrlCostVel",
  "FL_xPosAccCosts",
  "FL_clear",
  "FL_xPosVelCosts",
  "FL_kinematic",
  "FL_something",
  nullptr
};

//===========================================================================

void rai::Flag::apply(rai::Configuration& K) {
  rai::Frame& a = *K.frames(frameId);
  if(flag.x==FL_clear) { a.flags=0; return; }
  if(setTrue) a.flags |= (1<<flag.x);
  else a.flags &= ~(1<<flag.x);
}

void rai::Flag::write(std::ostream& os, rai::Configuration* K) const {
  os <<"FLAG '" <<flag<<"'"
     <<"  frame=" <<frameId;
  if(K) os <<"'" <<K->frames(frameId)->name <<"'";
  os <<"  stepOfApplication=" <<stepOfApplication
     <<"  persist=" <<persist
     <<"  setTrue=" <<setTrue;
}
