#include "flag.h"
#include "frame.h"

//===========================================================================

template<> const char* mlr::Enum<FrameFlagType>::names []={
  "FT_zeroVel",
  "FT_zeroAcc",
  "FT_gravityAcc",
  "FT_zeroQVel",
  "FT_zeroQAcc",
  "FT_noQControlCosts",
  NULL
};

//===========================================================================

void mlr::Flag::apply(mlr::KinematicWorld &K){
  mlr::Frame &a = *K.frames(frameId);
  if(setTrue) a.flags |= (1<<flag.x);
  else a.flags &= ~(1<<flag.x);
}
