#pragma once

#include <KOMO/komo.h>

struct KOMO_Motif{
  //these are set by the 'anayze' method
  rai::Array<GroundedObjective*> objs;
  FrameL F;
  int timeSlice=-1;

  bool matches(GroundedObjective* ob, int _timeSlice);

  rai::String getHash();

  DofL getDofs(rai::Configuration& C, int verbose);

//  void initialize(KOMO& komo, const arr& x){
//    DofL dofs = getDofs(komo.pathConfig);
//    komo.pathConfig.setDofState(x, dofs);
//  }

  std::shared_ptr<SolverReturn> solve(KOMO& komo, str opt_or_sample, int verbose);

};

typedef rai::Array<std::shared_ptr<KOMO_Motif>> MotifL;

