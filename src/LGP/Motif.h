#pragma once

#include <KOMO/komo.h>

struct KOMO_Motif{
  //these are set by the 'anayze' method
  rai::Array<GroundedObjective*> objs;
  FrameL F;
  int timeSlice=-1;

  bool matches(GroundedObjective* ob, int _timeSlice){
    CHECK(objs.N, "");
    if(_timeSlice != timeSlice) return false;
    FrameL shared = setSection(F, ob->frames);
    if(!shared.N) return false;
    return true;
  }

  rai::String getHash();

  DofL getDofs(rai::Configuration& C, int verbose);

//  void initialize(KOMO& komo, const arr& x){
//    DofL dofs = getDofs(komo.pathConfig);
//    komo.pathConfig.setDofState(x, dofs);
//  }

  std::shared_ptr<SolverReturn> solve(KOMO& komo, int verbose);

};

typedef rai::Array<std::shared_ptr<KOMO_Motif>> MotifL;

