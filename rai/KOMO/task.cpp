/*  ------------------------------------------------------------------
    Copyright (c) 2017 Marc Toussaint
    email: marc.toussaint@informatik.uni-stuttgart.de

    This code is distributed under the MIT License.
    Please see <root-path>/LICENSE for details.
    --------------------------------------------------------------  */

#include "task.h"
#include <Core/graph.h>

//===========================================================================

void Task::setCostSpecs(int fromStep, int toStep, const arr& _target, double _prec) {
  if(&_target) target = _target; else target = {0.};
  if(fromStep<0) fromStep=0;
  CHECK(toStep>=fromStep,"");
  prec.resize(toStep+1).setZero();
  for(uint t=fromStep; t<=(uint)toStep; t++) prec(t) = _prec;
}

#define STEP(t) (floor(t*double(stepsPerPhase) + .500001))-1

void Task::setCostSpecs(double fromTime, double toTime, int stepsPerPhase, uint T, const arr& _target, double _prec, int deltaStep) {
  if(stepsPerPhase<0) stepsPerPhase=T;
//  if(STEP(toTime)>T-1){
//    LOG(-1) <<"beyond the time!: endTime=" <<toTime <<" phases=" <<double(T)/stepsPerPhase;
//  }
  int tFrom = (fromTime<0.?0:STEP(fromTime));
  int tTo = (toTime<0.?T-1:STEP(toTime));
  if(tTo<0) tTo=0;
  if(tFrom>tTo && tFrom-tTo<=(int)map->order) tFrom=tTo;
  
  if(deltaStep) { tFrom+=deltaStep; tTo+=deltaStep; }
  
  setCostSpecs(tFrom, tTo, _target, _prec);
}

//===========================================================================
