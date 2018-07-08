/*  ------------------------------------------------------------------
    Copyright (c) 2017 Marc Toussaint
    email: marc.toussaint@informatik.uni-stuttgart.de

    This code is distributed under the MIT License.
    Please see <root-path>/LICENSE for details.
    --------------------------------------------------------------  */

#include "objective.h"
#include <Kin/switch.h>
#include <Core/graph.h>

//===========================================================================

void Objective::setCostSpecs(int fromStep, int toStep, const arr& _target, double _prec) {
  if(&_target) target = _target; else target.clear();
  if(fromStep<0) fromStep=0;
  CHECK_GE(toStep, fromStep,"");
  prec.resize(toStep+1).setZero();
  for(uint t=fromStep; t<=(uint)toStep; t++) prec(t) = _prec;
#if 1
  vars.resize(prec.N, map->order+1);
  for(uint t=0;t<vars.d0;t++)
    for(int i=0;i<(int)map->order+1;i++) vars(t,i) = t+i-(int)map->order;
#endif
}

void Objective::setCostSpecs(double fromTime, double toTime, int stepsPerPhase, uint T, const arr& _target, double _prec, int deltaStep) {
  if(stepsPerPhase<0) stepsPerPhase=T;
//  if(conv_time2step(toTime, stepsPerPhase)>T-1){
//    LOG(-1) <<"beyond the time!: endTime=" <<toTime <<" phases=" <<double(T)/stepsPerPhase;
//  }
  int tFrom = (fromTime<0.?0:conv_time2step(fromTime, stepsPerPhase));
  int tTo = (toTime<0.?T-1:conv_time2step(toTime, stepsPerPhase));
  if(tTo<0) tTo=0;
  if(tFrom>tTo && tFrom-tTo<=(int)map->order) tFrom=tTo;
  
  if(deltaStep) { tFrom+=deltaStep; tTo+=deltaStep; }
  
  setCostSpecs(tFrom, tTo, _target, _prec);
}

void Objective::setCostSpecsDense(intA _vars, const arr& _target, double _prec) {
  if(&_target) target = _target; else target.clear();
  prec = ARR(_prec);
  vars = _vars;
  vars.reshape(1, vars.N);
}


//===========================================================================
