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

void Objective::setCostSpecs(int fromStep, int toStep, bool sparse) {
  CHECK_GE(fromStep, 0, "");
  CHECK_GE(toStep, fromStep, "");
  if(!sparse){
    vars.resize(toStep+1).setZero();
    for(uint t=fromStep; t<=(uint)toStep; t++) vars(t) = 1;
  }else{
    vars.resize(1+toStep-fromStep, map->order+1);
    for(uint t=fromStep; t<=(uint)toStep; t++)
      for(uint i=0;i<vars.d1;i++) vars(t-fromStep,i) = t+i-int(map->order);
  }
#if 0
  vars.resize(prec.N, map->order+1);
  for(uint t=0;t<vars.d0;t++)
    for(int i=0;i<(int)map->order+1;i++) vars(t,i) = t+i-(int)map->order;
#endif
}

void Objective::setCostSpecs(double fromTime, double toTime, int stepsPerPhase, uint T,
                             int deltaFromStep, int deltaToStep, bool sparse) {

  if(toTime>double(T)/stepsPerPhase+1.){
    LOG(-1) <<"beyond the time!: endTime=" <<toTime <<" phases=" <<double(T)/stepsPerPhase;
  }

//  if(stepsPerPhase<0) stepsPerPhase=T;
  CHECK_GE(stepsPerPhase, 0, "");
//  if(conv_time2step(toTime, stepsPerPhase)>T-1){
//    LOG(-1) <<"beyond the time!: endTime=" <<toTime <<" phases=" <<double(T)/stepsPerPhase;
//  }
  int tFrom = (fromTime<0.?0:conv_time2step(fromTime, stepsPerPhase));
  int tTo = (toTime<0.?T-1:conv_time2step(toTime, stepsPerPhase));
  
  if(fromTime>=0 && deltaFromStep) tFrom+=deltaFromStep;
  if(toTime>=0 && deltaToStep) tTo+=deltaToStep;

  if(tFrom<0) tFrom=0;
  if(tTo<0) tTo=0;
  if(tTo>=(int)T) tTo=T-1;

  setCostSpecs(tFrom, tTo, sparse);
}

void Objective::setCostSpecsDense(const intA& _vars) {
  HALT("why do we need this??");
  vars = _vars;
  vars.reshape(1, vars.N);
}

bool Objective::isActive(uint t) {
  if(!vars.N) return false;
  CHECK_EQ(vars.nd, 1, "variables are not time indexed (tuples for dense problem instead)");
  return (vars.N>t && vars(t));
}

void Objective::write(std::ostream& os) const {
  os <<"TASK '" <<name <<"'";
  if(vars.N){
    if(vars.nd==1){
      if(vars.N>4) writeConsecutiveConstant(os,vars);
      else os <<" ("<<vars <<')';
    }else os <<" (" <<vars.first() <<".." <<vars.last() <<')';
  }else os <<" ()";
  os <<"  type=" <<type
    <<"  order=" <<map->order
   <<"  target=[" <<map->target <<']'
  <<"  scale=" <<map->scale;
}


//===========================================================================
