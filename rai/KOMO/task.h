/*  ------------------------------------------------------------------
    Copyright (c) 2017 Marc Toussaint
    email: marc.toussaint@informatik.uni-stuttgart.de
    
    This code is distributed under the MIT License.
    Please see <root-path>/LICENSE for details.
    --------------------------------------------------------------  */

#pragma once

#include <Optim/optimization.h>
#include <Kin/taskMap.h>

struct Task {
  TaskMap *map;
  const mlr::Enum<ObjectiveType> type;  ///< element of {sumOfSqr, inequality, equality}
  mlr::String name;
  arr target, prec;     ///< optional linear, time-dependent, rescaling (with semantics of target & precision)

  Task(TaskMap *m, const ObjectiveType& type) : map(m), type(type){}
  ~Task(){ if(map) delete map; map=NULL; }

  void setCostSpecs(int fromStep, int toStep, const arr& _target={}, double _prec=1.);
  void setCostSpecs(double fromTime, double toTime, int stepsPerPhase, uint T,
                    const arr& _target, double _prec, int deltaStep=0);
  bool isActive(uint t){ return (prec.N>t && prec(t)); }
  void write(std::ostream& os) const{
    os <<"TASK '" <<name <<"'"
      <<"  type=" <<type
      <<"  order=" <<map->order
      <<"  target=[" <<target <<']'
      <<"  prec=";
    writeConsecutiveConstant(os,prec);
  }
};
stdOutPipe(Task)
