/*  ------------------------------------------------------------------
    Copyright (c) 2017 Marc Toussaint
    email: marc.toussaint@informatik.uni-stuttgart.de

    This code is distributed under the MIT License.
    Please see <root-path>/LICENSE for details.
    --------------------------------------------------------------  */

#pragma once

#include <Optim/optimization.h>
#include <Kin/feature.h>

struct Objective {
  Feature *map;
  const rai::Enum<ObjectiveType> type;  ///< element of {sumOfSqr, inequality, equality}
  rai::String name;
  arr target, prec;     ///< optional linear, time-dependent, rescaling (with semantics of target & precision)
  intA vars;
  
  Objective(Feature *m, const ObjectiveType& type) : map(m), type(type) {}
  ~Objective() { if(map) delete map; map=NULL; }
  
  void setCostSpecs(int fromStep, int toStep, const arr& _target= {}, double _prec=1.);
  void setCostSpecs(double fromTime, double toTime, int stepsPerPhase, uint T,
                    const arr& _target, double _prec, int deltaStep=0);
  void setCostSpecsDense(intA _vars, const arr& _target, double _prec);
  bool isActive(uint t) { return (prec.N>t && prec(t)); }
  void write(std::ostream& os) const {
    os <<"TASK '" <<name <<"'";
    if(vars.d0==1) os <<" ("<<vars <<')';
    os <<"  type=" <<type
       <<"  order=" <<map->order
       <<"  target=[" <<target <<']'
       <<"  prec=";
    if(prec.N>4) writeConsecutiveConstant(os,prec); else os <<'[' <<prec <<']';
  }
};
stdOutPipe(Objective)
