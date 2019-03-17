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
  ptr<Feature> map;
  const rai::Enum<ObjectiveType> type;  ///< element of {sumOfSqr, inequality, equality}
  rai::String name;
  intA vars; //either a (0,1)-indicator per time slice, or a list of variable tuples
  
  Objective(const ptr<Feature>& _map, const ObjectiveType& _type) : map(_map), type(_type) {}
  ~Objective() {}
  
  void setCostSpecs(int fromStep, int toStep);
  void setCostSpecs(double fromTime, double toTime, int stepsPerPhase, uint T,
                    int deltaFromStep=0, int deltaToStep=0);
  void setCostSpecsDense(const intA& _vars);
  bool isActive(uint t);
  void write(std::ostream& os) const;
};
stdOutPipe(Objective)
