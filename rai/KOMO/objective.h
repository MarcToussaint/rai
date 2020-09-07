/*  ------------------------------------------------------------------
    Copyright (c) 2011-2020 Marc Toussaint
    email: toussaint@tu-berlin.de

    This code is distributed under the MIT License.
    Please see <root-path>/LICENSE for details.
    --------------------------------------------------------------  */

#pragma once

#include "../Optim/optimization.h"
#include "../Kin/feature.h"

struct Objective {
  std::shared_ptr<Feature> feat;
  const rai::Enum<ObjectiveType> type;  ///< element of {f, sumOfSqr, inequality, equality}
  rai::String name;
  intA configs; //either a (0,1)-indicator per time slice, or a list of variable tuples

  Objective(const ptr<Feature>& _feat, const ObjectiveType& _type, const rai::String& _name=rai::String()) : feat(_feat), type(_type), name(_name) {}
  ~Objective() {}

  void setCostSpecs(int fromStep, int toStep, bool tuples=false);
  void setCostSpecs(const arr& times, int stepsPerPhase, uint T,
                    int deltaFromStep=0, int deltaToStep=0, bool tuples=false);
  bool isActive(uint t);
  void write(std::ostream& os) const;
};
stdOutPipe(Objective)

struct GroundedObjective {
  std::shared_ptr<Feature> feat;
  const rai::Enum<ObjectiveType> type;  ///< element of {f, sumOfSqr, inequality, equality}
  FrameL frames;
  intA configs;
  int objId=-1;

  GroundedObjective(const ptr<Feature>& _feat, const ObjectiveType& _type) : feat(_feat), type(_type) {}
  ~GroundedObjective() {}
};
