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
  ObjectiveType type;  ///< element of {f, sumOfSqr, inequality, equality}
  rai::String name;
  arr times;

  Objective(const ptr<Feature>& _feat, const ObjectiveType& _type, const rai::String& _name, const arr& _times)
    : feat(_feat), type(_type), name(_name), times(_times) {}

  void write(std::ostream& os) const;
};
stdOutPipe(Objective)

struct GroundedObjective {
  std::shared_ptr<Feature> feat;
  const rai::Enum<ObjectiveType> type;  ///< element of {f, sumOfSqr, inequality, equality}
  FrameL frames;
  intA timeSlices;
  int objId=-1;

  GroundedObjective(const ptr<Feature>& _feat, const ObjectiveType& _type) : feat(_feat), type(_type) {}
  ~GroundedObjective() {}

  rai::String name(){ return feat->shortTag(frames.first()->C); }
};
