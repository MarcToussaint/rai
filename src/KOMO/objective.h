/*  ------------------------------------------------------------------
    Copyright (c) 2011-2024 Marc Toussaint
    email: toussaint@tu-berlin.de

    This code is distributed under the MIT License.
    Please see <root-path>/LICENSE for details.
    --------------------------------------------------------------  */

#pragma once

#include "../Core/util.h"
#include "../Core/array.h"

//===========================================================================

namespace rai{
struct Frame;
struct Configuration;
}
struct Feature;
enum FeatureSymbol : int;
enum ObjectiveType : int;

//===========================================================================

struct Objective {
  std::shared_ptr<Feature> feat;
  ObjectiveType type;  ///< element of {f, sumOfSqr, inequality, equality}
  rai::String name;
  arr times;
  rai::Array<struct GroundedObjective*> groundings;

  Objective(const shared_ptr<Feature>& _feat, const ObjectiveType& _type, const rai::String& _name, const arr& _times)
    : feat(_feat), type(_type), name(_name), times(_times) {}

  bool activeAtTime(double time);
  void write(std::ostream& os) const;
};
stdOutPipe(Objective)

//===========================================================================

struct GroundedObjective {
  std::shared_ptr<Feature> feat;
  const rai::Enum<ObjectiveType> type;  ///< element of {f, sumOfSqr, inequality, equality}
  rai::Array<rai::Frame*> frames;
  intA timeSlices;
  int objId=-1;
  bool active = true;

  GroundedObjective(const shared_ptr<Feature>& _feat, const ObjectiveType& _type, const intA& _timeSlices) : feat(_feat), type(_type), timeSlices(_timeSlices) {}
  ~GroundedObjective() {}

  rai::String name();
};

//===========================================================================

struct ObjectiveL : rai::Array<shared_ptr<Objective>> {

  shared_ptr<struct Objective> add(const arr& times, const shared_ptr<Feature>& f, ObjectiveType type, const char* name=0);

  shared_ptr<struct Objective> add(const arr& times,
                                   const shared_ptr<Feature>& f, const rai::Configuration& C, const StringA& frames,
                                   ObjectiveType type, const arr& scale=NoArr, const arr& target=NoArr, int order=-1, int deltaFromStep=0, int deltaToStep=0);

  shared_ptr<struct Objective> add(const arr& times,
                                   const FeatureSymbol& feat,  const rai::Configuration& C, const StringA& frames,
                                   ObjectiveType type, const arr& scale=NoArr, const arr& target=NoArr, int order=-1, int deltaFromStep=0, int deltaToStep=0);

  double maxError(const rai::Configuration& C, double time, int verbose=0) const;
};
