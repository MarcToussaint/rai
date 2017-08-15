/*  ------------------------------------------------------------------
    Copyright 2016 Marc Toussaint
    email: marc.toussaint@informatik.uni-stuttgart.de

    This program is free software: you can redistribute it and/or modify
    it under the terms of the GNU General Public License as published by
    the Free Software Foundation, either version 3 of the License, or (at
    your option) any later version. This program is distributed without
    any warranty. See the GNU General Public License for more details.
    You should have received a COPYING file of the full GNU General Public
    License along with this program. If not, see
    <http://www.gnu.org/licenses/>
    --------------------------------------------------------------  */


#ifndef MLR_switch_h
#define MLR_switch_h

#include "frame.h"

namespace mlr{

struct KinematicSwitch{
  enum OperatorSymbol{ none=-1, deleteJoint=0, addJointZero, addJointAtFrom, addJointAtTo, addActuated, addSliderMechanism, insertJoint };
  Enum<OperatorSymbol> symbol;
  Enum<JointType> jointType;
  uint timeOfApplication;
  uint fromId, toId;
  mlr::Transformation jA,jB;
  KinematicSwitch();
  KinematicSwitch(OperatorSymbol op, JointType type,
                  const char* ref1, const char* ref2,
                  const mlr::KinematicWorld& K, uint _timeOfApplication,
                  const mlr::Transformation& jFrom=NoTransformation, const mlr::Transformation& jTo=NoTransformation);
  void setTimeOfApplication(double time, bool before, int stepsPerPhase, uint T);
  void apply(KinematicWorld& G);
  void temporallyAlign(const KinematicWorld& Gprevious, KinematicWorld& G, bool copyFromBodies);
  mlr::String shortTag(const KinematicWorld* G) const;
  void write(std::ostream& os, mlr::KinematicWorld *K=NULL) const;
  static KinematicSwitch* newSwitch(const Node *specs, const mlr::KinematicWorld& world, int stepsPerPhase, uint T);
  static KinematicSwitch* newSwitch(const mlr::String& type, const char* ref1, const char* ref2, const mlr::KinematicWorld& world, uint _timeOfApplication, const mlr::Transformation& jFrom=NoTransformation, const mlr::Transformation& jTo=NoTransformation);
  static const char* name(OperatorSymbol s);
};

} // namespace mlr

stdOutPipe(mlr::KinematicSwitch)

#endif
