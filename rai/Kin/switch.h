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

enum SwitchType {
  none=-1,
  deleteJoint=0,
  SW_effJoint,
  addJointAtFrom,
  addJointAtTo,
  SW_actJoint,
  addSliderMechanism,
  SW_insertEffJoint,
  insertActuated,
  makeDynamic,
  makeKinematic,
  SW_fixCurrent
};

struct KinematicSwitch{
  Enum<SwitchType> symbol;
  Enum<JointType> jointType;
  uint timeOfApplication;
  uint fromId, toId;
  mlr::Transformation jA,jB;
  KinematicSwitch();
  KinematicSwitch(SwitchType op, JointType type,
                  const char* ref1, const char* ref2,
                  const mlr::KinematicWorld& K, uint _timeOfApplication=0,
                  const mlr::Transformation& jFrom=NoTransformation, const mlr::Transformation& jTo=NoTransformation);
  void setTimeOfApplication(double time, bool before, int stepsPerPhase, uint T);
  void apply(KinematicWorld& K);
  void temporallyAlign(const KinematicWorld& Gprevious, KinematicWorld& G, bool copyFromBodies);
  mlr::String shortTag(const KinematicWorld* G) const;
  void write(std::ostream& os, mlr::KinematicWorld *K=NULL) const;
  static KinematicSwitch* newSwitch(const Node *specs, const mlr::KinematicWorld& world, int stepsPerPhase, uint T);
  static KinematicSwitch* newSwitch(const mlr::String& type, const char* ref1, const char* ref2, const mlr::KinematicWorld& world, uint _timeOfApplication, const mlr::Transformation& jFrom=NoTransformation, const mlr::Transformation& jTo=NoTransformation);
  static const char* name(SwitchType s);
};

} // namespace mlr

stdOutPipe(mlr::KinematicSwitch)

#endif
