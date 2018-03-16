/*  ------------------------------------------------------------------
    Copyright (c) 2017 Marc Toussaint
    email: marc.toussaint@informatik.uni-stuttgart.de
    
    This code is distributed under the MIT License.
    Please see <root-path>/LICENSE for details.
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
