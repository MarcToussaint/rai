/*  ------------------------------------------------------------------
    Copyright (c) 2017 Marc Toussaint
    email: marc.toussaint@informatik.uni-stuttgart.de

    This code is distributed under the MIT License.
    Please see <root-path>/LICENSE for details.
    --------------------------------------------------------------  */

#ifndef RAI_switch_h
#define RAI_switch_h

#include "frame.h"

namespace rai {

enum SwitchType {
  SW_none=-1,
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
  SW_fixCurrent,
  SW_delContact,
  SW_addContact,
  SW_addComplementaryContact,
};

enum SwitchInitializationType {
  SWInit_zero=0,
  SWInit_copy,
  SWInit_random
};

struct KinematicSwitch {
  Enum<SwitchType> symbol;
  Enum<JointType> jointType;
  Enum<SwitchInitializationType> init;
  int timeOfApplication;
  uint fromId, toId;
  rai::Transformation jA,jB;
  KinematicSwitch();
  KinematicSwitch(SwitchType op, JointType type,
                  const char* ref1, const char* ref2,
                  const rai::KinematicWorld& K, int _timeOfApplication=0,
                  const rai::Transformation& jFrom=NoTransformation, const rai::Transformation& jTo=NoTransformation);
  void setTimeOfApplication(double time, bool before, int stepsPerPhase, uint T);
  void apply(KinematicWorld& K);
  void temporallyAlign(const KinematicWorld& Gprevious, KinematicWorld& G, bool copyFromBodies);
  rai::String shortTag(const KinematicWorld* G) const;
  void write(std::ostream& os, rai::KinematicWorld *K=NULL) const;
  static KinematicSwitch* newSwitch(const Node *specs, const rai::KinematicWorld& world, int stepsPerPhase, uint T);
  static KinematicSwitch* newSwitch(const rai::String& type, const char* ref1, const char* ref2, const rai::KinematicWorld& world, int _timeOfApplication, const rai::Transformation& jFrom=NoTransformation, const rai::Transformation& jTo=NoTransformation);
  static const char* name(SwitchType s);
};

} // namespace rai

stdOutPipe(rai::KinematicSwitch)

#endif

int conv_time2step(double time, uint stepsPerPhase);
