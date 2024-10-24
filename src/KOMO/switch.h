/*  ------------------------------------------------------------------
    Copyright (c) 2011-2024 Marc Toussaint
    email: toussaint@tu-berlin.de

    This code is distributed under the MIT License.
    Please see <root-path>/LICENSE for details.
    --------------------------------------------------------------  */

#pragma once

#include "../Kin/frame.h"

namespace rai {

enum SwitchType {
  SW_none=-1,
  SW_noJointLink=0,
  SW_joint,
  makeDynamic,
  makeKinematic,
  SW_delContact,
  SW_addContact,
  SW_addPOAonly,
};

enum SwitchInitializationType : int {
  SWInit_zero=0,
  SWInit_copy,
  SWInit_random
};

struct KinematicSwitch {
  Enum<SwitchType> symbol;
  Enum<JointType> jointType;
  Enum<SwitchInitializationType> init;
  int timeOfApplication;
  int fromId, toId;
  bool isStable=false;
  rai::Transformation rel;
  KinematicSwitch();
  KinematicSwitch(SwitchType op, JointType type,
                  int aFrame, int bFrame,
                  SwitchInitializationType _init=SWInit_zero,
                  int _timeOfApplication=0,
                  const rai::Transformation& _rel=NoTransformation); //, const rai::Transformation& jTo=NoTransformation);
  KinematicSwitch(SwitchType op, JointType type,
                  const char* ref1, const char* ref2,
                  const rai::Configuration& K,
                  SwitchInitializationType _init=SWInit_zero,
                  int _timeOfApplication=0,
                  const rai::Transformation& _rel=NoTransformation); //, const rai::Transformation& jTo=NoTransformation);
  void setTimeOfApplication(double time, bool before, int stepsPerPhase, uint T);
  Frame* apply(FrameL& frames) const;
  rai::String shortTag(const Configuration* G) const;
  void write(std::ostream& os, const FrameL& frames= {}) const;
};

} // namespace rai

stdOutPipe(rai::KinematicSwitch)
