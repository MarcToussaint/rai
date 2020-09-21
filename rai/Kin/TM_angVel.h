/*  ------------------------------------------------------------------
    Copyright (c) 2011-2020 Marc Toussaint
    email: toussaint@tu-berlin.de

    This code is distributed under the MIT License.
    Please see <root-path>/LICENSE for details.
    --------------------------------------------------------------  */

#pragma once

#include "feature.h"

//===========================================================================

struct TM_LinVel : Feature {
  bool impulseInsteadOfAcceleration=false;
  TM_LinVel() { order=1; }
  virtual void phi2(arr& y, arr& J, const FrameL& F);
  virtual uint dim_phi2(const FrameL& F) { return 3; }
};

//===========================================================================

struct TM_AngVel : Feature {
  bool impulseInsteadOfAcceleration=false;
  TM_AngVel() { order=1; }
  virtual void phi2(arr& y, arr& J, const FrameL& F);
  virtual uint dim_phi2(const FrameL& G) { return 3; }
};

//===========================================================================

struct TM_LinAngVel : Feature {
  bool impulseInsteadOfAcceleration=false;
  TM_LinAngVel() { order=1; }
  virtual void phi2(arr& y, arr& J, const FrameL& F);
  virtual uint dim_phi2(const FrameL& F){ return 6; }
};

//===========================================================================

struct TM_NoJumpFromParent : Feature {
  TM_NoJumpFromParent(const rai::Configuration& C, const char* iShapeName) { frameIDs = TUP(initIdArg(C, iShapeName)); }
  virtual void phi2(arr& y, arr& J, const FrameL& F);
  virtual uint dim_phi2(const FrameL& F){ return 7; }
};
