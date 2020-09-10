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
  int i;               ///< which shapes does it refer to?
  bool impulseInsteadOfAcceleration=false;

  TM_LinVel(int iShape=-1) : i(iShape) { order=1; }
  TM_LinVel(const rai::Configuration& K, const char* iShapeName=nullptr) : i(initIdArg(K, iShapeName)) { order=1; }

  virtual void phi2(arr& y, arr& J, const FrameL& F);
  virtual void phi(arr& y, arr& J, const rai::Configuration& G) { NIY; }
  virtual void phi(arr& y, arr& J, const ConfigurationL& Ktuple);
  virtual uint dim_phi(const rai::Configuration& G) { return 3; }
  virtual rai::String shortTag(const rai::Configuration& G) { return STRING("TM_LinVel-" <<order <<'-' <<G.frames(i)->name); }
};

//===========================================================================

struct TM_AngVel : Feature {
  int i;               ///< which shapes does it refer to?
  bool impulseInsteadOfAcceleration=false;

  TM_AngVel(int iShape=-1) : i(iShape) { order=1; }
  TM_AngVel(const rai::Configuration& K, const char* iShapeName=nullptr) : i(initIdArg(K, iShapeName)) { order=1; }

  virtual void phi2(arr& y, arr& J, const FrameL& F);
  virtual void phi(arr& y, arr& J, const rai::Configuration& G) { NIY; }
  virtual void phi(arr& y, arr& J, const ConfigurationL& Ktuple);
  virtual uint dim_phi(const rai::Configuration& G);
  virtual rai::String shortTag(const rai::Configuration& G) { return STRING("AngVel-" <<order <<'-' <<G.frames(i)->name); }
};

//===========================================================================

struct TM_LinAngVel : Feature {
  bool impulseInsteadOfAcceleration=false;

  TM_LinAngVel(int iShape) { frameIDs=TUP(iShape); order=1; }
  TM_LinAngVel(const rai::Configuration& K, const char* iShapeName) : TM_LinAngVel(initIdArg(K, iShapeName)) {}

  virtual void phi(arr& y, arr& J, const rai::Configuration& G) { NIY; }
  virtual void phi(arr& y, arr& J, const ConfigurationL& Ctuple);
  virtual void phi2(arr& y, arr& J, const FrameL& F);
  virtual uint dim_phi2(const FrameL& F){ return 6; }
  virtual rai::String shortTag(const rai::Configuration& G) { return STRING("LinAngVel"); }
};

//===========================================================================

struct TM_NoJumpFromParent : Feature {
  TM_NoJumpFromParent(const rai::Configuration& C, const char* iShapeName) { frameIDs = TUP(initIdArg(C, iShapeName)); }
  virtual void phi2(arr& y, arr& J, const FrameL& F);
  virtual uint dim_phi2(const FrameL& F){ return 7; }
};
