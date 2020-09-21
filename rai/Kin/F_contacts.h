/*  ------------------------------------------------------------------
    Copyright (c) 2011-2020 Marc Toussaint
    email: toussaint@tu-berlin.de

    This code is distributed under the MIT License.
    Please see <root-path>/LICENSE for details.
    --------------------------------------------------------------  */

#pragma once

#include "feature.h"

struct TM_Contact_POA : Feature {
  void phi2(arr& y, arr& J, const FrameL& F);
  uint dim_phi2(const FrameL& F) { return 3; }
};

struct F_LinearForce : Feature {
  void phi2(arr& y, arr& J, const FrameL& F);
  uint dim_phi2(const FrameL& F) { return 3; }
};

struct F_Wrench2 : Feature {
  void phi2(arr& y, arr& J, const FrameL& F);
  uint dim_phi2(const FrameL& F){ return 6; }
};

struct F_HingeXTorque : Feature {
  void phi2(arr& y, arr& J, const FrameL& F);
  uint dim_phi2(const FrameL& F){ return 1; }
};

struct TM_Contact_ForceIsNormal : Feature {
  void phi2(arr& y, arr& J, const FrameL& F);
  uint dim_phi2(const FrameL& F) { return 3; }
};

struct TM_Contact_ForceIsComplementary : Feature {
  void phi2(arr& y, arr& J, const FrameL& F);
  uint dim_phi2(const FrameL& F);
};

struct TM_Contact_ForceIsPositive : Feature {
  void phi2(arr& y, arr& J, const FrameL& F);
  uint dim_phi2(const FrameL& F) { return 1; }
};

struct TM_Contact_POAmovesContinuously : Feature {
  void phi2(arr& y, arr& J, const FrameL& F);
  uint dim_phi2(const FrameL& F) { return 3; }
};

struct TM_Contact_NormalForceEqualsNormalPOAmotion: Feature {
  void phi2(arr& y, arr& J, const FrameL& F);
  uint dim_phi2(const FrameL& F) { return 1; }
};

struct TM_ContactConstraints_Vel : Feature {
  TM_ContactConstraints_Vel(){ order=1; }
  void phi2(arr& y, arr& J, const FrameL& F);
  uint dim_phi2(const FrameL& F);
};

struct TM_Contact_POAzeroRelVel : Feature {
  bool normalOnly=false;
  TM_Contact_POAzeroRelVel(bool _normalOnly=false) : normalOnly(_normalOnly) { order=1; }
  void phi2(arr& y, arr& J, const FrameL& F);
  uint dim_phi2(const FrameL& F) { if(normalOnly) return 1; return 3; }
};

struct TM_Contact_ElasticVel : Feature {
  double elasticity, stickiness;
  TM_Contact_ElasticVel(double _elasticity, double _stickiness) : elasticity(_elasticity), stickiness(_stickiness) { order=1; }
  void phi2(arr& y, arr& J, const FrameL& F);
  uint dim_phi2(const FrameL& F) { return 4; }
};

struct TM_Contact_NormalVelIsComplementary : Feature {
  double elasticity, stickiness;
  TM_Contact_NormalVelIsComplementary(double _elasticity, double _stickiness) : elasticity(_elasticity), stickiness(_stickiness) { order=1; }
  void phi2(arr& y, arr& J, const FrameL& F);
  uint dim_phi2(const FrameL& F) { return 1; }
};

struct TM_Contact_POAisInIntersection_InEq : Feature {
  double margin=0.;
  TM_Contact_POAisInIntersection_InEq(double _margin=0.) : margin(_margin) {}
  void phi2(arr& y, arr& J, const FrameL& F);
  uint dim_phi2(const FrameL& F) { return 2; }
};

struct TM_Contact_POA_isAtWitnesspoint : Feature {
  bool use2ndObject=false;
  TM_Contact_POA_isAtWitnesspoint(bool _use2ndObject=false) : use2ndObject(_use2ndObject) {}
  void phi2(arr& y, arr& J, const FrameL& F);
  uint dim_phi2(const FrameL& F) { return 3; }
};
