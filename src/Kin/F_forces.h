/*  ------------------------------------------------------------------
    Copyright (c) 2011-2024 Marc Toussaint
    email: toussaint@tu-berlin.de

    This code is distributed under the MIT License.
    Please see <root-path>/LICENSE for details.
    --------------------------------------------------------------  */

#pragma once

#include "feature.h"

//===========================================================================
// trivial read out of forces

struct F_fex_POA : Feature {
  void phi2(arr& y, arr& J, const FrameL& F);
  uint dim_phi(const FrameL& F) { return 3; }
};

struct F_fex_Force : Feature {
  void phi2(arr& y, arr& J, const FrameL& F);
  uint dim_phi(const FrameL& F) { return 3; }
};

struct F_fex_Torque : Feature {
  void phi2(arr& y, arr& J, const FrameL& F);
  uint dim_phi(const FrameL& F) { return 3; }
};

struct F_fex_Wrench : Feature {
  void phi2(arr& y, arr& J, const FrameL& F);
  uint dim_phi(const FrameL& F) { return 6; }
};

//===========================================================================
// totals acting on a frame (joint or object)

struct F_HingeXTorque : Feature {
  void phi2(arr& y, arr& J, const FrameL& F);
  uint dim_phi(const FrameL& F) { return 1; }
};

struct F_TotalForce : Feature {
  double gravity=9.81;
  F_TotalForce(bool _zeroGravity=false);
  virtual void phi2(arr& y, arr& J, const FrameL& F);
  virtual uint dim_phi(const FrameL& C) { return 6; }
};

//===========================================================================
// dynamics

struct F_GravityAcceleration : Feature {
  double gravity=9.81;
  bool impulseInsteadOfAcceleration=false;
  F_GravityAcceleration() {
    gravity = rai::getParameter<double>("gravity", 9.81);
  }
  Feature& setImpulseInsteadOfAcceleration() { impulseInsteadOfAcceleration=true; return *this; }
  virtual void phi2(arr& y, arr& J, const FrameL& F);
  virtual uint dim_phi(const FrameL& F) { return 6; }
};

struct F_NewtonEuler : Feature {
  bool useGravity=true;
  F_NewtonEuler(bool _useGravity = true) : useGravity(_useGravity) { order = 2; }
  virtual void phi2(arr& y, arr& J, const FrameL& F);
  virtual uint dim_phi(const FrameL& F) { return 6; }
};

struct F_NewtonEuler_DampedVelocities : Feature {
  bool useGravity=false;
  F_NewtonEuler_DampedVelocities(bool _useGravity = false) : useGravity(_useGravity) {
    order = 1;
  }
  virtual void phi2(arr& y, arr& J, const FrameL& F);
  virtual uint dim_phi(const FrameL& F) { return 6; }
};

struct F_Energy : Feature {
  double gravity=9.81;
  F_Energy() {
    order=1;
    gravity = rai::getParameter<double>("gravity", 9.81);
  }
  virtual void phi2(arr& y, arr& J, const FrameL& F);
  virtual uint dim_phi(const FrameL& F) {  return 1;  }
};

//===========================================================================
// force geometry, complementarity, velocities

struct F_fex_POAAtFrame : Feature {
  arr phi(const FrameL& F);
  uint dim_phi(const FrameL& F) { return 3; }
};

struct F_fex_ForceInFrameCone : Feature {
  double mu;
  F_fex_ForceInFrameCone(double _mu=.8) : mu(_mu) {}
  arr phi(const FrameL& F);
  uint dim_phi(const FrameL& F) { return 1; }
};

//===========================================================================

struct F_fex_ForceIsNormal : Feature {
  arr phi(const FrameL& F);
  uint dim_phi(const FrameL& F) { return 3; }
};

struct F_fex_ForceInFrictionCone : Feature {
  double mu;
  F_fex_ForceInFrictionCone(double _mu=.5) : mu(_mu) {}
  arr phi(const FrameL& F);
  uint dim_phi(const FrameL& F) { return 1; }
};

struct F_fex_ForceIsComplementary : Feature {
  void phi2(arr& y, arr& J, const FrameL& F);
  uint dim_phi(const FrameL& F);
};

struct F_fex_ForceIsPositive : Feature {
  void phi2(arr& y, arr& J, const FrameL& F);
  uint dim_phi(const FrameL& F) { return 1; }
};

struct F_fex_NormalForceEqualsNormalPOAmotion: Feature {
  void phi2(arr& y, arr& J, const FrameL& F);
  uint dim_phi(const FrameL& F) { return 1; }
};

struct F_fex_POA_PositionRel : Feature {
  bool b_or_a;
  F_fex_POA_PositionRel(bool b_or_a) : b_or_a(b_or_a) { order=1; }
  void phi2(arr& y, arr& J, const FrameL& F);
  uint dim_phi(const FrameL& F) { return 3; }
};

struct F_fex_POAzeroRelVel : Feature {
  bool normalOnly=false;
  F_fex_POAzeroRelVel(bool _normalOnly=false) : normalOnly(_normalOnly) { order=1; }
  arr phi(const FrameL& F);
  uint dim_phi(const FrameL& F) { if(normalOnly) return 1; return 3; }
};

struct F_fex_ElasticVel : Feature {
  double elasticity, stickiness;
  F_fex_ElasticVel(double _elasticity, double _stickiness) : elasticity(_elasticity), stickiness(_stickiness) { order=1; }
  arr phi(const FrameL& F);
  uint dim_phi(const FrameL& F) { return 4; }
};

struct F_fex_NormalVelIsComplementary : Feature {
  double elasticity, stickiness;
  F_fex_NormalVelIsComplementary(double _elasticity, double _stickiness) : elasticity(_elasticity), stickiness(_stickiness) { order=1; }
  arr phi(const FrameL& F);
  uint dim_phi(const FrameL& F) { return 1; }
};

struct F_fex_POASurfaceDistance : Feature {
  rai::ArgWord leftRight;
  F_fex_POASurfaceDistance(rai::ArgWord leftRight) : leftRight(leftRight) {}
  void phi2(arr& y, arr& J, const FrameL& F);
  uint dim_phi(const FrameL& F) { return 1; }
};

struct F_fex_POASurfaceNormal : Feature {
  rai::ArgWord leftRight;
  F_fex_POASurfaceNormal(rai::ArgWord leftRight) : leftRight(leftRight) {}
  void phi2(arr& y, arr& J, const FrameL& F);
  uint dim_phi(const FrameL& F) { return 3; }
};

struct F_fex_POASurfaceNormalsOppose : Feature {
  void phi2(arr& y, arr& J, const FrameL& F);
  uint dim_phi(const FrameL& F) { return 3; }
};

struct F_fex_POASurfaceAvgNormal : Feature {
  void phi2(arr& y, arr& J, const FrameL& F);
  uint dim_phi(const FrameL& F) { return 3; }
};

struct F_fex_POAContactDistances : Feature {
  void phi2(arr& y, arr& J, const FrameL& F);
  uint dim_phi(const FrameL& F) { return 2; }
};

struct F_fex_POA_isAtWitnesspoint : Feature {
  bool use2ndObject=false;
  F_fex_POA_isAtWitnesspoint(bool _use2ndObject=false) : use2ndObject(_use2ndObject) {}
  void phi2(arr& y, arr& J, const FrameL& F);
  uint dim_phi(const FrameL& F) { return 3; }
};

//===========================================================================

struct F_PushRadiusPrior : Feature {
  double rad;
  F_PushRadiusPrior(double _rad) : rad(_rad) {}
  virtual arr phi(const FrameL& F);
  virtual uint dim_phi(const FrameL& F) { return 3; }
};

struct F_PushAligned : Feature {
  virtual arr phi(const FrameL& F);
  virtual uint dim_phi(const FrameL& F) { return 3; }
};

struct F_PushSide : Feature {
  virtual arr phi(const FrameL& F);
  virtual uint dim_phi(const FrameL& F) { return 1; }
};
