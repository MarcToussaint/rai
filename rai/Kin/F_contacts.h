/*  ------------------------------------------------------------------
    Copyright (c) 2019 Marc Toussaint
    email: marc.toussaint@informatik.uni-stuttgart.de

    This code is distributed under the MIT License.
    Please see <root-path>/LICENSE for details.
    --------------------------------------------------------------  */

#pragma once

#include "feature.h"

struct TM_Contact_POA : Feature {
  int a, b;
  TM_Contact_POA(int aShape, int bShape) : a(aShape), b(bShape) {}
  TM_Contact_POA(const rai::Configuration& K, const char* aShapeName=nullptr, const char* bShapeName=nullptr)
    : TM_Contact_POA(initIdArg(K, aShapeName), initIdArg(K, bShapeName)) {}

  void phi(arr& y, arr& J, const rai::Configuration& C);
  uint dim_phi(const rai::Configuration& K) { return 3; }
  rai::String shortTag(const rai::Configuration& K) { return STRING("TM_Contact_POA-" <<K.frames(a)->name <<'-' <<K.frames(b)->name); }
};

struct TM_Contact_Force : Feature {
  int a, b;
  TM_Contact_Force(int aShape, int bShape) : a(aShape), b(bShape) {}
  TM_Contact_Force(const rai::Configuration& K, const char* aShapeName=nullptr, const char* bShapeName=nullptr)
    : TM_Contact_Force(initIdArg(K, aShapeName), initIdArg(K, bShapeName)) {}

  void phi(arr& y, arr& J, const rai::Configuration& C);
  uint dim_phi(const rai::Configuration& K) { return 3; }
  rai::String shortTag(const rai::Configuration& K) { return STRING("TM_Contact_Force-" <<K.frames(a)->name <<'-' <<K.frames(b)->name); }
};

struct TM_Contact_ForceIsNormal : Feature {
  int a, b;
  TM_Contact_ForceIsNormal(int aShape, int bShape) : a(aShape), b(bShape) {}
  TM_Contact_ForceIsNormal(const rai::Configuration& K, const char* aShapeName=nullptr, const char* bShapeName=nullptr)
    : TM_Contact_ForceIsNormal(initIdArg(K, aShapeName), initIdArg(K, bShapeName)) {}

  void phi(arr& y, arr& J, const rai::Configuration& K);
  uint dim_phi(const rai::Configuration& K) { return 3; }
  rai::String shortTag(const rai::Configuration& K) { return STRING("TM_Contact_ForceIsNormal-" <<K.frames(a)->name <<'-' <<K.frames(b)->name); }
};

struct TM_Contact_ForceIsComplementary : Feature {
  int a, b;
  TM_Contact_ForceIsComplementary(int aShape, int bShape) : a(aShape), b(bShape) {}
  TM_Contact_ForceIsComplementary(const rai::Configuration& K, const char* aShapeName=nullptr, const char* bShapeName=nullptr)
    : TM_Contact_ForceIsComplementary(initIdArg(K, aShapeName), initIdArg(K, bShapeName)) {}

  void phi(arr& y, arr& J, const rai::Configuration& K);
  uint dim_phi(const rai::Configuration& K);
  rai::String shortTag(const rai::Configuration& K) { return STRING("TM_Contact_ForceIsComplementary-" <<K.frames(a)->name <<'-' <<K.frames(b)->name); }
};

struct TM_Contact_ForceIsPositive : Feature {
  int a, b;
  TM_Contact_ForceIsPositive(int aShape, int bShape) : a(aShape), b(bShape) {}
  TM_Contact_ForceIsPositive(const rai::Configuration& K, const char* aShapeName=nullptr, const char* bShapeName=nullptr)
    : TM_Contact_ForceIsPositive(initIdArg(K, aShapeName), initIdArg(K, bShapeName)) {}

  void phi(arr& y, arr& J, const rai::Configuration& K);
  uint dim_phi(const rai::Configuration& K) { return 1; }
  rai::String shortTag(const rai::Configuration& K) { return STRING("TM_Contact_ForceIsPositive-" <<K.frames(a)->name <<'-' <<K.frames(b)->name); }
};

struct TM_Contact_POAmovesContinuously : Feature {
  int a, b;
  TM_Contact_POAmovesContinuously(int aShape, int bShape) : a(aShape), b(bShape) {}
  TM_Contact_POAmovesContinuously(const rai::Configuration& K, const char* aShapeName=nullptr, const char* bShapeName=nullptr)
    : TM_Contact_POAmovesContinuously(initIdArg(K, aShapeName), initIdArg(K, bShapeName)) {}

  void phi(arr& y, arr& J, const rai::Configuration& K) { NIY }
  void phi(arr& y, arr& J, const ConfigurationL& Ktuple);
  uint dim_phi(const rai::Configuration& K) { return 3; }
  rai::String shortTag(const rai::Configuration& K) { return STRING("TM_Contact_POAmovesContinuously-" <<K.frames(a)->name <<'-' <<K.frames(b)->name); }
};

struct TM_Contact_NormalForceEqualsNormalPOAmotion: Feature {
  int a,b;
  TM_Contact_NormalForceEqualsNormalPOAmotion(int aShape, int bShape) : a(aShape), b(bShape) { order = 1; }
  TM_Contact_NormalForceEqualsNormalPOAmotion(const rai::Configuration& K, const char* aShapeName=NULL, const char* bShapeName=NULL)
    : TM_Contact_NormalForceEqualsNormalPOAmotion(initIdArg(K,aShapeName), initIdArg(K,bShapeName)){}

  void phi(arr& y, arr& J, const rai::Configuration& K){ NIY }
  void phi(arr& y, arr& J, const ConfigurationL& Ktuple);
  uint dim_phi(const rai::Configuration& K){ return 1; }
  rai::String shortTag(const rai::Configuration& K) { return STRING("TM_Contact_NormalForceEqualsNormalPOAmotion-" <<K.frames(a)->name <<'-' <<K.frames(b)->name); }
};

struct TM_ContactConstraints_Vel : Feature {
  int a, b;
  TM_ContactConstraints_Vel(int aShape, int bShape) : a(aShape), b(bShape) { order=1; }
  TM_ContactConstraints_Vel(const rai::Configuration& K, const char* aShapeName=nullptr, const char* bShapeName=nullptr)
    : TM_ContactConstraints_Vel(initIdArg(K, aShapeName), initIdArg(K, bShapeName)) {}

  void phi(arr& y, arr& J, const rai::Configuration& K) { NIY }
  void phi(arr& y, arr& J, const ConfigurationL& Ktuple);
  uint dim_phi(const rai::Configuration& K);
  rai::String shortTag(const rai::Configuration& K) { return STRING("TM_ContactConstraints_Vel-" <<K.frames(a)->name <<'-' <<K.frames(b)->name); }
};

struct TM_Contact_POAzeroRelVel : Feature {
  int a, b;
  bool normalOnly=false;
  TM_Contact_POAzeroRelVel(int aShape, int bShape, bool _normalOnly=false) : a(aShape), b(bShape), normalOnly(_normalOnly) { order=1; }
  TM_Contact_POAzeroRelVel(const rai::Configuration& K, const char* aShapeName=nullptr, const char* bShapeName=nullptr, bool _normalOnly=false)
    : TM_Contact_POAzeroRelVel(initIdArg(K, aShapeName), initIdArg(K, bShapeName),  _normalOnly){}

  void phi(arr& y, arr& J, const rai::Configuration& K) { NIY }
  void phi(arr& y, arr& J, const ConfigurationL& Ktuple);
  uint dim_phi(const rai::Configuration& K) { if(normalOnly) return 1; return 3; }
  rai::String shortTag(const rai::Configuration& K) { return STRING("TM_Contact_POAzeroRelVel-" <<K.frames(a)->name <<'-' <<K.frames(b)->name); }
};

struct TM_Contact_ElasticVel : Feature {
  int a, b;
  double elasticity, stickiness;
  TM_Contact_ElasticVel(int aShape, int bShape, double _elasticity, double _stickiness) : a(aShape), b(bShape), elasticity(_elasticity), stickiness(_stickiness) { order=1; }
  TM_Contact_ElasticVel(const rai::Configuration& K, const char* aShapeName, const char* bShapeName, double _elasticity, double _stickiness)
    : TM_Contact_ElasticVel(initIdArg(K, aShapeName), initIdArg(K, bShapeName), _elasticity, _stickiness) {}

  void phi(arr& y, arr& J, const rai::Configuration& K) { NIY }
  void phi(arr& y, arr& J, const ConfigurationL& Ktuple);
  uint dim_phi(const rai::Configuration& K) { return 4; }
  rai::String shortTag(const rai::Configuration& K) { return STRING("TM_Contact_ElasticVel-" <<K.frames(a)->name <<'-' <<K.frames(b)->name); }
};

struct TM_Contact_NormalVelIsComplementary : Feature {
  int a, b;
  double elasticity, stickiness;
  TM_Contact_NormalVelIsComplementary(int aShape, int bShape, double _elasticity, double _stickiness) : a(aShape), b(bShape), elasticity(_elasticity), stickiness(_stickiness) { order=1; }
  TM_Contact_NormalVelIsComplementary(const rai::Configuration& K, const char* aShapeName, const char* bShapeName, double _elasticity, double _stickiness)
    : TM_Contact_NormalVelIsComplementary(initIdArg(K, aShapeName), initIdArg(K, bShapeName), _elasticity, _stickiness) {}

  void phi(arr& y, arr& J, const rai::Configuration& K) { NIY }
  void phi(arr& y, arr& J, const ConfigurationL& Ktuple);
  uint dim_phi(const rai::Configuration& K) { return 1; }
  rai::String shortTag(const rai::Configuration& K) { return STRING("TM_Contact_ElasticVelIsComplementary-" <<K.frames(a)->name <<'-' <<K.frames(b)->name); }
};

struct TM_Contact_POAisInIntersection_InEq : Feature {
  int a, b;
  double margin=0.;
  TM_Contact_POAisInIntersection_InEq(int aShape, int bShape, double _margin=0.) : a(aShape), b(bShape), margin(_margin) {}
  TM_Contact_POAisInIntersection_InEq(const rai::Configuration& K, const char* aShapeName=nullptr, const char* bShapeName=nullptr, double _margin=0.)
    : TM_Contact_POAisInIntersection_InEq(initIdArg(K, aShapeName), initIdArg(K, bShapeName), _margin) {}

  void phi(arr& y, arr& J, const rai::Configuration& K);
  uint dim_phi(const rai::Configuration& K) { return 2; }
  rai::String shortTag(const rai::Configuration& K) { return STRING("TM_Contact_POAisInIntersection_InEq-" <<K.frames(a)->name <<'-' <<K.frames(b)->name); }
};

struct TM_Contact_POA_isAtWitnesspoint : Feature {
  int a,b;
  bool use2ndObject=false;
  TM_Contact_POA_isAtWitnesspoint(int aShape, int bShape, bool _use2ndObject=false) : a(aShape), b(bShape), use2ndObject(_use2ndObject) {}
  TM_Contact_POA_isAtWitnesspoint(const rai::Configuration& K, const char* aShapeName=NULL, const char* bShapeName=NULL, bool _use2ndObject=false)
    : TM_Contact_POA_isAtWitnesspoint(initIdArg(K,aShapeName), initIdArg(K,bShapeName), _use2ndObject) {}

  void phi(arr& y, arr& J, const rai::Configuration& C);
  uint dim_phi(const rai::Configuration& K){ return 3; }
  rai::String shortTag(const rai::Configuration& K) { return STRING("TM_Contact_POA_isAtWitnesspoint-" <<K.frames(a)->name <<'-' <<K.frames(b)->name); }
};
