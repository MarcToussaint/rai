/*  ------------------------------------------------------------------
    Copyright (c) 2017 Marc Toussaint
    email: marc.toussaint@informatik.uni-stuttgart.de

    This code is distributed under the MIT License.
    Please see <root-path>/LICENSE for details.
    --------------------------------------------------------------  */

#pragma once

#include "feature.h"

#include "TM_qItself.h"
#include "TM_GJK.h"
#include "TM_PairCollision.h"
#include "TM_transition.h"
#include "TM_default.h"
#include "TM_qLimits.h"
#include "TM_pushConsistent.h"
#include "TM_FixSwitchedObjects.h"
#include "TM_AboveBox.h"
#include "TM_AlignStacking.h"
#include "TM_linTrans.h"
#include "TM_proxy.h"
#include "TM_Align.h"

//===========================================================================

struct CollisionConstraint:Feature {
  double margin;
  CollisionConstraint(double _margin=.1):margin(_margin) {}
  virtual void phi(arr& y, arr& J, const rai::KinematicWorld& G);
  virtual uint dim_phi(const rai::KinematicWorld& G) { return 1; }
  virtual rai::String shortTag(const rai::KinematicWorld& G) { return STRING("CollisionConstraint"); }
};

//===========================================================================

struct PairCollisionConstraint:Feature {
  int i,j;       ///< which shapes does it refer to?
  double margin;
  
  PairCollisionConstraint(double _margin)
    : i(-1), j(-1), margin(_margin) {
  }
  PairCollisionConstraint(const rai::KinematicWorld& G, const char* iShapeName, const char* jShapeName, double _margin=.02);
  
  virtual void phi(arr& y, arr& J, const rai::KinematicWorld& G);
  virtual uint dim_phi(const rai::KinematicWorld& G) { return 1; }
};

//===========================================================================

struct PlaneConstraint:Feature {
  int i;       ///< which shapes does it refer to?
  arr planeParams;  ///< parameters of the variable (e.g., liner coefficients, limits, etc)
  
  PlaneConstraint(const rai::KinematicWorld& G, const char* iShapeName, const arr& _planeParams);
  
  virtual void phi(arr& y, arr& J, const rai::KinematicWorld& G);
  virtual uint dim_phi(const rai::KinematicWorld& G) { return 1; }
};

//===========================================================================

//this is NOT a constraint -- it turns a constraint into stickiness
struct ConstraintStickiness:Feature {
  Feature& map;
  ConstraintStickiness(Feature& _map)
    : map(_map) {
  }
  
  virtual void phi(arr& y, arr& J, const rai::KinematicWorld& G);
  virtual uint dim_phi(const rai::KinematicWorld& G) { return 1; }
};

//===========================================================================

struct PointEqualityConstraint:Feature {
  int i, j;               ///< which shapes does it refer to?
  rai::Vector ivec, jvec; ///< additional position or vector
  
  PointEqualityConstraint(const rai::KinematicWorld &G,
                          const char* iShapeName=NULL, const rai::Vector& _ivec=NoVector,
                          const char* jShapeName=NULL, const rai::Vector& _jvec=NoVector) {
    TM_Default dummy(TMT_pos, G, iShapeName, _ivec, jShapeName, _jvec); //is deleted in a sec..
    i=dummy.i;
    j=dummy.j;
    ivec=dummy.ivec;
    jvec=dummy.jvec;
  }
  
  virtual void phi(arr& y, arr& J, const rai::KinematicWorld& G);
  virtual uint dim_phi(const rai::KinematicWorld& G) { return 3; }
};

//===========================================================================

struct ContactEqualityConstraint:Feature {
  int i;       ///< which shapes does it refer to?
  int j;       ///< which shapes does it refer to?
  double margin;
  ContactEqualityConstraint(const rai::KinematicWorld& G, const char* iShapeName, const char* jShapeName,double _margin);
  virtual void phi(arr& y, arr& J, const rai::KinematicWorld& G);
  virtual uint dim_phi(const rai::KinematicWorld& G) {
    return 1;
  }
};

//===========================================================================

struct VelAlignConstraint:Feature {
  int i;       ///< which shapes does it refer to?
  int j;       ///< which shapes does it refer to?
  rai::Vector ivec, jvec; ///< additional position or vector
  double target;
  
  double margin;
  VelAlignConstraint(const rai::KinematicWorld& G,
                     const char* iShapeName=NULL, const rai::Vector& _ivec=NoVector,
                     const char* jShapeName=NULL, const rai::Vector& _jvec=NoVector, double _target = 0.);
                     
  virtual void phi(arr& y, arr& J, const rai::KinematicWorld& G, int t=1) { } ;
  virtual void phi(arr& y, arr& J, const WorldL& G);
  virtual uint dim_phi(const rai::KinematicWorld& G) { return 1; }
};

//===========================================================================

struct qItselfConstraint:Feature {
  arr M;
  
  qItselfConstraint(uint singleQ, uint qN) { M=zeros(1,qN); M(0,singleQ)=1.; }
  qItselfConstraint(const arr& _M=NoArr) { if(!!_M) M=_M; }
  
  virtual void phi(arr& y, arr& J, const rai::KinematicWorld& G);
  virtual uint dim_phi(const rai::KinematicWorld& G) {
    if(M.nd==2) return M.d0;
    return G.getJointStateDimension();
  }
};

//===========================================================================

