/*  ------------------------------------------------------------------
    Copyright (c) 2011-2020 Marc Toussaint
    email: toussaint@tu-berlin.de

    This code is distributed under the MIT License.
    Please see <root-path>/LICENSE for details.
    --------------------------------------------------------------  */

#pragma once

#include "feature.h"

#include "F_qFeatures.h"
#include "TM_GJK.h"
#include "F_PairCollision.h"
#include "TM_default.h"
#include "TM_pushConsistent.h"
#include "TM_FixSwitchedObjects.h"
#include "TM_AboveBox.h"
#include "TM_AlignStacking.h"
#include "TM_linTrans.h"
#include "F_collisions.h"
#include "TM_Align.h"

//===========================================================================

struct CollisionConstraint:Feature {
  double margin;
  CollisionConstraint(double _margin=.1):margin(_margin) {}
  virtual void phi(arr& y, arr& J, const rai::Configuration& G);
  virtual uint dim_phi(const rai::Configuration& G) { return 1; }
  virtual rai::String shortTag(const rai::Configuration& G) { return STRING("CollisionConstraint"); }
};

//===========================================================================

struct PairCollisionConstraint:Feature {
  int i, j;      ///< which shapes does it refer to?
  double margin;

  PairCollisionConstraint(double _margin)
    : i(-1), j(-1), margin(_margin) {
  }
  PairCollisionConstraint(const rai::Configuration& G, const char* iShapeName, const char* jShapeName, double _margin=.02);

  virtual void phi(arr& y, arr& J, const rai::Configuration& G);
  virtual uint dim_phi(const rai::Configuration& G) { return 1; }
};

//===========================================================================

struct PlaneConstraint:Feature {
  int i;       ///< which shapes does it refer to?
  arr planeParams;  ///< parameters of the variable (e.g., liner coefficients, limits, etc)

  PlaneConstraint(const rai::Configuration& G, const char* iShapeName, const arr& _planeParams);

  virtual void phi(arr& y, arr& J, const rai::Configuration& G);
  virtual uint dim_phi(const rai::Configuration& G) { return 1; }
};

//===========================================================================

//this is NOT a constraint -- it turns a constraint into stickiness
struct ConstraintStickiness:Feature {
  Feature& map;
  ConstraintStickiness(Feature& _map)
    : map(_map) {
  }

  virtual void phi(arr& y, arr& J, const rai::Configuration& G);
  virtual uint dim_phi(const rai::Configuration& G) { return 1; }
};

//===========================================================================

struct PointEqualityConstraint:Feature {
  int i, j;               ///< which shapes does it refer to?
  rai::Vector ivec, jvec; ///< additional position or vector

  PointEqualityConstraint(const rai::Configuration& G,
                          const char* iShapeName=nullptr, const rai::Vector& _ivec=NoVector,
                          const char* jShapeName=nullptr, const rai::Vector& _jvec=NoVector) {
    TM_Default dummy(TMT_pos, G, iShapeName, _ivec, jShapeName, _jvec); //is deleted in a sec..
    i=dummy.i;
    j=dummy.j;
    ivec=dummy.ivec;
    jvec=dummy.jvec;
  }

  virtual void phi(arr& y, arr& J, const rai::Configuration& G);
  virtual uint dim_phi(const rai::Configuration& G) { return 3; }
};

//===========================================================================

struct ContactEqualityConstraint:Feature {
  int i;       ///< which shapes does it refer to?
  int j;       ///< which shapes does it refer to?
  double margin;
  ContactEqualityConstraint(const rai::Configuration& G, const char* iShapeName, const char* jShapeName, double _margin);
  virtual void phi(arr& y, arr& J, const rai::Configuration& G);
  virtual uint dim_phi(const rai::Configuration& G) {
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
  VelAlignConstraint(const rai::Configuration& G,
                     const char* iShapeName=nullptr, const rai::Vector& _ivec=NoVector,
                     const char* jShapeName=nullptr, const rai::Vector& _jvec=NoVector, double _target = 0.);

  virtual void phi(arr& y, arr& J, const rai::Configuration& G, int t=1) { } ;
  virtual void phi(arr& y, arr& J, const ConfigurationL& G);
  virtual uint dim_phi(const rai::Configuration& G) { return 1; }
};

//===========================================================================

struct qItselfConstraint:Feature {
  arr M;

  qItselfConstraint(uint singleQ, uint qN) { M=zeros(1, qN); M(0, singleQ)=1.; }
  qItselfConstraint(const arr& _M=NoArr) { if(!!_M) M=_M; }

  virtual void phi(arr& y, arr& J, const rai::Configuration& G);
  virtual uint dim_phi(const rai::Configuration& G) {
    if(M.nd==2) return M.d0;
    return G.getJointStateDimension();
  }
};

//===========================================================================

