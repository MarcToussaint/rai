/*  ------------------------------------------------------------------
    Copyright (c) 2017 Marc Toussaint
    email: marc.toussaint@informatik.uni-stuttgart.de
    
    This code is distributed under the MIT License.
    Please see <root-path>/LICENSE for details.
    --------------------------------------------------------------  */

#pragma once

#include "taskMap.h"

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

struct CollisionConstraint:TaskMap {
  double margin;
  CollisionConstraint(double _margin=.1):margin(_margin){}
  virtual void phi(arr& y, arr& J, const mlr::KinematicWorld& G, int t=-1);
  virtual uint dim_phi(const mlr::KinematicWorld& G){ return 1; }
  virtual mlr::String shortTag(const mlr::KinematicWorld& G){ return STRING("CollisionConstraint"); }
};



//===========================================================================

struct PairCollisionConstraint:TaskMap {
  int i,j;       ///< which shapes does it refer to?
  double margin;
  intA referenceIds; ///< the shapes it refers to DEPENDENT on time
  PairCollisionConstraint(double _margin)
    : i(-1), j(-1), margin(_margin){
  }
  PairCollisionConstraint(const mlr::KinematicWorld& G, const char* iShapeName, const char* jShapeName, double _margin=.02);

  virtual void phi(arr& y, arr& J, const mlr::KinematicWorld& G, int t=-1);
  virtual uint dim_phi(const mlr::KinematicWorld& G){ return 1; }
};

//===========================================================================

struct PlaneConstraint:TaskMap {
  int i;       ///< which shapes does it refer to?
  arr planeParams;  ///< parameters of the variable (e.g., liner coefficients, limits, etc)

  PlaneConstraint(const mlr::KinematicWorld& G, const char* iShapeName, const arr& _planeParams);

  virtual void phi(arr& y, arr& J, const mlr::KinematicWorld& G, int t=1);
  virtual uint dim_phi(const mlr::KinematicWorld& G){ return 1; }
};

//===========================================================================

//this is NOT a constraint -- it turns a constraint into stickiness
struct ConstraintStickiness:TaskMap {
  TaskMap& map;
  ConstraintStickiness(TaskMap& _map)
    : map(_map) {
  }

  virtual void phi(arr& y, arr& J, const mlr::KinematicWorld& G, int t=-1);
  virtual uint dim_phi(const mlr::KinematicWorld& G){ return 1; }
};

//===========================================================================

struct PointEqualityConstraint:TaskMap {
  int i, j;               ///< which shapes does it refer to?
  mlr::Vector ivec, jvec; ///< additional position or vector

  PointEqualityConstraint(const mlr::KinematicWorld &G,
                          const char* iShapeName=NULL, const mlr::Vector& _ivec=NoVector,
                          const char* jShapeName=NULL, const mlr::Vector& _jvec=NoVector){
    TM_Default dummy(TMT_pos, G, iShapeName, _ivec, jShapeName, _jvec); //is deleted in a sec..
    i=dummy.i;
    j=dummy.j;
    ivec=dummy.ivec;
    jvec=dummy.jvec;
  }

  virtual void phi(arr& y, arr& J, const mlr::KinematicWorld& G, int t=-1);
  virtual uint dim_phi(const mlr::KinematicWorld& G){ return 3; }
};

//===========================================================================

struct ContactEqualityConstraint:TaskMap {
  int i;       ///< which shapes does it refer to?
  int j;       ///< which shapes does it refer to?
  double margin;
  ContactEqualityConstraint(const mlr::KinematicWorld& G, const char* iShapeName, const char* jShapeName,double _margin);
  virtual void phi(arr& y, arr& J, const mlr::KinematicWorld& G, int t=1);
  virtual uint dim_phi(const mlr::KinematicWorld& G){
    return 1;
  }
};

//===========================================================================

struct VelAlignConstraint:TaskMap {
  int i;       ///< which shapes does it refer to?
  int j;       ///< which shapes does it refer to?
  mlr::Vector ivec, jvec; ///< additional position or vector
  double target;

  double margin;
  VelAlignConstraint(const mlr::KinematicWorld& G,
                     const char* iShapeName=NULL, const mlr::Vector& _ivec=NoVector,
                     const char* jShapeName=NULL, const mlr::Vector& _jvec=NoVector, double _target = 0.);

  virtual void phi(arr& y, arr& J, const mlr::KinematicWorld& G, int t=1) { } ;
  virtual void phi(arr& y, arr& J, const WorldL& G, double tau, int t=-1);
  virtual uint dim_phi(const mlr::KinematicWorld& G){ return 1; }
};

//===========================================================================

struct qItselfConstraint:TaskMap {
  arr M;

  qItselfConstraint(uint singleQ, uint qN){ M=zeros(1,qN); M(0,singleQ)=1.; }
  qItselfConstraint(const arr& _M=NoArr){ if(&_M) M=_M; }

  virtual void phi(arr& y, arr& J, const mlr::KinematicWorld& G, int t=1);
  virtual uint dim_phi(const mlr::KinematicWorld& G){
    if(M.nd==2) return M.d0;
    return G.getJointStateDimension();
  }
};

//===========================================================================

