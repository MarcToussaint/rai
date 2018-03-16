/*  ------------------------------------------------------------------
    Copyright (c) 2017 Marc Toussaint
    email: marc.toussaint@informatik.uni-stuttgart.de
    
    This code is distributed under the MIT License.
    Please see <root-path>/LICENSE for details.
    --------------------------------------------------------------  */

#pragma once
#include "taskMap.h"

//===========================================================================

enum TM_DefaultType {
  TMT_no=0,    ///< non-initialization
  TMT_pos,     ///< 3D position of reference
  TMT_vec,     ///< 3D vec (orientation)
  TMT_quat,    ///< 4D quaterion
  TMT_posDiff, ///< the difference of two positions (NOT the relative position)
  TMT_vecDiff, ///< the difference of two vectors (NOT the relative position)
  TMT_quatDiff,///< the difference of 2 quaternions (NOT the relative quaternion)
  TMT_vecAlign,///< 1D vector alignment, can have 2nd reference, param (optional) determins alternative reference world vector
  TMT_gazeAt,  ///< 2D orthogonality measure of object relative to camera plane
  pos1TMT_D,
  TMT_poseDiff
};
extern const char* TM_DefaultType2String[];

struct TM_Default:TaskMap {
  TM_DefaultType type;
  int i, j;               ///< which shapes does it refer to?
  mlr::Vector ivec, jvec; ///< additional position or vector
  intA referenceIds; ///< the shapes it refers to DEPENDENT on time

  TM_Default(TM_DefaultType type,
                 int iShape=-1, const mlr::Vector& ivec=NoVector,
                 int jShape=-1, const mlr::Vector& jvec=NoVector);

  TM_Default(TM_DefaultType type, const mlr::KinematicWorld& K,
                 const char* iShapeName=NULL, const mlr::Vector& ivec=NoVector,
                 const char* jShapeName=NULL, const mlr::Vector& jvec=NoVector);

  TM_Default(const Graph &parameters, const mlr::KinematicWorld& G);
  TM_Default(const Node *parameters, const mlr::KinematicWorld& G);

  virtual void phi(arr& y, arr& J, const mlr::KinematicWorld& G, int t=-1);
  virtual uint dim_phi(const mlr::KinematicWorld& G);
  virtual mlr::String shortTag(const mlr::KinematicWorld& G);
};

