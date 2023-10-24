/*  ------------------------------------------------------------------
    Copyright (c) 2011-2020 Marc Toussaint
    email: toussaint@tu-berlin.de

    This code is distributed under the MIT License.
    Please see <root-path>/LICENSE for details.
    --------------------------------------------------------------  */

#pragma once

#include "feature.h"

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
  TMT_pose,
  TMT_poseDiff,
  pos1TMT_D,
};

struct TM_Default : Feature {
  rai::Enum<TM_DefaultType> type;   ///< joint type

  int i, j;               ///< which shapes does it refer to?
  rai::Vector ivec, jvec; ///< additional position or vector

  TM_Default(TM_DefaultType type,
             int iShape=-1, const rai::Vector& ivec=NoVector,
             int jShape=-1, const rai::Vector& jvec=NoVector);

  TM_Default(TM_DefaultType type, const rai::Configuration& K,
             const char* iShapeName=nullptr, const rai::Vector& ivec=NoVector,
             const char* jShapeName=nullptr, const rai::Vector& jvec=NoVector);

  TM_Default(const rai::Graph& parameters, const rai::Configuration& G);
  TM_Default(const rai::Node* parameters, const rai::Configuration& G);

  virtual void phi(arr& y, arr& J, const rai::Configuration& C);
  virtual uint dim_phi(const rai::Configuration& C);
  virtual void signature(intA& S, const rai::Configuration& C);
  virtual rai::String shortTag(const rai::Configuration& C);
  virtual rai::Graph getSpec(const rai::Configuration& C);
};

