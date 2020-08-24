/*  ------------------------------------------------------------------
    Copyright (c) 2011-2020 Marc Toussaint
    email: toussaint@tu-berlin.de

    This code is distributed under the MIT License.
    Please see <root-path>/LICENSE for details.
    --------------------------------------------------------------  */

#ifndef _MT_taskMap_default_h
#define _MT_taskMap_default_h

#include "motion.h"

enum TM_DefaultType {
  TMT_none,     ///< undefined
  TMT_pos,      ///< 3D position of reference
  TMT_vec,      ///< 3D vec (orientation)
  TMT_quat,     ///< 4D quaterion
  TMT_vecAlign, ///< 1D vector alignment, can have 2nd reference, param (optional) determins alternative reference world vector
  TMT_qItself,  ///< q itself as task variable, no param
  TMT_qLinear,  ///< k-dim variable linear in q, no references, param: k-times-n matrix
  TMT_qSingle,  ///< 1D entry of q, reference-integer=index, no param
  TMT_qSquared, ///< 1D square norm of q, no references, param: n-times-n matrix
  TMT_qLimits,  ///< 1D meassure for joint limit violation, no references, param: n-times-2 matrix with lower and upper limits for each joint
  TMT_coll,     ///< 1D meassure for collision violation, no references, param: 1D number defining the distance margin
  TMT_colCon,   ///< 1D meassure collision CONSTRAINT meassure, no references, param: 1D number defining the distance margin
  TMT_com,      ///< 2D vector of the horizontal center of mass, no refs, no param
  TMT_skin      ///< vector of skin pressures...
};

struct TM_Default:Feature {
  TM_DefaultType type;
  int i, j;               ///< which shapes does it refer to?
  rai::Vector ivec, jvec; ///< additional position or vector
  arr params;             ///< parameters of the variable (e.g., liner coefficients, limits, etc)

  TM_Default(TM_DefaultType type,
             int iShape=-1, const rai::Vector& ivec=NoVector,
             int jShape=-1, const rai::Vector& jvec=NoVector,
             const arr& params=NoArr);

  TM_Default(TM_DefaultType type, const rai::Configuration& G,
             const char* iShapeName=nullptr, const rai::Vector& ivec=NoVector,
             const char* jShapeName=nullptr, const rai::Vector& jvec=NoVector,
             const arr& params=NoArr);

  virtual void phi(arr& y, arr& J, const rai::Configuration& G);
  virtual uint dim_phi(const rai::Configuration& G);
};

#endif
