/*  ---------------------------------------------------------------------
    Copyright 2014 Marc Toussaint
    email: marc.toussaint@informatik.uni-stuttgart.de
    
    This program is free software: you can redistribute it and/or modify
    it under the terms of the GNU General Public License as published by
    the Free Software Foundation, either version 3 of the License, or
    (at your option) any later version.
    
    This program is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
    GNU General Public License for more details.
    
    You should have received a COPYING file of the GNU General Public License
    along with this program. If not, see <http://www.gnu.org/licenses/>
    -----------------------------------------------------------------  */
#ifndef _MT_taskMap_default_h
#define _MT_taskMap_default_h

#include "motion.h"

enum TaskMap_DefaultType {
  noneTMT,     ///< undefined
  posTMT,      ///< 3D position of reference
  vecTMT,      ///< 3D vec (orientation)
  quatTMT,     ///< 4D quaterion
  vecAlignTMT, ///< 1D vector alignment, can have 2nd reference, param (optional) determins alternative reference world vector
  qItselfTMT,  ///< q itself as task variable, no param
  qLinearTMT,  ///< k-dim variable linear in q, no references, param: k-times-n matrix
  qSingleTMT,  ///< 1D entry of q, reference-integer=index, no param
  qSquaredTMT, ///< 1D square norm of q, no references, param: n-times-n matrix
  qLimitsTMT,  ///< 1D meassure for joint limit violation, no references, param: n-times-2 matrix with lower and upper limits for each joint
  collTMT,     ///< 1D meassure for collision violation, no references, param: 1D number defining the distance margin
  colConTMT,   ///< 1D meassure collision CONSTRAINT meassure, no references, param: 1D number defining the distance margin
  comTMT,      ///< 2D vector of the horizontal center of mass, no refs, no param
  skinTMT      ///< vector of skin pressures...
};



struct TaskMap_Default:TaskMap {
  TaskMap_DefaultType type;
  int i, j;               ///< which shapes does it refer to?
  mlr::Vector ivec, jvec; ///< additional position or vector
  arr params;             ///< parameters of the variable (e.g., liner coefficients, limits, etc)

  TaskMap_Default(TaskMap_DefaultType type,
                 int iShape=-1, const mlr::Vector& ivec=NoVector,
                 int jShape=-1, const mlr::Vector& jvec=NoVector,
                 const arr& params=NoArr);

  TaskMap_Default(TaskMap_DefaultType type, const mlr::KinematicWorld& G,
                 const char* iShapeName=NULL, const mlr::Vector& ivec=NoVector,
                 const char* jShapeName=NULL, const mlr::Vector& jvec=NoVector,
                 const arr& params=NoArr);

  virtual void phi(arr& y, arr& J, const mlr::KinematicWorld& G);
  virtual uint dim_phi(const mlr::KinematicWorld& G);
};

#endif
