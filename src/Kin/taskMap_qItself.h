/*  ------------------------------------------------------------------
    Copyright 2016 Marc Toussaint
    email: marc.toussaint@informatik.uni-stuttgart.de
    
    This program is free software: you can redistribute it and/or modify
    it under the terms of the GNU General Public License as published by
    the Free Software Foundation, either version 3 of the License, or (at
    your option) any later version. This program is distributed without
    any warranty. See the GNU General Public License for more details.
    You should have received a COPYING file of the full GNU General Public
    License along with this program. If not, see
    <http://www.gnu.org/licenses/>
    --------------------------------------------------------------  */


#pragma once

#include "taskMap.h"

//===========================================================================

enum TaskMap_qItself_PickMode{ QIP_byJointNames, QIP_byBodyNames, QIP_byJointGroups };

struct TaskMap_qItself:TaskMap {
  uintA selectedBodies; ///< optionally, select only a subset of joints, indicated by the BODIES! indices (reason: body indices are stable across kinematic switches)
  bool moduloTwoPi; ///< if false, consider multiple turns of a joint as different q values (Default: true)
  bool relative_q0; ///< if true, absolute values are given relative to Joint::q0

//  TaskMap_qItself(uint singleQ, uint qN); ///< The singleQ parameter generates a matrix M that picks out a single q value
//  TaskMap_qItself(const mlr::KinematicWorld& G, mlr::Joint* j);
//  TaskMap_qItself(const mlr::KinematicWorld& G, const char* jointName);
//  TaskMap_qItself(const mlr::KinematicWorld& G, const char* jointName1, const char* jointName2);

  TaskMap_qItself(bool relative_q0=false);
  TaskMap_qItself(TaskMap_qItself_PickMode pickMode, const StringA& picks, const mlr::KinematicWorld& G, bool relative_q0=false);
  TaskMap_qItself(uintA _selectedBodies, bool relative_q0=false);

  virtual void phi(arr& y, arr& J, const mlr::KinematicWorld& G, int t=-1);
  virtual void phi(arr& y, arr& J, const WorldL& G, double tau, int t);
  virtual uint dim_phi(const mlr::KinematicWorld& G);
  virtual uint dim_phi(const WorldL& G, int t);
  virtual mlr::String shortTag(const mlr::KinematicWorld& G);
private:
  uintA dimPhi;
};

//===========================================================================

struct TaskMap_qZeroVels:TaskMap {
  TaskMap_qZeroVels(){ }

  virtual void phi(arr& y, arr& J, const mlr::KinematicWorld& G, int t=-1){NIY}
  virtual void phi(arr& y, arr& J, const WorldL& G, double tau, int t);
  virtual uint dim_phi(const mlr::KinematicWorld& G){NIY}
  virtual uint dim_phi(const WorldL& G, int t);
private:
  uintA dimPhi;
};

//===========================================================================

mlr::Array<mlr::Joint*> getMatchingJoints(const WorldL& G, bool zeroVelJointsOnly);
mlr::Array<mlr::Joint*> getSwitchedJoints(const mlr::KinematicWorld& G0, const mlr::KinematicWorld& G1, int verbose=0);
mlr::Array<mlr::Body*> getSwitchedBodies(const mlr::KinematicWorld& G0, const mlr::KinematicWorld& G1, int verbose=0);

