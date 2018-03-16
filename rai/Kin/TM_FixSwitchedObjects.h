/*  ------------------------------------------------------------------
    Copyright (c) 2017 Marc Toussaint
    email: marc.toussaint@informatik.uni-stuttgart.de
    
    This code is distributed under the MIT License.
    Please see <root-path>/LICENSE for details.
    --------------------------------------------------------------  */

#pragma once

#include "taskMap.h"

//===========================================================================

/// defines a transition cost vector, which is q.N-dimensional and captures
/// accelerations or velocities over consecutive time steps
struct TM_FixSwichedObjects:TaskMap {
  TM_FixSwichedObjects(){}
  virtual void phi(arr& y, arr& J, const WorldL& G, double tau, int t=-1);
  virtual void phi(arr& y, arr& J, const mlr::KinematicWorld& G, int t=-1){ HALT("can only be of order 1"); }
  virtual uint dim_phi(const mlr::KinematicWorld& G){ HALT("can only be of order 1"); }
  virtual uint dim_phi(const WorldL& G, int t);
  virtual mlr::String shortTag(const mlr::KinematicWorld& G){ return STRING("FixSwichedObjects"); }
};
