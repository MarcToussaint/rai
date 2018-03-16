/*  ------------------------------------------------------------------
    Copyright (c) 2017 Marc Toussaint
    email: marc.toussaint@informatik.uni-stuttgart.de
    
    This code is distributed under the MIT License.
    Please see <root-path>/LICENSE for details.
    --------------------------------------------------------------  */

#pragma once

#include "taskMap.h"

struct TM_BeliefTransition : TaskMap{
  TaskMap *viewError;
  TM_BeliefTransition(TaskMap *viewError = NULL) : viewError(viewError) {}
  ~TM_BeliefTransition(){ if(viewError) delete viewError; }
  virtual void phi(arr& y, arr& J, const WorldL& G, double tau, int t=-1);
  virtual void phi(arr& y, arr& J, const mlr::KinematicWorld& G, int t=-1){ HALT("can only be of higher order"); }
  virtual uint dim_phi(const mlr::KinematicWorld& G);
  virtual mlr::String shortTag(const mlr::KinematicWorld& G){ return STRING("BeliefTransition"); }
};
