/*  ------------------------------------------------------------------
    Copyright (c) 2017 Marc Toussaint
    email: marc.toussaint@informatik.uni-stuttgart.de
    
    This code is distributed under the MIT License.
    Please see <root-path>/LICENSE for details.
    --------------------------------------------------------------  */

#pragma once
#include "taskMap.h"

struct TM_PushConsistent : TaskMap {
  int i, j;               ///< which shapes does it refer to?

  TM_PushConsistent(int iShape=-1, int jShape=-1);

  TM_PushConsistent(const mlr::KinematicWorld& G,
                         const char* iShapeName=NULL, const char* jShapeName=NULL);

  virtual void phi(arr& y, arr& J, const WorldL& G, double tau, int t=-1);
  virtual void phi(arr& y, arr& J, const mlr::KinematicWorld& G, int t=-1){  HALT("you shouldn't be here!");  }
  virtual uint dim_phi(const mlr::KinematicWorld& G){ return 3; }
  virtual mlr::String shortTag(const mlr::KinematicWorld& G);
};

