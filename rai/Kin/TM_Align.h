/*  ------------------------------------------------------------------
    Copyright (c) 2017 Marc Toussaint
    email: marc.toussaint@informatik.uni-stuttgart.de
    
    This code is distributed under the MIT License.
    Please see <root-path>/LICENSE for details.
    --------------------------------------------------------------  */

#pragma once

#include "taskMap.h"

struct TM_Align : TaskMap {
  int i, j;               ///< which shapes does it refer to?

  TM_Align(const mlr::KinematicWorld& G, const char* iName=NULL, const char* jName=NULL);

  virtual void phi(arr& y, arr& J, const mlr::KinematicWorld& G, int t=-1);
  virtual uint dim_phi(const mlr::KinematicWorld& G){ return 3; }
  virtual mlr::String shortTag(const mlr::KinematicWorld& G);
};

