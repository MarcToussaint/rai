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

#include "taskMaps.h"

struct TaskMap_PairCollision : TaskMap{
  int i, j;               ///< which shapes does it refer to?
  bool negScalar;

  TaskMap_PairCollision(const mlr::Frame *s1, const mlr::Frame *s2, bool negScalar=false);
  TaskMap_PairCollision(const mlr::KinematicWorld& K, const char* s1, const char* s2, bool negScalar=false);
  virtual void phi(arr& y, arr& J, const mlr::KinematicWorld& K, int t=-1);
  virtual uint dim_phi(const mlr::KinematicWorld& G){ if(negScalar) return 1;  return 3; }
  virtual mlr::String shortTag(const mlr::KinematicWorld& G);
};
