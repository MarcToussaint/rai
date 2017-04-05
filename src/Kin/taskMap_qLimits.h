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

struct TaskMap_qLimits:TaskMap {
  //TODO (danny) allow margin specification
  arr limits;

  TaskMap_qLimits(const arr& _limits=NoArr){ if(&_limits) limits=_limits; } ///< if no limits are provided, they are taken from G's joints' attributes on the first call of phi
  virtual void phi(arr& y, arr& J, const mlr::KinematicWorld& G, int t=-1);
  virtual uint dim_phi(const mlr::KinematicWorld& G){ return 1; }
  virtual mlr::String shortTag(const mlr::KinematicWorld& G){ return STRING("TaskMap_qLimits"); }
};

//===========================================================================

struct LimitsConstraint:TaskMap {
  double margin;
  arr limits;
  LimitsConstraint(double _margin=.05):margin(_margin){}
  virtual void phi(arr& y, arr& J, const mlr::KinematicWorld& G, int t=1);
  virtual uint dim_phi(const mlr::KinematicWorld& G){ return 1; }
  virtual mlr::String shortTag(const mlr::KinematicWorld& G){ return STRING("LimitsConstraint"); }
};

