/*  ------------------------------------------------------------------
    Copyright (c) 2017 Marc Toussaint
    email: marc.toussaint@informatik.uni-stuttgart.de
    
    This code is distributed under the MIT License.
    Please see <root-path>/LICENSE for details.
    --------------------------------------------------------------  */

#pragma once

#include "taskMap.h"

//===========================================================================

struct TM_FlagConstraints : TaskMap {
  double g=1.; //gravity constant, usually 9.81
  TM_FlagConstraints(){ g = mlr::getParameter<double>("FlagConstraints/gravity", 1.); }
  virtual void phi(arr& y, arr& J, const WorldL& G, double tau, int t=-1);
  virtual void phi(arr& y, arr& J, const mlr::KinematicWorld& G, int t=-1){ HALT("can only be of order 1"); }
  virtual uint dim_phi(const mlr::KinematicWorld& K){ HALT("can only be of order 1"); }
  virtual uint dim_phi(const WorldL& Ktuple, int t);
  virtual mlr::String shortTag(const mlr::KinematicWorld& G){ return STRING("FlagConstraints"); }
};

//===========================================================================

struct TM_FlagCosts : TaskMap {
  TM_FlagCosts(){}
  virtual void phi(arr& y, arr& J, const WorldL& G, double tau, int t=-1);
  virtual void phi(arr& y, arr& J, const mlr::KinematicWorld& G, int t=-1){ HALT("can only be of order 1"); }
  virtual uint dim_phi(const mlr::KinematicWorld& K){ HALT("can only be of order 1"); }
  virtual uint dim_phi(const WorldL& Ktuple, int t);
  virtual mlr::String shortTag(const mlr::KinematicWorld& G){ return STRING("TM_FlagCosts"); }
};
