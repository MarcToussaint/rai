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
struct TM_Transition:TaskMap {
  double posCoeff, velCoeff, accCoeff;  ///< coefficients to blend between velocity and acceleration penalization
  arr H_rate_diag;            ///< cost rate (per TIME, not step), given as diagonal of the matrix H
  double H_rate;  ///< cost rate (per TIME, not step), given as scalar, will be multiplied by Joint->H (given in ors file)
  bool effectiveJointsOnly;

  TM_Transition(const rai::KinematicWorld& G, bool effectiveJointsOnly=false);

  virtual void phi(arr& y, arr& J, const WorldL& Ktuple, double tau, int t=-1);
  virtual void phi(arr& y, arr& J, const rai::KinematicWorld& G, int t=-1){ HALT("can only be of higher order"); }
  virtual uint dim_phi(const rai::KinematicWorld& G){ return G.getJointStateDimension(); }
  virtual uint dim_phi(const WorldL& G, int t);
  virtual rai::String shortTag(const rai::KinematicWorld& G){ return STRING("Transition:"<<(effectiveJointsOnly?"eDOF":"") <<":pos" <<posCoeff <<":vel" <<velCoeff<<":acc"<<accCoeff); }
};
