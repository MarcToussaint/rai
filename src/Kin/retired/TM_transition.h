/*  ------------------------------------------------------------------
    Copyright (c) 2011-2020 Marc Toussaint
    email: toussaint@tu-berlin.de

    This code is distributed under the MIT License.
    Please see <root-path>/LICENSE for details.
    --------------------------------------------------------------  */

#pragma once

#include "feature.h"

//===========================================================================

/// defines a transition cost vector, which is q.N-dimensional and captures
/// accelerations or velocities over consecutive time steps
struct TM_Transition:Feature {
  double posCoeff, velCoeff, accCoeff;  ///< coefficients to blend between velocity and acceleration penalization
  arr H_rate_diag;            ///< cost rate (per TIME, not step), given as diagonal of the matrix H
  double H_rate;  ///< cost rate (per TIME, not step), given as scalar, will be multiplied by Joint->H (given in ors file)
  bool effectiveJointsOnly;

  TM_Transition(const rai::Configuration& G, bool effectiveJointsOnly=false);

  virtual void phi(arr& y, arr& J, const ConfigurationL& Ktuple);
  virtual void phi(arr& y, arr& J, const rai::Configuration& G) { HALT("can only be of higher order"); }
  virtual uint dim_phi(const rai::Configuration& G) { return G.getJointStateDimension(); }
  virtual uint dim_phi(const ConfigurationL& G);
  virtual rai::String shortTag(const rai::Configuration& G) { return STRING("Transition:"<<(effectiveJointsOnly?"eDOF":"") <<":pos" <<posCoeff <<":vel" <<velCoeff<<":acc"<<accCoeff); }
  virtual Graph getSpec(const rai::Configuration& K) { return Graph({{"feature", "Transition"}}); }
};
