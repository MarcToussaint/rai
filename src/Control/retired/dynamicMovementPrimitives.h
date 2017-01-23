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

#ifndef DYNAMICMOVEMENTPRIMITIVES_H
#define DYNAMICMOVEMENTPRIMITIVES_H

#include <Core/util.h>
#include <Core/array.h>
#include <Kin/kin.h>


struct DynamicMovementPrimitives {
  DynamicMovementPrimitives(arr &y_ref_, uint nBase_, double dt_, double lambda_=1e-7);
  ~DynamicMovementPrimitives();

  void trainDMP();
  void iterate();
  void plotDMP();
  void printDMP();
  void reset();
  void changeGoal(const arr &goal_);

  double lambda; // regularization parameter
  double tau; // Time constant, T = 0.5/tau
  double T; // Motion time [s]
  double dt; // Time step rate [s]
  uint nBase; // Number of basis functions for force
  uint dimY; // Dimension of the DMP
  arr weights; // Weights of the force f = phi(x)'*w
  arr amp; // Amplitude of the DMP

  arr y0; // Start state

  arr goal; // Goal state

  double alphax; // Canonical system constant
  double alphay; // Dynamics constant
  double betay; // Dynamics constant

  arr C; // Centers of basis functions
  arr  H; // Widths of basis functions

  double X; // Canonical system state
  double Xd;

  arr Y; // Dynamical system states
  arr Yd;
  arr Ydd;

  arr y_ref; // Reference trajectory

  arr PHI; // basis function activation

  // bookkeeping
  arr y_bk;
  arr yd_bk;
  arr x_bk;
};

#endif // DYNAMICMOVEMENTPRIMITIVES_H
