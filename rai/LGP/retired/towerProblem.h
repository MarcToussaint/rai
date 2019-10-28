/*  ------------------------------------------------------------------
    Copyright (c) 2017 Marc Toussaint
    email: marc.toussaint@informatik.uni-stuttgart.de

    This code is distributed under the MIT License.
    Please see <root-path>/LICENSE for details.
    --------------------------------------------------------------  */

#pragma once

#include <Kin/kin.h>

//===========================================================================

struct TowerProblem {
  rai::Configuration world;
  Graph symbols;
  uint nObjects;
  
  TowerProblem():world("world_base.kvg"), symbols("symbols_base.kvg") {
    nObjects = world.bodies.N;
    setRandom();
    nObjects = world.bodies.N - nObjects;
  }
  void setRandom();
  double reward(const rai::Configuration& world, const Graph& symbols);
};
