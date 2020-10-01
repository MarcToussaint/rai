/*  ------------------------------------------------------------------
    Copyright (c) 2011-2020 Marc Toussaint
    email: toussaint@tu-berlin.de

    This code is distributed under the MIT License.
    Please see <root-path>/LICENSE for details.
    --------------------------------------------------------------  */

#pragma once

//#include "manipulationTree.h"
#include "pathProblem.h"
#include "effectivePoseProblem.h"
#include "towerProblem.h"
#include "../KOMO/komo.h"
#include "../Logic/fol_mcts_world.h"

//===========================================================================

struct LogicGeometricProgram {
  rai::Configuration world_root;
  FOL_World fol_root;

  LogicGeometricProgram() {}
  ~LogicGeometricProgram() {}

  virtual bool isFeasible(const rai::Configuration& world, const Graph& symbols) = 0;
  virtual double psi(const rai::Configuration& world, const Graph& symbols) = 0;
  virtual KOMO& getPathProblem(const rai::Configuration& world, const Graph& symbols) = 0;
  virtual ConstrainedProblem& getEffPoseProblem(const rai::Configuration& world, const Graph& symbols) = 0;
};

//===========================================================================

struct TowerProblem_new:LogicGeometricProgram {
  uint nObjects;

  TowerProblem_new() {
    world_root.readFromGraph("LGP-world.g");
    fol_root.init(FILE("LGP-symbols.g"));
    nObjects = world_root.bodies.N;
    setRandom();
    nObjects = world_root.bodies.N - nObjects;
  }
  ~TowerProblem_new() {}
  void setRandom();

  bool isFeasible(const rai::Configuration& world, const Graph& symbols) { NIY }
  double psi(const rai::Configuration& world, const Graph& symbols) { NIY }
  KOMO& getPathProblem(const rai::Configuration& world, const Graph& symbols) { NIY }
  ConstrainedProblem& getEffPoseProblem(const rai::Configuration& world, const Graph& symbols) { NIY }
};

//===========================================================================

struct SticksProblem:LogicGeometricProgram {
  SticksProblem() {
    world_root.readFromGraph("LGP-sticks-world.g");
    fol_root.init(FILE("LGP-sticks-symbols.g"));
  }
  ~SticksProblem() {}

  bool isFeasible(const rai::Configuration& world, const Graph& symbols) { NIY }
  double psi(const rai::Configuration& world, const Graph& symbols) { NIY }
  KOMO& getPathProblem(const rai::Configuration& world, const Graph& symbols) { NIY }
  ConstrainedProblem& getEffPoseProblem(const rai::Configuration& world, const Graph& symbols) { NIY }
};

//===========================================================================

void runMonteCarlo(Graph& symbols);

