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

//#include "manipulationTree.h"
#include "pathProblem.h"
#include "effectivePoseProblem.h"

#include "towerProblem.h"

#include <KOMO/komo.h>
#include <Logic/fol_mcts_world.h>

//===========================================================================

struct LogicGeometricProgram{
  rai::KinematicWorld world_root;
  FOL_World fol_root;

  LogicGeometricProgram(){}
  ~LogicGeometricProgram(){}

  virtual bool isFeasible(const rai::KinematicWorld& world, const Graph& symbols) = 0;
  virtual double psi(const rai::KinematicWorld& world, const Graph& symbols) = 0;
  virtual KOMO& getPathProblem(const rai::KinematicWorld& world, const Graph& symbols) = 0;
  virtual ConstrainedProblem& getEffPoseProblem(const rai::KinematicWorld& world, const Graph& symbols) = 0;
};

//===========================================================================

struct TowerProblem_new:LogicGeometricProgram{
  uint nObjects;

  TowerProblem_new(){
    world_root.init("LGP-world.g");
    fol_root.init(FILE("LGP-symbols.g"));
    nObjects = world_root.bodies.N;
    setRandom();
    nObjects = world_root.bodies.N - nObjects;
  }
  ~TowerProblem_new(){}
  void setRandom();

  bool isFeasible(const rai::KinematicWorld& world, const Graph& symbols){ NIY }
  double psi(const rai::KinematicWorld& world, const Graph& symbols){ NIY }
  KOMO& getPathProblem(const rai::KinematicWorld& world, const Graph& symbols){ NIY }
  ConstrainedProblem& getEffPoseProblem(const rai::KinematicWorld& world, const Graph& symbols){ NIY }
};

//===========================================================================

struct SticksProblem:LogicGeometricProgram{
  SticksProblem(){
    world_root.init("LGP-sticks-world.g");
    fol_root.init(FILE("LGP-sticks-symbols.g"));
  }
  ~SticksProblem(){}

  bool isFeasible(const rai::KinematicWorld& world, const Graph& symbols){ NIY }
  double psi(const rai::KinematicWorld& world, const Graph& symbols){ NIY }
  KOMO& getPathProblem(const rai::KinematicWorld& world, const Graph& symbols){ NIY }
  ConstrainedProblem& getEffPoseProblem(const rai::KinematicWorld& world, const Graph& symbols){ NIY }
};


//===========================================================================

void runMonteCarlo(Graph& symbols);

