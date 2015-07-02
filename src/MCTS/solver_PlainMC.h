#pragma once

#include <Core/array.h>
#include <Core/graph.h>

#include "env_marc.h"

//===========================================================================

struct PlainMC{
  struct Statistics{ arr X; };

  MCTS_Environment& world;
  MT::Array<MCTS_Environment::Handle> A;  ///< what decision do we have in the start state
  MT::Array<Statistics> D;                ///< data of returns for all first actions
  int verbose;


  PlainMC(MCTS_Environment& world);
  void reset();

  void addRollout(int stepAbort=-1);                 ///< adds one more rollout to the tree
  void report();
  MCTS_Environment::Handle getBestAction();
};

//===========================================================================

