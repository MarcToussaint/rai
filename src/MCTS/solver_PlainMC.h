#pragma once

#include <Core/array.h>
#include <Core/graph.h>

#include "environment.h"

//===========================================================================

struct PlainMC{
  struct Statistics{ uint n=0; arr X; };

  MCTS_Environment& world;
  MT::Array<MCTS_Environment::Handle> A;  ///< what decision do we have in the start state
  MT::Array<Statistics> D;                ///< data of returns for all first actions
  double gamma=.9;
  int verbose=2;
  uint topSize=10;

  PlainMC(MCTS_Environment& world);
  void reset();

  void addRollout(int stepAbort=-1);                 ///< adds one more rollout to the tree
  void report();
  MCTS_Environment::Handle getBestAction();
};

//===========================================================================

