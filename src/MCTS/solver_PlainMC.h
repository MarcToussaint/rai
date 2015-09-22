#pragma once

#include <Core/array.h>
#include <Core/graph.h>

#include "environment.h"

//===========================================================================

struct PlainMC{
  struct Statistics{ uint n; arr X; Statistics():n(0){} };

  MCTS_Environment& world;
  MT::Array<MCTS_Environment::Handle> A;  ///< what decision do we have in the start state
  MT::Array<Statistics> D;                ///< data of returns for all first actions
  double gamma;
  int verbose;
  uint topSize;
  StringA blackList;

  PlainMC(MCTS_Environment& world);
  void reset();

  void addRollout(int stepAbort=-1);                 ///< adds one more rollout to the tree
  void report();
  MCTS_Environment::Handle getBestAction();
};

//===========================================================================

