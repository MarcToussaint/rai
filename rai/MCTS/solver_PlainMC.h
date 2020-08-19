/*  ------------------------------------------------------------------
    Copyright (c) 2011-2020 Marc Toussaint
    email: toussaint@tu-berlin.de

    This code is distributed under the MIT License.
    Please see <root-path>/LICENSE for details.
    --------------------------------------------------------------  */

#pragma once

#include "environment.h"
#include "../Core/array.h"
#include "../Core/graph.h"

//===========================================================================

// stored per node, or at root only, or root and its children
struct MCStatistics {
  uint n;
  arr X;
  MCStatistics():n(0) {}
  void clear() { X.clear(); n=0; }
  void add(double R, uint topSize=10);
};

//===========================================================================

struct PlainMC {

  MCTS_Environment& world;
  rai::Array<MCTS_Environment::Handle> A;  ///< what decisions do we have in the start state
  rai::Array<MCStatistics> D;                ///< data of returns for all first actions
  MCStatistics Droot;
  double gamma;
  int verbose;
  uint topSize;
  StringA blackList;

  //partly internal: results of a rollout
  uint rolloutStep;
  double rolloutR, rolloutDiscount;
  rai::Array<MCTS_Environment::Handle> rolloutDecisions;

  PlainMC(MCTS_Environment& world);
  void reset();

  double initRollout(const rai::Array<MCTS_Environment::Handle>& prefixDecisions);
  double finishRollout(int stepAbort=-1);
  double generateRollout(int stepAbort=-1, const rai::Array<MCTS_Environment::Handle>& prefixDecisions= {});
  double addRollout(int stepAbort=-1);                 ///< adds one more rollout to the tree
  void addReturnToStatistics(double R, MCTS_Environment::Handle decision, int decisionIndex=-1);
  void report();
  uint getBestActionIdx();
  MCTS_Environment::Handle getBestAction();
};

//===========================================================================

