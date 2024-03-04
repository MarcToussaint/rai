/*  ------------------------------------------------------------------
    Copyright (c) 2011-2024 Marc Toussaint
    email: toussaint@tu-berlin.de

    This code is distributed under the MIT License.
    Please see <root-path>/LICENSE for details.
    --------------------------------------------------------------  */

#pragma once

#include "TreeSearchNode.h"
#include "../Algo/priorityQueue.h"

//===========================================================================

namespace rai {

struct AStar {
  enum SearchMode { astar=0, treePolicy=1, FIFO=2 };

  typedef std::shared_ptr<TreeSearchNode> NodeP;
  rai::Array<NodeP> mem;
  NodeP root;
  PriorityQueue<TreeSearchNode*> queue;
  rai::Array<TreeSearchNode*> solutions;
  uint steps=0;
  int verbose=1;
  double currentLevel=0.;
  SearchMode mode = astar;

  AStar(const std::shared_ptr<TreeSearchNode>& _root, SearchMode _mode = astar);

  void step();
  bool run(int stepsLimit=-1);
  void report();
  bool isEmpty() { return mode==astar && !queue.N; }

  TreeSearchNode* selectByTreePolicy();

 private:
  void addToQueue(TreeSearchNode* node);
};

} //namespace

//===========================================================================

