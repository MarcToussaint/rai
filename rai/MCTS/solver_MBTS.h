/*  ------------------------------------------------------------------
    Copyright (c) 2011-2020 Marc Toussaint
    email: toussaint@tu-berlin.de

    This code is distributed under the MIT License.
    Please see <root-path>/LICENSE for details.
    --------------------------------------------------------------  */

#pragma once

#include "environment.h"
#include "../Core/graph.h"
#include "../Algo/priorityQueue.h"

//===========================================================================

struct MBTS;
struct MBTS_Node;
typedef rai::Array<MBTS_Node*> MBTS_NodeL;

//===========================================================================

struct MBTS_Node {
  MBTS& MBTS;
  MCTS_Environment& world;
  MCTS_Environment::Handle action;
  MCTS_Environment::Handle state;
  MCTS_Environment::TransitionReturn ret;

  MBTS_Node* parent;
  rai::Array<MBTS_Node*> children;

  uint d;      ///< decision depth of this node
  double time; ///< real time

  arr g; ///< cost-so-far for each level
  arr h; ///< cost-to-go heuristic for each level

  boolA isEvaluated; ///< for each level
  boolA isInfeasible; ///< for each level
  bool isTerminal=false; ///< for logic level only

  /// root node init
  MBTS_Node(MBTS& MBTS, MCTS_Environment& world);

  /// child node creation
  MBTS_Node(MBTS_Node* parent, const MCTS_Environment::Handle& a);

  ~MBTS_Node() { NIY; }

  //- computations on the node
  void expand();           ///< expand this node (symbolically: compute possible decisions and add their effect nodes)

  virtual void evaluate(int level=-1) { NIY; }

  //-- helpers
  void labelInfeasible(); ///< sets the infeasible label AND removes all children!
  MBTS_NodeL getTreePath(); ///< return the decision path in terms of a list of nodes (just walking to the root)
  MBTS_Node* getRoot(); ///< return the decision path in terms of a list of nodes (just walking to the root)
  void getAllChildren(MBTS_NodeL& tree);

  bool recomputeAllFolStates();
  void recomputeAllMCStats(bool excludeLeafs=true);

  void checkConsistency();

  void write(ostream& os=cout, bool recursive=false) const;
  void getGraph(Graph& G, Node* n=nullptr);
  Graph getGraph() { Graph G; getGraph(G, nullptr); G.checkConsistency(); return G; }

  void getAll(MBTS_NodeL& L);
  MBTS_NodeL getAll() { MBTS_NodeL L; getAll(L); return L; }
};
stdOutPipe(MBTS_Node)

//===========================================================================

struct MBTS_Heuristic {
  struct Return {
    double g;
    double h;
    bool terminal;
    bool feasible;
  };

  virtual ~MBTS_Heuristic() {}
  virtual Return evaluate(MBTS_Node* n, int level);
};

//===========================================================================

struct MBTS {
  MBTS_Node* root;
  rai::Array<PriorityQueue<MBTS_Node*>> queue; //for each level
  MBTS_Heuristic& heuristic;

  rai::Array<MBTS_Node*> solutions;
  uint size, depth;

  MBTS(MCTS_Environment& world, MBTS_Heuristic& heuristic, uint L);

  bool step(int level);
  void run();

  void reportQueue();
};

//===========================================================================

