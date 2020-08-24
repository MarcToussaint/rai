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

struct AStar;
struct AStar_Node;
typedef rai::Array<AStar_Node*> AStar_NodeL;

//===========================================================================

struct AStar_Node {
  AStar& astar;
  MCTS_Environment& world;
  MCTS_Environment::Handle action;
  MCTS_Environment::Handle state;
  MCTS_Environment::TransitionReturn ret;

  AStar_Node* parent;
  rai::Array<AStar_Node*> children;

  uint d;      ///< decision depth of this node
  double time; ///< real time
  double g=0.; ///< cost-so-far, path cost (TODO: replace by array of bounds)
  double h=0.; ///< cost-so-far, path cost (TODO: replace by array of bounds)

  bool isExpanded=false;
  bool isInfeasible=false;
  bool isTerminal=false;

  /// root node init
  AStar_Node(AStar& astar, MCTS_Environment& world);

  /// child node creation
  AStar_Node(AStar_Node* parent, const MCTS_Environment::Handle& a);

  ~AStar_Node() { NIY; }

  //- computations on the node
  void expand();           ///< expand this node (symbolically: compute possible decisions and add their effect nodes)

  //-- helpers
  void labelInfeasible(); ///< sets the infeasible label AND removes all children!
  AStar_NodeL getTreePath(); ///< return the decision path in terms of a list of nodes (just walking to the root)
  AStar_Node* getRoot(); ///< return the decision path in terms of a list of nodes (just walking to the root)
  void getAllChildren(AStar_NodeL& tree);

  bool recomputeAllFolStates();
  void recomputeAllMCStats(bool excludeLeafs=true);

  void checkConsistency();

  void write(ostream& os=cout, bool recursive=false) const;
  void getGraph(rai::Graph& G, rai::Node* n=nullptr);
  rai::Graph getGraph() { rai::Graph G; getGraph(G, nullptr); G.checkConsistency(); return G; }

  void getAll(AStar_NodeL& L);
  AStar_NodeL getAll() { AStar_NodeL L; getAll(L); return L; }
};
stdOutPipe(AStar_Node)

//===========================================================================

struct AStar {
  AStar_Node* root;
  PriorityQueue<AStar_Node*> queue;
  rai::Array<AStar_Node*> solutions;
  uint size, depth;

  AStar(MCTS_Environment& world);

  bool step();
  void run();

  void reportQueue();
};

//===========================================================================

