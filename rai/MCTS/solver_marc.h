/*  ------------------------------------------------------------------
    Copyright (c) 2017 Marc Toussaint
    email: marc.toussaint@informatik.uni-stuttgart.de

    This code is distributed under the MIT License.
    Please see <root-path>/LICENSE for details.
    --------------------------------------------------------------  */

#pragma once

#include <Core/array.h>
#include <Core/graph.h>

#include "environment.h"

//===========================================================================

struct MCTS_Node {
  MCTS_Node *parent;
  rai::Array<MCTS_Node*> children;
  MCTS_Environment::Handle decision;           ///< what decision (relative to the parent) does this node represent
  
  double Qup,Qme,Qlo;   ///< upper, mean, and lower Q estimates
  double r, R;          ///< last and total immediate rewards
  uint N;               ///< # of visits (to normalize immediate rewards)
  double Q;             ///< total (on-policy) returns
  
  uint t;               ///< depth of this node
  void *data;           ///< dummy helper (to convert to other data structures)
  
  MCTS_Node(MCTS_Node *parent, MCTS_Environment::Handle decision):parent(parent), decision(decision), Qup(0.), Qme(0.), Qlo(0.), r(0.), R(0.), N(0), Q(0.), t(0), data(NULL) {
    if(parent) {
      t=parent->t+1;
      parent->children.append(this);
    }
  }
};

//===========================================================================

struct MCTS {
  MCTS_Environment& world;
  MCTS_Node root;
  int verbose;
  double beta;
  
  MCTS(MCTS_Environment& world):world(world), root(NULL, NULL), verbose(2), beta(2.) {}
  
  void addRollout(int stepAbort=-1);                 ///< adds one more rollout to the tree
  MCTS_Node* treePolicy(MCTS_Node *n);   ///< policy to choose the child from which to do a rollout or to expand
  double Qvalue(MCTS_Node* n, int optimistic); ///< current value estimates at a node
  arr Qfunction(MCTS_Node* n=NULL, int optimistic=0); ///< the Q-function (value estimates of all children) at a node
  arr Qvariance(MCTS_Node* n=NULL);
  void reportQ(ostream& os, MCTS_Node* n=NULL);
  uint Nnodes(MCTS_Node *n=NULL, bool subTree=true);
  void reportDecisions(ostream& os, MCTS_Node* n=NULL);
  
  //only to display
  void writeToGraph(Graph& G, MCTS_Node* n=NULL);
  Graph getGraph() { Graph G; writeToGraph(G); return G; }
};

//===========================================================================

