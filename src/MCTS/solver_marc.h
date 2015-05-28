#pragma once

#include <Core/array.h>
#include <Core/graph.h>

#include "env_marc.h"

//===========================================================================

struct Node{
  Node *parent;
  MT::Array<Node*> children;
  MCTS_Environment::Handle decision;           ///< what decision (relative to the parent) does this node represent

  double Qup,Qme,Qlo;   ///< upper, mean, and lower Q estimates
  double r, R;          ///< last and total immediate rewards
  uint N;               ///< # of visits (to normalize immediate rewards)
  double Q;             ///< total (on-policy) returns

  uint t;               ///< depth of this node
  void *data;           ///< dummy helper (to convert to other data structures)

  Node(Node *parent, MCTS_Environment::Handle decision):parent(parent), decision(decision), Qup(0.), Qme(0.), Qlo(0.), r(0.), R(0.), N(0), Q(0.), t(0), data(NULL){
    if(parent){
      t=parent->t+1;
      parent->children.append(this);
    }
  }
};

//===========================================================================

struct MCTS{
  MCTS_Environment& world;
  Node root;
  int verbose;

  MCTS(MCTS_Environment& world):world(world), root(NULL, NULL), verbose(2){}

  void addRollout();                 ///< adds one more rollout to the tree
  Node* treePolicy(Node *n);   ///< policy to choose the child from which to do a rollout or to expand
  double Qvalue(Node* n, int optimistic); ///< current value estimates at a node
  arr Qfunction(Node* n=NULL, int optimistic=0); ///< the Q-function (value estimates of all children) at a node

  //only to display
  void writeToGraph(Graph& G, Node* n=NULL);
  Graph getGraph(){ Graph G; writeToGraph(G); return G; }
};

//===========================================================================

