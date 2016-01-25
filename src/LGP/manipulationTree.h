#pragma once

#include <Ors/ors.h>
#include <FOL/fol_mcts_world.h>
#include "LGP.h"
#include <FOL/fol.h>
#include <Motion/komo.h>

struct ManipulationTree_Node;
typedef mlr::Array<ManipulationTree_Node*> ManipulationTree_NodeL;

//===========================================================================

struct ManipulationTree_Node{
  LogicGeometricProgram &lgp;
  ManipulationTree_Node *parent;
  mlr::Array<ManipulationTree_Node*> children;
  uint s;               ///< depth/step of this node
//  double t;             ///< real time
  uint graphIndex=0;

  //-- info on the state and action this node represents
  FOL_World& fol; ///< the symbolic KB (all Graphs below are subgraphs of this large KB)
  FOL_World::Handle decision; ///< the decision that led to this node
  Graph *folState; ///< the symbolic state after the decision
  Node  *folDecision; ///< the predicate in the folState that represents the decision

  //-- kinematics: the kinematic structure of the world after the decision path
  ors::KinematicWorld kinematics; ///< actual kinematics after action (includes all previous switches)
  ors::KinematicWorld effKinematics; ///< the effective kinematics (computed from kinematics and symbolic state)

  bool isExpanded=false;
  bool hasEffKinematics=false;

  //-- specs and results of the three optimization problems
  MotionProblem poseProblem, seqProblem, pathProblem;
  Graph *poseProblemSpecs, *seqProblemSpecs, *pathProblemSpecs;
  arr pose, seq, path;
  double poseCost, seqCost, pathCost;
  double effPoseReward;

  /// root node init
  ManipulationTree_Node(LogicGeometricProgram& lgp);

  /// child node creation
  ManipulationTree_Node(LogicGeometricProgram& lgp, ManipulationTree_Node *parent, FOL_World::Handle& a);

  //- computations on the node
  void expand();           ///< expand this node (symbolically: compute possible decisions and add their effect nodes)
  void solvePoseProblem(); ///< solve the effective pose problem
  void solveSeqProblem(int verbose=0);  ///< compute a sequence of key poses along the decision path
  void solvePathProblem(uint microSteps, int verbose=0); ///< compute a full path along the decision path

  //-- helpers
  ManipulationTree_NodeL getTreePath(); ///< return the decision path in terms of a list of nodes (just walking to the root)

  void write(ostream& os=cout) const;
  void getGraph(Graph& G, Node *n=NULL);
  Graph getGraph(){ Graph G; getGraph(G, NULL); G.checkConsistency(); return G; }
};

inline ostream& operator<<(ostream& os, const ManipulationTree_Node& n){ n.write(os); return os; }

//===========================================================================

//struct ManipulationTree{
//  ManipulationTree_Node root;

//  LGP lgp;

//  ManipulationTree(const ors::KinematicWorld& world_root, const Graph& symbols_root)
//    : root(world_root, symbols_root), fol(FILE("fol.g")), mc(fol) {}

//  ManipulationTree_Node& getRndNode();
//  void addRollout(){
//    mc.addRollout(100);
//  }

//  void optimEffPose(ManipulationTree_Node& n);
//  void optimPath(ManipulationTree_Node& n);

//}
