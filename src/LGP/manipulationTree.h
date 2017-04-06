/*  ------------------------------------------------------------------
    Copyright 2016 Marc Toussaint
    email: marc.toussaint@informatik.uni-stuttgart.de
    
    This program is free software: you can redistribute it and/or modify
    it under the terms of the GNU General Public License as published by
    the Free Software Foundation, either version 3 of the License, or (at
    your option) any later version. This program is distributed without
    any warranty. See the GNU General Public License for more details.
    You should have received a COPYING file of the full GNU General Public
    License along with this program. If not, see
    <http://www.gnu.org/licenses/>
    --------------------------------------------------------------  */


#pragma once

#include <Kin/kin.h>
#include <Logic/fol_mcts_world.h>
#include "LGP.h"
#include <Logic/fol.h>
#include <KOMO/komo.h>

struct ManipulationTree_Node;
struct PlainMC;
struct MCStatistics;
typedef mlr::Array<ManipulationTree_Node*> ManipulationTree_NodeL;

extern uint COUNT_kin, COUNT_evals, COUNT_poseOpt, COUNT_seqOpt, COUNT_pathOpt;

//===========================================================================

struct ManipulationTree_Node{
  ManipulationTree_Node *parent;
  mlr::Array<ManipulationTree_Node*> children;
  uint s;               ///< decision depth/step of this node
  double time;          ///< real time
  uint graphIndex=0;

  //-- info on the state and action this node represents
  FOL_World& fol; ///< the symbolic KB (all Graphs below are subgraphs of this large KB)
  FOL_World::Handle decision; ///< the decision that led to this node
  Graph *folState; ///< the symbolic state after the decision
  Node  *folDecision; ///< the predicate in the folState that represents the decision
  double folReward;  ///< the reward collected with this transition step
  Graph *folAddToState; ///< facts that are added to the state /after/ the fol.transition, e.g., infeasibility predicates

  //-- kinematics: the kinematic structure of the world after the decision path
  const mlr::KinematicWorld& startKinematics; ///< initial start state kinematics
  mlr::KinematicWorld effKinematics; ///< the effective kinematics (computed from kinematics and symbolic state)

  bool isExpanded=false;
  bool hasEffKinematics=false;
  bool isInfeasible=false;
  bool isTerminal=false;

  //-- specs and results of the three optimization problems
//  KOMO *poseProblem, *seqProblem, *pathProblem;
  PlainMC *rootMC;
  MCStatistics *mcStats;
  KOMO *poseProblem, *seqProblem, *pathProblem;
  Graph *poseProblemSpecs, *seqProblemSpecs, *pathProblemSpecs;
  arr pose, seq, path;
  uint mcCount, poseCount, seqCount, pathCount;
  double symCost, poseCost, poseConstraints, seqCost, seqConstraints, pathCost, pathConstraints;
  bool symTerminal, poseFeasible, seqFeasible, pathFeasible;

  bool inFringe1, inFringe2;

  /// root node init
  ManipulationTree_Node(mlr::KinematicWorld& kin, FOL_World& fol);

  /// child node creation
  ManipulationTree_Node(ManipulationTree_Node *parent, FOL_World::Handle& a);

  //- computations on the node
  void expand();           ///< expand this node (symbolically: compute possible decisions and add their effect nodes)
  arr generateRootMCRollouts(uint num, int stepAbort, const mlr::Array<MCTS_Environment::Handle>& prefixDecisions);
  void addMCRollouts(uint num,int stepAbort);
  void solvePoseProblem(); ///< solve the effective pose problem
  void solveSeqProblem(int verbose=0);  ///< compute a sequence of key poses along the decision path
  void solvePathProblem(uint microSteps, int verbose=0); ///< compute a full path along the decision path

  //-- helpers
  void labelInfeasible(); ///< sets the infeasible label AND removes all children!
  ManipulationTree_NodeL getTreePath(); ///< return the decision path in terms of a list of nodes (just walking to the root)
  ManipulationTree_Node* getRoot(); ///< return the decision path in terms of a list of nodes (just walking to the root)
  void getAllChildren(ManipulationTree_NodeL& tree);
  ManipulationTree_Node *treePolicy_random(); ///< returns leave -- by descending children randomly
  ManipulationTree_Node *treePolicy_softMax(double temperature);
  bool recomputeAllFolStates();
  void recomputeAllMCStats(bool excludeLeafs=true);

  void checkConsistency();

  void write(ostream& os=cout, bool recursive=false) const;
  void getGraph(Graph& G, Node *n=NULL);
  Graph getGraph(){ Graph G; getGraph(G, NULL); G.checkConsistency(); return G; }
  void getAll(ManipulationTree_NodeL& L);
  ManipulationTree_NodeL getAll(){ ManipulationTree_NodeL L; getAll(L); return L; }
};

inline ostream& operator<<(ostream& os, const ManipulationTree_Node& n){ n.write(os); return os; }

//===========================================================================

//struct ManipulationTree{
//  ManipulationTree_Node root;

//  LGP lgp;

//  ManipulationTree(const mlr::KinematicWorld& world_root, const Graph& symbols_root)
//    : root(world_root, symbols_root), fol(FILE("fol.g")), mc(fol) {}

//  ManipulationTree_Node& getRndNode();
//  void addRollout(){
//    mc.addRollout(100);
//  }

//  void optimEffPose(ManipulationTree_Node& n);
//  void optimPath(ManipulationTree_Node& n);

//}
