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
#include <Logic/fol.h>

struct ManipulationTree_Node;
struct PlainMC;
struct MCStatistics;
typedef mlr::Array<ManipulationTree_Node*> ManipulationTree_NodeL;

extern uint COUNT_kin, COUNT_evals;
extern uintA COUNT_opt;

enum LEVEL{ l_symbolic=0, l_pose=1, l_seq=2, l_path=3 };

//===========================================================================

struct ManipulationTree_Node{
  ManipulationTree_Node *parent;
  mlr::Array<ManipulationTree_Node*> children;
  uint step;            ///< decision depth/step of this node
  double time;          ///< real time

  //-- info on the symbolic state and action this node represents
  FOL_World& fol; ///< the symbolic KB (all Graphs below are subgraphs of this large KB)
  FOL_World::Handle decision; ///< the decision that led to this node
  FOL_World::TransitionReturn ret;
  Graph *folState; ///< the symbolic state after the decision
  Node  *folDecision; ///< the predicate in the folState that represents the decision
  Graph *folAddToState; ///< facts that are added to the state /after/ the fol.transition, e.g., infeasibility predicates

  //-- kinematics: the kinematic structure of the world after the decision path
  const mlr::KinematicWorld& startKinematics; ///< initial start state kinematics
  mlr::KinematicWorld effKinematics; ///< the effective kinematics (computed from kinematics and symbolic state)

  bool isExpanded=false;
  bool isInfeasible=false;
  bool isTerminal=false;

  //-- bound values
  uint L;           ///< number of bound levels
  arr cost;         ///< cost-so-far for each level
  arr constraints;  ///< constraint violation (so-far) -- when significantly>0 indicates infeasibility
  boolA feasible;   ///< feasibility for each level
  uintA count;      ///< how often was this level evaluated
  double bound=0.;

  // temporary stuff -- only for convenience to display and store
  // MC stuff -- TODO
  PlainMC *rootMC; //only the root node owns an MC rollout generator
  MCStatistics *mcStats;
  uint mcCount;
  double mcCost;

  mlr::Array<struct KOMO*> komoProblem; //komo problems for all levels
  arrA opt; //these are the optima computed

  // display helpers
  mlr::String note;

  /// root node init
  ManipulationTree_Node(mlr::KinematicWorld& kin, FOL_World& fol, uint levels);

  /// child node creation
  ManipulationTree_Node(ManipulationTree_Node *parent, FOL_World::Handle& a);

  //- computations on the node
  void expand(int verbose=0);           ///< expand this node (symbolically: compute possible decisions and add their effect nodes)
  void optLevel(uint level, bool collisions=false);
  //MC stuff -- TODO
  arr generateRootMCRollouts(uint num, int stepAbort, const mlr::Array<MCTS_Environment::Handle>& prefixDecisions);
  void addMCRollouts(uint num,int stepAbort);

  //-- helpers
  ManipulationTree_NodeL getTreePath() const; ///< return the decision path in terms of a list of nodes (just walking to the root)
  ManipulationTree_Node* getRoot(); ///< return the decision path in terms of a list of nodes (just walking to the root)
  ManipulationTree_Node* getChildByAction(Node  *folDecision); ///<
  void getAll(ManipulationTree_NodeL& L);
  ManipulationTree_NodeL getAll(){ ManipulationTree_NodeL L; getAll(L); return L; }
  void checkConsistency();
private:
  void setInfeasible(); ///< set this and all children infeasible
  void labelInfeasible(); ///< sets this infeasible AND propagates this label up-down to others
  ManipulationTree_Node *treePolicy_random(); ///< returns leave -- by descending children randomly
  ManipulationTree_Node *treePolicy_softMax(double temperature);
  bool recomputeAllFolStates();
  void recomputeAllMCStats(bool excludeLeafs=true);
public:

  void write(ostream& os=cout, bool recursive=false, bool path=true) const;
  void getGraph(Graph& G, Node *n=NULL);
  Graph getGraph(){ Graph G; getGraph(G, NULL); G.checkConsistency(); return G; }
};

inline ostream& operator<<(ostream& os, const ManipulationTree_Node& n){ n.write(os); return os; }

//===========================================================================
