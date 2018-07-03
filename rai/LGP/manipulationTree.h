/*  ------------------------------------------------------------------
    Copyright (c) 2017 Marc Toussaint
    email: marc.toussaint@informatik.uni-stuttgart.de

    This code is distributed under the MIT License.
    Please see <root-path>/LICENSE for details.
    --------------------------------------------------------------  */

#pragma once

#include <Kin/kin.h>
#include <Logic/fol_mcts_world.h>
#include <Logic/fol.h>
#include <KOMO/komo.h>

struct MNode;
typedef rai::Array<MNode*> MNodeL;

//struct SkeletonEntry{ StringL symbols; uint k0,k1; double phase0, phase1; };
//typedef rai::Array<SkeletonEntry> Skeleton;

extern uint COUNT_kin, COUNT_evals, COUNT_node;
extern uintA COUNT_opt;
extern double COUNT_time;
extern rai::String OptLGPDataPath;
extern ofstream *filNodes;
extern bool LGP_useHoming;

enum LEVEL { l_symbolic=0, l_pose=1, l_seq=2, l_path=3 };

//===========================================================================

struct MNode {
  MNode *parent;
  rai::Array<MNode*> children;
  uint step;            ///< decision depth/step of this node
  double time;          ///< real time
  uint id;
  
  //-- info on the symbolic state and action this node represents
  FOL_World& fol; ///< the symbolic KB (all Graphs below are subgraphs of this large KB)
  FOL_World::Handle decision; ///< the decision that led to this node
  FOL_World::TransitionReturn ret;
  Graph *folState=NULL; ///< the symbolic state after the decision
  Node  *folDecision=NULL; ///< the predicate in the folState that represents the decision
  Graph *folAddToState=NULL; ///< facts that are added to the state /after/ the fol.transition, e.g., infeasibility predicates
  
  //-- kinematics: the kinematic structure of the world after the decision path
  const rai::KinematicWorld& startKinematics; ///< initial start state kinematics
  rai::KinematicWorld effKinematics; ///< the effective kinematics (computed from kinematics and symbolic state)
  
  bool isExpanded=false;
  bool isInfeasible=false;
  bool isTerminal=false;
  
  //-- bound values
  uint L;           ///< number of bound levels
  arr cost;         ///< cost-so-far for each level
  arr constraints;  ///< constraint violation (so-far) -- when significantly>0 indicates infeasibility
  boolA feasible;   ///< feasibility for each level
  uintA count;      ///< how often was this level evaluated
  arr computeTime;  ///< computation times for each level
  double bound=0.;
  
  rai::Array<KOMO*> komoProblem; //komo problems for all levels
  arrA opt; //these are the optima (trajectories) computed
  
  // display helpers
  rai::String note;
  
  /// root node init
  MNode(rai::KinematicWorld& kin, FOL_World& fol, uint levels);
  
  /// child node creation
  MNode(MNode *parent, FOL_World::Handle& a);
  
  ~MNode();
  
  //- computations on the node
  void expand(int verbose=0);           ///< expand this node (symbolically: compute possible decisions and add their effect nodes)
  void optLevel(uint level, bool collisions=false);
  void resetData();
  
  //-- helpers to get other nodes
  MNodeL getTreePath() const; ///< return the decision path in terms of a list of nodes (just walking to the root)
  rai::String getTreePathString(char sep=' ') const;
  MNode* getRoot(); ///< return the decision path in terms of a list of nodes (just walking to the root)
  MNode* getChildByAction(Node  *folDecision); ///<
  void getAll(MNodeL& L);
  MNodeL getAll() { MNodeL L; getAll(L); return L; }
  void checkConsistency();
  
  Skeleton getSkeleton(StringA predicateFilter={}, bool finalStateOnly=false) const;
private:
  void setInfeasible(); ///< set this and all children infeasible
  void labelInfeasible(); ///< sets this infeasible AND propagates this label up-down to others
  MNode *treePolicy_random(); ///< returns leave -- by descending children randomly
  MNode *treePolicy_softMax(double temperature);
  bool recomputeAllFolStates();
  
public:
  void write(ostream& os=cout, bool recursive=false, bool path=true) const;
  void getGraph(Graph& G, Node *n=NULL, bool brief=false);
  Graph getGraph(bool brief=false) { Graph G; getGraph(G, NULL, brief); G.checkConsistency(); return G; }
};

inline ostream& operator<<(ostream& os, const MNode& n) { n.write(os); return os; }

//===========================================================================
