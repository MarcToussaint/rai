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
#include "bounds.h"

struct LGP_Node;
typedef rai::Array<LGP_Node*> MNodeL;

//struct SkeletonEntry{ StringL symbols; uint k0,k1; double phase0, phase1; };
//typedef rai::Array<SkeletonEntry> Skeleton;

extern uint COUNT_kin, COUNT_evals, COUNT_node;
extern uintA COUNT_opt;
extern double COUNT_time;
extern rai::String OptLGPDataPath;
extern ofstream *filNodes;
extern bool LGP_useHoming;

//===========================================================================

struct LGP_Node {
  LGP_Node *parent;
  rai::Array<LGP_Node*> children;
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
  double highestBound=0.;
  
  rai::Array<std::shared_ptr<KOMO>> komoProblem; //komo problems for all levels
  arrA opt; //these are the optima (trajectories) computed
  
  // display helpers
  rai::String note;
  
  /// root node init
  LGP_Node(rai::KinematicWorld& kin, FOL_World& fol, uint levels);
  
  /// child node creation
  LGP_Node(LGP_Node *parent, FOL_World::Handle& a);
  
  ~LGP_Node();
  
  //- computations on the node
  void expand(int verbose=0);           ///< expand this node (symbolically: compute possible decisions and add their effect nodes)
  void optBound(BoundType bound, bool collisions=false, int verbose=-1);
  ptr<KOMO> optSubCG(const SubCG& scg, bool collisions, int verbose);
  ptr<CG> getCGO(bool collisions=false, int verbose=-1);
  void resetData();
  void computeEndKinematics();
  
  //-- helpers to get other nodes
  MNodeL getTreePath() const; ///< return the decision path in terms of a list of nodes (just walking to the root)
  rai::String getTreePathString(char sep=' ') const;
  LGP_Node* getRoot(); ///< return the decision path in terms of a list of nodes (just walking to the root)
  LGP_Node* getChildByAction(Node  *folDecision); ///<
  void getAll(MNodeL& L);
  MNodeL getAll() { MNodeL L; getAll(L); return L; }
  void checkConsistency();
  
  Skeleton getSkeleton(StringA predicateFilter={}, bool finalStateOnly=false) const;
private:
  void setInfeasible(); ///< set this and all children infeasible
  void labelInfeasible(); ///< sets this infeasible AND propagates this label up-down to others
  LGP_Node *treePolicy_random(); ///< returns leave -- by descending children randomly
  LGP_Node *treePolicy_softMax(double temperature);
  bool recomputeAllFolStates();
  
public:
  void write(ostream& os=cout, bool recursive=false, bool path=true) const;
  Graph getInfo() const;
  void getGraph(Graph& G, Node *n=NULL, bool brief=false);
  Graph getGraph(bool brief=false) { Graph G; getGraph(G, NULL, brief); G.checkConsistency(); return G; }
};

inline ostream& operator<<(ostream& os, const LGP_Node& n) { n.write(os); return os; }

//===========================================================================
