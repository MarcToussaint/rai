/*  ------------------------------------------------------------------
    Copyright (c) 2011-2024 Marc Toussaint
    email: toussaint@tu-berlin.de

    This code is distributed under the MIT License.
    Please see <root-path>/LICENSE for details.
    --------------------------------------------------------------  */

#pragma once

#include "../Kin/kin.h"
#include "../Logic/folWorld.h"
#include "../Logic/fol.h"
#include "../KOMO/komo.h"
#include "../KOMO/skeleton.h"

namespace rai {

struct LGP_Node;
struct LGP_Tree;
typedef Array<LGP_Node*> LGP_NodeL;

enum BoundType { BD_all=-1,
                 BD_symbolic=0,
                 BD_pose,
                 BD_seq,
                 BD_path,
                 BD_seqPath,
                 BD_max
               };

rai::SkeletonTranscription skeleton2Bound2(BoundType boundType, rai::Skeleton& S, const Configuration& C, const arrA& waypoints= {});

//===========================================================================

struct LGP_Node {
  LGP_Node* parent;
  LGP_Tree& tree;
  Array<LGP_Node*> children;
  shared_ptr<Skeleton> skeleton;
  Array<SkeletonTranscription> problem;
  uint step;            ///< decision depth/step of this node
  double time;          ///< real time
  uint id;

  //-- info on the symbolic state and action this node represents
  FOL_World& fol; ///< the symbolic KB (all Graphs below are subgraphs of this large KB)
  FOL_World::Handle decision; ///< the decision that led to this node
  FOL_World::TransitionReturn ret;
  Graph* folState=nullptr; ///< the symbolic state after the decision
  Node*  folDecision=nullptr; ///< the predicate in the folState that represents the decision
  Graph* folAddToState=nullptr; ///< facts that are added to the state /after/ the fol.transition, e.g., infeasibility predicates

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

  // display helpers
  String note;

  /// root node init
  LGP_Node(rai::LGP_Tree& _tree, uint levels);

  /// child node creation
  LGP_Node(LGP_Node* parent, FOL_World::Handle& a);

  ~LGP_Node();

  //- computations on the node
  void expand(int verbose=0);           ///< expand this node (symbolically: compute possible decisions and add their effect nodes)
  void optBound(BoundType bound, bool useBroadCollisions=false, int verbose=-1);
  void resetData();

  //-- helpers to get other nodes
  LGP_NodeL getTreePath() const; ///< return the decision path in terms of a list of nodes (just walking to the root)
  String getTreePathString(char sep=' ') const;
  LGP_Node* getRoot(); ///< return the decision path in terms of a list of nodes (just walking to the root)
  LGP_Node* getChildByAction(Node*  folDecision); ///<
  void getAll(LGP_NodeL& L);
  LGP_NodeL getAll() { LGP_NodeL L; getAll(L); return L; }
  void checkConsistency();

  void ensure_skeleton();

 private:
  void setInfeasible(); ///< set this and all children infeasible
  void labelInfeasible(); ///< sets this infeasible AND propagates this label up-down to others
  LGP_Node* treePolicy_random(); ///< returns leave -- by descending children randomly
  bool recomputeAllFolStates();

 public:
  void write(ostream& os=cout, bool recursive=false, bool path=true) const;
  Graph getInfo() const;
  void getGraph(Graph& G, Node* n=nullptr, bool brief=false);
  Graph getGraph(bool brief=false) { Graph G; getGraph(G, nullptr, brief); G.checkConsistency(); return G; }
  void displayBound(ConfigurationViewer& V, BoundType bound);
};

inline ostream& operator<<(ostream& os, const LGP_Node& n) { n.write(os); return os; }

//===========================================================================

} //namespace
