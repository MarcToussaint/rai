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

  FOL_World& fol;
  FOL_World::Handle decision;
  Graph *folState; ///< the symbolic state after action
  Node  *folDecision; ///< the predicate in the folState that represents the decision

  //  ors::KinematicSwitch sw; ///< the kinematic switch(es) that this action implies
  ors::KinematicWorld kinematics; ///< actual kinematics after action (includes all previous switches)

  //-- results of effective pose optimization
  Graph *poseProblemSpecs;
  ors::KinematicWorld effKinematics; ///< the effective kinematics (computed from kinematics and symbolic state)
  arr effPose;
  double effPoseCost;
  double effPoseReward;

  //-- results of full path optimization
  MotionProblem pathProblem;
  Graph *pathProblemSpecs;
  arr path;
  double pathCost;

  ///root node init
  ManipulationTree_Node(LogicGeometricProgram& lgp);

  ///child node creation
  ManipulationTree_Node(LogicGeometricProgram& lgp, ManipulationTree_Node *parent, FOL_World::Handle& a);

  void expand();
  void solvePoseProblem();
  void solvePathProblem(uint microSteps);

  void write(ostream& os=cout) const;
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
