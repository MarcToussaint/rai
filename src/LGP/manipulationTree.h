#pragma once

#include <Ors/ors.h>
#include <FOL/fol_mcts_world.h>
#include "LGP.h"

struct ManipulationTree_Node;
typedef mlr::Array<ManipulationTree_Node*> ManipulationTree_NodeL;

//===========================================================================

struct ManipulationTree_Node{
  ManipulationTree_Node *parent;
  mlr::Array<ManipulationTree_Node*> children;
  uint s;               ///< depth/step of this node
//  double t;             ///< real time

  FOL_World& fol;
  FOL_World::Handle decision;
  Graph *folState; ///< the symbolic state after action

  //  ors::KinematicSwitch sw; ///< the kinematic switch(es) that this action implies
  ors::KinematicWorld kinematics; ///< actual kinematics after action (includes all previous switches)

  //-- results of effective pose optimization
  ors::KinematicWorld effKinematics; ///< the effective kinematics (computed from kinematics and symbolic state)
  arr effPose;
  double effPoseCost;
  double effPoseReward;

  //-- results of full path optimization
  arr path;
  double pathCosts;

  ///root node init
  ManipulationTree_Node(LogicGeometricProgram& lgp)
    : parent(NULL), s(0), fol(lgp.fol_root), kinematics(lgp.world_root), effKinematics(kinematics), effPoseReward(0.){
    fol.generateStateTree=true;
    folState = fol.getState();
  }

  ///child node creation
  ManipulationTree_Node(ManipulationTree_Node *parent, FOL_World::Handle& a)
    : parent(parent), fol(parent->fol), kinematics(parent->kinematics), effKinematics(kinematics), effPoseReward(0.){
    s=parent->s+1;
    parent->children.append(this);
    fol.setState(parent->folState);
    if(a){
      fol.transition(a);
    }else{
      LOG(-1) <<"this doesn't make sense";
    }
    folState = fol.getState();
    decision = a;
  }

  void write(ostream& os=cout) const;

  void expand(){
    fol.setState(folState);
    auto actions = fol.get_actions();
    for(FOL_World::Handle& a:actions){
      cout <<"  DECISION: " <<*a <<endl;
      new ManipulationTree_Node(this, a);
    }
  }
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
