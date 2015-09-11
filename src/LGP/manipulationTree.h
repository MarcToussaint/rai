#pragma once

#include <Ors/ors.h>

//===========================================================================

struct ManipulationTree_Node{
  ManipulationTree_Node *parent;
  MT::Array<ManipulationTree_Node*> children;
  uint s;               ///< depth/step of this node
//  double t;             ///< real time
//  action;           ///< what decision (relative to the parent) does this node represent

//  ors::KinematicSwitch sw; ///< the kinematic switch(es) that this action implies
  ors::KinematicWorld kinematics; ///< actual kinematics after action (includes all previous switches)
  Graph symbols; ///< the symbolic state after action

  //-- results of effective pose optimization
  ors::KinematicWorld effKinematics; ///< the effective kinematics (computed from kinematics and symbolic state)
  arr effPose;
  double effPoseCost;
  double effPoseReward;

  //-- results of full path optimization
  arr path;
  double pathCosts;

  ///root node init
  ManipulationTree_Node(const ors::KinematicWorld& world_root, const Graph& symbols_root)
    : parent(NULL), s(0), kinematics(world_root), symbols(symbols_root), effKinematics(kinematics), effPoseReward(0.){
  }

  ///child node creation
  ManipulationTree_Node(ManipulationTree_Node *parent)
    : parent(parent), kinematics(parent->kinematics), symbols(parent->symbols), effKinematics(kinematics), effPoseReward(0.){
    s=parent->s+1;
    parent->children.append(this);
  }
};
