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
  Graph *poseProblem;
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
    : parent(parent), fol(parent->fol), kinematics(parent->kinematics), effKinematics(parent->effKinematics), effPoseReward(0.){
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
      cout <<"  EXPAND DECISION: " <<*a <<endl;
      new ManipulationTree_Node(this, a);
    }
  }

  void solvePoseProblem(){
    Node *n = new Node_typed<Graph>(fol.KB, {"PoseProblem"}, {folState->isNodeOfParentGraph}, new Graph, true);
    poseProblem = &n->graph();
    poseProblem->copy(*folState, &fol.KB);
    NodeL komoRules = fol.KB.getNodes("EffectiveKinematicsRule");
    listWrite(komoRules, cout, "\n"); cout <<endl;
    forwardChaining_FOL(*poseProblem, komoRules/*, NULL, NoGraph, 5*/);
    cout <<"POSE PROBLEM:" <<*poseProblem <<endl;
//    KOMO komo(*poseProblem);
    MotionProblem problem(effKinematics, false);
    problem.setTiming(0, 1.);
    problem.k_order=0;
    problem.parseTasks(*poseProblem);
//    problem.featureReport();

    for(ors::KinematicSwitch *sw: problem.switches)
      if(sw->timeOfApplication==0) sw->apply(effKinematics);

    arr x=problem.x0;
    rndGauss(x, .1, true);
    OptConstrained opt(x, NoArr, problem.InvKinProblem(), OPT(verbose=0));
    opt.run();

//    problem.featureReport();
//    problem.costReport();
//    problem.world.gl().watch();
//    effKinematics.setJointState(problem.x0);

    for(ors::KinematicSwitch *sw: problem.switches)
      if(sw->timeOfApplication==1) sw->apply(effKinematics);
    effKinematics.topSort();
    effKinematics.checkConsistency();

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
