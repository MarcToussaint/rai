#include "manipulationTree.h"

ManipulationTree_Node::ManipulationTree_Node(LogicGeometricProgram& lgp)
  : parent(NULL), s(0), fol(lgp.fol_root), kinematics(lgp.world_root), effKinematics(kinematics), effPoseCost(0.), effPoseReward(0.), pathCost(0.){
  fol.generateStateTree=true;
  folState = fol.getState();
}

ManipulationTree_Node::ManipulationTree_Node(ManipulationTree_Node* parent, MCTS_Environment::Handle& a)
  : parent(parent), fol(parent->fol), kinematics(parent->kinematics), effKinematics(parent->effKinematics), effPoseCost(0.), effPoseReward(0.), pathCost(0.){
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

void ManipulationTree_Node::expand(){
  fol.setState(folState);
  auto actions = fol.get_actions();
  for(FOL_World::Handle& a:actions){
    cout <<"  EXPAND DECISION: " <<*a <<endl;
    new ManipulationTree_Node(this, a);
  }
}

void ManipulationTree_Node::solvePoseProblem(){
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
  //    problem.reportFull();

  for(ors::KinematicSwitch *sw: problem.switches)
    if(sw->timeOfApplication==0) sw->apply(effKinematics);

  arr x=problem.x0;
  rndGauss(x, .1, true);
  OptConstrained opt(x, NoArr, problem.InvKinProblem(), OPT(verbose=0));
  opt.run();
  effPoseCost = opt.newton.fx;

  problem.reportFull();
  problem.costReport();
  //    problem.world.gl().watch();
  //    effKinematics.setJointState(problem.x0);

  for(ors::KinematicSwitch *sw: problem.switches)
    if(sw->timeOfApplication==1) sw->apply(effKinematics);
  effKinematics.topSort();
  effKinematics.checkConsistency();

}

void ManipulationTree_Node::write(ostream& os) const{
  for(uint i=0;i<s+1;i++) os <<"--";
  if(decision) os <<" a= " <<*decision <<endl;
  else os <<" a= <ROOT>"<<endl;

  for(uint i=0;i<s+1;i++) os <<"  ";
  os <<" s= ";
  folState->write(os, " ");
  os <<endl;
  for(uint i=0;i<s+1;i++) os <<"  ";  os <<" poseCost=" <<effPoseCost <<endl;
  for(uint i=0;i<s+1;i++) os <<"  ";  os <<" poseReward=" <<effPoseReward <<endl;
  for(uint i=0;i<s+1;i++) os <<"  ";  os <<" pathCost=" <<pathCost <<endl;
  for(ManipulationTree_Node *n:children) n->write(os);
}

