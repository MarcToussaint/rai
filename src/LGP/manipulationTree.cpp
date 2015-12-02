#include "manipulationTree.h"

ManipulationTree_Node::ManipulationTree_Node(LogicGeometricProgram& lgp)
  : lgp(lgp), parent(NULL), s(0), fol(lgp.fol_root),
    kinematics(lgp.world_root),
    effKinematics(lgp.world_root), effPoseCost(0.), effPoseReward(0.),
    pathProblem(lgp.world_root, false), pathCost(0.){
  fol.generateStateTree=true;
  folState = fol.getState();
  folDecision = NULL;
  decision = NULL;
}

ManipulationTree_Node::ManipulationTree_Node(LogicGeometricProgram& lgp, ManipulationTree_Node* parent, MCTS_Environment::Handle& a)
  : lgp(lgp), parent(parent), fol(parent->fol),
    kinematics(parent->kinematics),
    effKinematics(parent->effKinematics), effPoseCost(0.), effPoseReward(0.),
    pathProblem(lgp.world_root, false), pathCost(0.){
  s=parent->s+1;
  parent->children.append(this);
  fol.setState(parent->folState);
  if(a){
    fol.transition(a);
  }else{
    LOG(-1) <<"this doesn't make sense";
  }
  folState = fol.getState();
  folDecision = fol.lastDecisionInState;
  decision = a;
}

void ManipulationTree_Node::expand(){
  fol.setState(folState);
  auto actions = fol.get_actions();
  for(FOL_World::Handle& a:actions){
    cout <<"  EXPAND DECISION: " <<*a <<endl;
    new ManipulationTree_Node(lgp, this, a);
  }
}

void ManipulationTree_Node::solvePoseProblem(){
  Node *n = new Node_typed<Graph>(fol.KB, {"PoseProblem"}, {folState->isNodeOfParentGraph}, new Graph, true);
  poseProblemSpecs = &n->graph();
  poseProblemSpecs->copy(*folState, &fol.KB);
  NodeL komoRules = fol.KB.getNodes("EffectiveKinematicsRule");
//  listWrite(komoRules, cout, "\n"); cout <<endl;
  forwardChaining_FOL(*poseProblemSpecs, komoRules/*, NULL, NoGraph, 5*/);
  cout <<"POSE PROBLEM:" <<*poseProblemSpecs <<endl;
  //    KOMO komo(*poseProblem);

  MotionProblem poseProblem(effKinematics, false);
  poseProblem.setTiming(0, 1.);
  poseProblem.k_order=0;
  poseProblem.parseTasks(*poseProblemSpecs);
  //    problem.reportFull();

  for(ors::KinematicSwitch *sw: poseProblem.switches)
    if(sw->timeOfApplication==0) sw->apply(effKinematics);

  arr x=poseProblem.x0;
  rndGauss(x, .1, true);
  OptConstrained opt(x, NoArr, poseProblem.InvKinProblem(), OPT(verbose=0));
  opt.run();
  effPoseCost = opt.newton.fx;

  poseProblem.reportFull();
  poseProblem.costReport();
  //    problem.world.gl().watch();
  //    effKinematics.setJointState(problem.x0);

  for(ors::KinematicSwitch *sw: poseProblem.switches)
    if(sw->timeOfApplication==1) sw->apply(effKinematics);
  effKinematics.topSort();
  effKinematics.checkConsistency();
}

void ManipulationTree_Node::solvePathProblem(uint microSteps){
  Node *pathProblemNode = new Node_typed<Graph>(fol.KB, {"PathProblem"}, {folState->isNodeOfParentGraph}, new Graph, true);
  pathProblemSpecs = &pathProblemNode->graph();

  //-- collect 'path nodes'
  ManipulationTree_NodeL pathnodes;
  ManipulationTree_Node *node=this;
  for(;node;){
    pathnodes.prepend(node);
    node = node->parent;
  }

  //-- add decisions to the path problem description
  pathProblem.setTiming(s*microSteps, 5.*s);
  pathProblem.k_order=2;
  NodeL komoRules = fol.KB.getNodes("PathProblemRule");
//  listWrite(komoRules, cout, "\n"); cout <<endl;
  for(ManipulationTree_Node *node:pathnodes) if(node->folDecision){
    CHECK(node->s > 0,"");
    node->folDecision->newClone(*pathProblemSpecs);
    forwardChaining_FOL(*pathProblemSpecs, komoRules); //, NULL, NoGraph, 4);
    pathProblem.parseTasks(*pathProblemSpecs, microSteps, (node->s-1)*microSteps);
    cout <<"PATH PROBLEM: (s=" <<node->s <<")\n" <<*pathProblemSpecs <<endl;
    pathProblemSpecs->clear();
  }

  //    KOMO komo(*pathProblem);
  pathProblem.reportFull(true);

  arr x = replicate(pathProblem.x0, pathProblem.T+1); //we initialize with a constant trajectory!
  rndGauss(x, .1, true);
  MotionProblemFunction MPF(pathProblem);
  if(!MPF.dim_g_h()){
    optNewton(x, Convert(MPF), OPT(verbose=2, stopIters=100, maxStep=.1, stepInc=1.1, stepDec=0.7 , damping=1., allowOverstep=true));
//    OptNewton opt(x, Convert(MPF), OPT(verbose=0));
//    opt.run();
    path = x;
//    pathCost = opt.fx;
  }else{
    OptConstrained opt(x, NoArr, Convert(MPF), OPT(verbose=0));
    opt.run();
    path = x;
    pathCost = opt.newton.fx;
  }

  pathProblem.reportFull(true);
  pathProblem.costReport();
  pathProblem.displayTrajectory(1, "PathProblem", -.01);

//  for(ors::KinematicSwitch *sw: pathProblem.switches)
//    if(sw->timeOfApplication==pathProblem.T+1) sw->apply(effKinematics);
}

void ManipulationTree_Node::write(ostream& os) const{
  for(uint i=0;i<s+1;i++) os <<"--";
  if(decision) os <<" a= " <<*decision <<endl;
  else os <<" a= <ROOT>"<<endl;

  for(uint i=0;i<s+1;i++) os <<"  ";
  os <<" s= ";
  folState->write(os, " ");
  os <<endl;
  for(uint i=0;i<s+1;i++) os <<"  ";  os <<" depth=" <<s <<endl;
  for(uint i=0;i<s+1;i++) os <<"  ";  os <<" poseCost=" <<effPoseCost <<endl;
  for(uint i=0;i<s+1;i++) os <<"  ";  os <<" poseReward=" <<effPoseReward <<endl;
  for(uint i=0;i<s+1;i++) os <<"  ";  os <<" pathCost=" <<pathCost <<endl;
  for(ManipulationTree_Node *n:children) n->write(os);
}



RUN_ON_INIT_BEGIN(manipulationTree)
ManipulationTree_NodeL::memMove = true;
RUN_ON_INIT_END(manipulationTree)
