#include "manipulationTree.h"

ManipulationTree_Node::ManipulationTree_Node(ors::KinematicWorld& kin, FOL_World& _fol)
  : parent(NULL), s(0), fol(_fol),
    startKinematics(kin),
    kinematics(kin),
    effKinematics(kin),
    poseProblem(NULL), seqProblem(NULL), pathProblem(NULL),
    poseCost(0.), seqCost(0.), pathCost(0.), effPoseReward(0.){
  fol.generateStateTree=true;
  folState = fol.getState();
  folDecision = NULL;
  decision = NULL;
  hasEffKinematics = true;
}

ManipulationTree_Node::ManipulationTree_Node(ManipulationTree_Node* parent, MCTS_Environment::Handle& a)
  : parent(parent), fol(parent->fol),
    startKinematics(parent->startKinematics),
    kinematics(parent->kinematics),
//    effKinematics(parent->effKinematics),
    poseProblem(NULL), seqProblem(NULL), pathProblem(NULL),
    poseCost(0.), seqCost(0.), pathCost(0.), effPoseReward(0.){
  s=parent->s+1;
  parent->children.append(this);
  fol.setState(parent->folState);
  if(a){
    fol.transition(a);
    time=parent->time+fol.lastStepDuration;
    folReward = fol.lastStepReward;
  }else{
    LOG(-1) <<"this doesn't make sense";
  }
  folState = fol.getState();
  folDecision = fol.lastDecisionInState;
  decision = a;
}

void ManipulationTree_Node::expand(){
  CHECK(!isExpanded,"");
  fol.setState(folState);
  auto actions = fol.get_actions();
  for(FOL_World::Handle& a:actions){
    cout <<"  EXPAND DECISION: " <<*a <<endl;
    new ManipulationTree_Node(this, a);
  }
  isExpanded=true;
}

void ManipulationTree_Node::solvePoseProblem(){
  //reset the effective kinematics:
  CHECK(!parent || parent->hasEffKinematics,"parent needs to have computed the pose first!");
  if(parent) effKinematics = parent->effKinematics;

  poseProblem = new KOMO();
  KOMO& komo(*poseProblem);
  komo.setModel(effKinematics);
  komo.setTiming(1,1,5.,1);

  komo.setSquaredQVelocities();
  cout <<"  ** PoseProblem for state" <<*folState <<endl;
  komo.setAbstractTask(0, *folState);

  komo.reset();
  komo.MP->reportFull(true, FILE("z.problem"));
  komo.run();

  Graph result = komo.getReport();
  double cost = result.get<double>({"total","sqrCosts"});
  if(!pose.N || cost<poseCost){
    poseCost = cost;
    pose = komo.x;
  }
//  komo.displayTrajectory(-1.);


  effKinematics.setJointState(pose);

  for(ors::KinematicSwitch *sw: poseProblem->MP->switches)
    if(sw->timeOfApplication==1) sw->apply(effKinematics);
  effKinematics.topSort();
  effKinematics.checkConsistency();
  effKinematics.getJointState();
  hasEffKinematics = true;
}

void ManipulationTree_Node::solveSeqProblem(int verbose){
  if(!s) return;

  //-- create new problem declaration (within the KB)
//  Node *seqProblemNode = fol.KB.appendSubgraph({"SeqProblem"}, {folState->isNodeOfParentGraph});
//  seqProblemSpecs = &seqProblemNode->graph();

  //-- collect 'path nodes'
  ManipulationTree_NodeL treepath = getTreePath();

  seqProblem = new KOMO();
  KOMO& komo(*seqProblem);
  komo.setModel(startKinematics);
  komo.setTiming(time, 2, 5., 1, false);

  komo.setHoming(-1., -1., 1e-1); //gradient bug??
  komo.setSquaredQVelocities();
  komo.setSquaredFixJointVelocities(-1., -1., 1e3);
  komo.setSquaredFixSwitchVelocities(-1., -1., 1e3);

  for(ManipulationTree_Node *node:treepath){
    komo.setAbstractTask(node->time, *node->folState, true);
  }

  komo.reset();
//  komo.MP->reportFull(true, FILE("z.problem"));
  komo.run();
  komo.MP->reportFull(true, FILE("z.problem"));
//  komo.checkGradients();

  Graph result = komo.getReport();
  double cost = result.get<double>({"total","sqrCosts"});
  if(!seq.N || cost<seqCost){
    seqCost = cost;
    seq = komo.x;
  }
}

void ManipulationTree_Node::solvePathProblem(uint microSteps, int verbose){
//  Node *pathProblemNode = fol.KB.appendSubgraph({"PathProblem"}, {folState->isNodeOfParentGraph});
//  pathProblemSpecs = &pathProblemNode->graph();

  //-- collect 'path nodes'
  ManipulationTree_NodeL treepath = getTreePath();

  pathProblem = new KOMO();
  KOMO& komo(*pathProblem);
  komo.setModel(startKinematics);
  komo.setTiming(time, microSteps, 5., 2, false);

  komo.setHoming(-1., -1., 1e-2); //gradient bug??
  komo.setSquaredQAccelerations();
  komo.setSquaredFixJointVelocities(-1., -1., 1e3);
  komo.setSquaredFixSwitchVelocities(-1., -1., 1e3);

  for(ManipulationTree_Node *node:treepath) {
    komo.setAbstractTask(node->time, *node->folState, true);
  }

  komo.reset();
  komo.run();
  komo.MP->reportFull(true, FILE("z.problem"));

  Graph result = komo.getReport();
  double cost = result.get<double>({"total","sqrCosts"});
  if(!path.N || cost<pathCost){
    pathCost = cost;
    path = komo.x;
  }
//  komo.displayTrajectory(-1.);
}

ManipulationTree_NodeL ManipulationTree_Node::getTreePath(){
  ManipulationTree_NodeL path;
  ManipulationTree_Node *node=this;
  for(;node;){
    path.prepend(node);
    node = node->parent;
  }
  return path;
}

void ManipulationTree_Node::write(ostream& os) const{
  for(uint i=0;i<s+1;i++) os <<"--";
  if(decision) os <<" a= " <<*decision <<endl;
  else os <<" a=<ROOT>"<<endl;

  for(uint i=0;i<s+1;i++) os <<"  ";
  os <<" s= ";
  folState->write(os, " ");
  os <<endl;
  for(uint i=0;i<s+1;i++) os <<"  ";  os <<" depth=" <<s <<endl;
  for(uint i=0;i<s+1;i++) os <<"  ";  os <<" poseCost=" <<poseCost <<endl;
  for(uint i=0;i<s+1;i++) os <<"  ";  os <<" poseReward=" <<effPoseReward <<endl;
  for(uint i=0;i<s+1;i++) os <<"  ";  os <<" seqCost=" <<seqCost <<endl;
  for(uint i=0;i<s+1;i++) os <<"  ";  os <<" pathCost=" <<pathCost <<endl;
//  for(ManipulationTree_Node *n:children) n->write(os);
}

void ManipulationTree_Node::getGraph(Graph& G, Node* n) {
  if(!n){
    n = new Node_typed<bool>(G, {"a:<ROOT>"}, NodeL(), true);
  }else{
    n = new Node_typed<bool>(G, {STRING("a:"<<*decision)}, {n}, true);
  }
  graphIndex = n->index;
  n->keys.append(STRING("cPose:" <<poseCost));
  n->keys.append(STRING("cPath:" <<pathCost));
  n->keys.append(STRING("reward:" <<effPoseReward));
  for(ManipulationTree_Node *ch:children) ch->getGraph(G, n);
}

RUN_ON_INIT_BEGIN(manipulationTree)
ManipulationTree_NodeL::memMove = true;
RUN_ON_INIT_END(manipulationTree)
