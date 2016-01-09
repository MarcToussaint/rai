#include "manipulationTree.h"

ManipulationTree_Node::ManipulationTree_Node(LogicGeometricProgram& lgp)
  : lgp(lgp), parent(NULL), s(0), fol(lgp.fol_root),
    kinematics(lgp.world_root),
    effKinematics(lgp.world_root),
    seqProblem(lgp.world_root, false), pathProblem(lgp.world_root, false),
    poseCost(0.), seqCost(0.), pathCost(0.), effPoseReward(0.){
  fol.generateStateTree=true;
  folState = fol.getState();
  folDecision = NULL;
  decision = NULL;
}

ManipulationTree_Node::ManipulationTree_Node(LogicGeometricProgram& lgp, ManipulationTree_Node* parent, MCTS_Environment::Handle& a)
  : lgp(lgp), parent(parent), fol(parent->fol),
    kinematics(parent->kinematics),
    effKinematics(parent->effKinematics),
    seqProblem(lgp.world_root, false), pathProblem(lgp.world_root, false),
    poseCost(0.), seqCost(0.), pathCost(0.), effPoseReward(0.){
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

  arr x=poseProblem.getInitialization();
  rndGauss(x, .1, true);
  OptConstrained opt(x, NoArr, poseProblem.InvKinProblem(), OPT(verbose=0));
  opt.run();
  poseCost = opt.newton.fx;

  poseProblem.reportFull();
  poseProblem.costReport();
  //    problem.world.gl().watch();
  //    effKinematics.setJointState(problem.x0);

  for(ors::KinematicSwitch *sw: poseProblem.switches)
    if(sw->timeOfApplication==1) sw->apply(effKinematics);
  effKinematics.topSort();
  effKinematics.checkConsistency();
}

void ManipulationTree_Node::solveSeqProblem(){
  //-- create new problem declaration (within the KB)
  Node *seqProblemNode = new Node_typed<Graph>(fol.KB, {"SeqProblem"}, {folState->isNodeOfParentGraph}, new Graph, true);
  seqProblemSpecs = &seqProblemNode->graph();

  //-- collect 'path nodes'
  ManipulationTree_NodeL treepath = getTreePath();

  //-- add decisions to the seq pose problem description
  seqProblem.setTiming(s, 5.*s);
  seqProblem.k_order=1;
  NodeL komoRules = fol.KB.getNodes("SeqProblemRule");
//  listWrite(komoRules, cout, "\n"); cout <<endl;
  for(ManipulationTree_Node *node:treepath) if(node->folDecision){ //(e.g. the root may not have a decision)
    CHECK(node->s > 0,""); //don't add anything for the root
    Graph tmp(*node->folState);
    Graph changes(fol.KB, {}, {});
    forwardChaining_FOL(tmp, komoRules, NULL, changes); //use the rules to add to the specs
    changes.checkConsistency();
    for(Node *n:changes){
      Graph *p;
      arr *t;
      double *tt;
      if((p=n->getValue<Graph>())){
        if((t=p->getValue<arr>("time"))) *t += (double)(node->s-1);
        if((tt=p->getValue<double>("time"))) *tt += (double)(node->s-1);
      }
    }
    seqProblemSpecs->copy(changes, NULL, true);
//    cout <<"SEQ PROBLEM: (s=" <<node->s <<")\n" <<*seqProblemSpecs <<endl;
  }

  cout <<"SEQ PROBLEM symbolic:\n" <<*seqProblemSpecs <<endl;
  seqProblem.parseTasks(*seqProblemSpecs, 1, 0);
  arr seq = seqProblem.getInitialization();
  cout <<"SEQ PROBLEM motion problem:\n";
  seqProblem.reportFull(true);
  rndGauss(seq, .1, true);

  Convert cvt(seqProblem);

  checkJacobianCP(cvt, seq, 1e-4);
  checkHessianCP(cvt, seq, 1e-4);
//  exit(0);

  if(!seqProblem.dim_g_h()){
    OptNewton opt(seq, cvt, OPT(verbose=2));
    opt.run();
    seqCost = opt.fx;
  }else{
    OptConstrained opt(seq, NoArr, cvt, OPT(verbose=0));
    opt.run();
    seqCost = opt.newton.fx;
  }

//  seqProblem.reportFull(true);
  seqProblem.costReport();
  seqProblem.displayTrajectory(1, "SeqProblem", -.01);
}

void ManipulationTree_Node::solvePathProblem(uint microSteps){
  Node *pathProblemNode = new Node_typed<Graph>(fol.KB, {"PathProblem"}, {folState->isNodeOfParentGraph}, new Graph, true);
  pathProblemSpecs = &pathProblemNode->graph();

  //-- collect 'path nodes'
  ManipulationTree_NodeL treepath = getTreePath();

  //-- add decisions to the path problem description
  pathProblem.setTiming(s*microSteps, 5.*s);
  pathProblem.k_order=2;
  NodeL komoRules = fol.KB.getNodes("PathProblemRule");
//  listWrite(komoRules, cout, "\n"); cout <<endl;
  for(ManipulationTree_Node *node:treepath) if(node->folDecision){
    CHECK(node->s > 0,"");
    node->folDecision->newClone(*pathProblemSpecs);
    forwardChaining_FOL(*pathProblemSpecs, komoRules); //, NULL, NoGraph, 4);
    pathProblem.parseTasks(*pathProblemSpecs, microSteps, (node->s-1)*microSteps);
    cout <<"PATH PROBLEM: (s=" <<node->s <<")\n" <<*pathProblemSpecs <<endl;
    pathProblemSpecs->clear();
  }

  //    KOMO komo(*pathProblem);
  pathProblem.reportFull(true);

  arr x = pathProblem.getInitialization();
  rndGauss(x, .1, true);
  if(!pathProblem.dim_g_h()){
    optNewton(x, Convert(pathProblem), OPT(verbose=2, stopIters=100, maxStep=.1, stepInc=1.1, stepDec=0.7 , damping=1., allowOverstep=true));
//    OptNewton opt(x, Convert(MPF), OPT(verbose=0));
//    opt.run();
    path = x;
//    pathCost = opt.fx;
  }else{
    OptConstrained opt(x, NoArr, Convert(pathProblem), OPT(verbose=0));
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
  for(uint i=0;i<s+1;i++) os <<"  ";  os <<" pathCost=" <<pathCost <<endl;
  for(ManipulationTree_Node *n:children) n->write(os);
}

void ManipulationTree_Node::getGraph(Graph& G, Node* n) const{
  if(!n){
    n = new Node_typed<bool>(G, {"a:<ROOT>"}, NodeL(), NULL, false);
  }else{
    n = new Node_typed<bool>(G, {STRING("a:"<<*decision)}, {n}, NULL, false);
  }
  n->keys.append(STRING("cPose:" <<poseCost));
  n->keys.append(STRING("cPath:" <<pathCost));
  n->keys.append(STRING("reward:" <<effPoseReward));
  for(const ManipulationTree_Node *ch:children) ch->getGraph(G, n);
}

RUN_ON_INIT_BEGIN(manipulationTree)
ManipulationTree_NodeL::memMove = true;
RUN_ON_INIT_END(manipulationTree)
