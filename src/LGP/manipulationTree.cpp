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

#if 0
  if(true || !poseProblem){ //create the pose problem
    Node *n = fol.KB.appendSubgraph({"PoseProblem"}, {folState->isNodeOfParentGraph});
    poseProblemSpecs = &n->graph();
    poseProblemSpecs->copy(*folState, &fol.KB);
    NodeL komoRules = fol.KB.getNodes("PoseProblemRule");  //  listWrite(komoRules, cout, "\n"); cout <<endl;
    forwardChaining_FOL(*poseProblemSpecs, komoRules/*, NULL, NoGraph, 5*/);
    cout <<"POSE PROBLEM:" <<*poseProblemSpecs <<endl;

    poseProblem = new MotionProblem(effKinematics, true);
    poseProblem->setTiming(0, 1.);
    poseProblem->k_order=0;
    poseProblem->parseTasks(*poseProblemSpecs);
    //    Problem->reportFull();
  }

  for(ors::KinematicSwitch *sw: poseProblem->switches)
    if(sw->timeOfApplication==0) sw->apply(effKinematics);

  arr newPose=poseProblem->getInitialization();
  rndGauss(newPose, .1, true);
  OptConstrained opt(newPose, NoArr, poseProblem->InvKinProblem(), OPT(verbose=2));
  opt.run();
  //  poseProblem->reportFull();
  poseProblem->costReport(false);
  //    Problem->world.gl().watch();

  if(!pose.N || opt.newton.fx<poseCost){
    poseCost = opt.newton.fx;
    pose = newPose;
  }
#else
  poseProblem = new KOMO();
  KOMO& komo(*poseProblem);
  poseProblem->setModel(effKinematics);
  poseProblem->setTiming(1,1,5.,1);

  komo.setSquaredQVelocities();
  cout <<"  ** PoseProblem for state" <<*folState <<endl;
  poseProblem->setAbstractTask(0, *folState);

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
#endif


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
  Node *seqProblemNode = fol.KB.appendSubgraph({"SeqProblem"}, {folState->isNodeOfParentGraph});
  seqProblemSpecs = &seqProblemNode->graph();

  //-- collect 'path nodes'
  ManipulationTree_NodeL treepath = getTreePath();

#if 0
  //-- add decisions to the seq pose problem description
  seqProblem = new MotionProblem(startKinematics, true);
  seqProblem->setTiming(s-1, 5.*s); //T=0 means one pose is optimized!!
  seqProblem->k_order=1;
  NodeL komoRules = fol.KB.getNodes("SeqProblemRule");
//  listWrite(komoRules, cout, "\n"); cout <<endl;
  for(ManipulationTree_Node *node:treepath) if(node->folDecision){ //(e.g. the root may not have a decision)
    CHECK(node->s > 0,""); //don't add anything for the root
    Graph tmp(*node->folState);
    Graph& changes = fol.KB.appendSubgraph({}, {})->graph();
    forwardChaining_FOL(tmp, komoRules, NULL, changes); //use the rules to add to the specs
    changes.checkConsistency();
    for(Node *n:changes){
      Graph *p;
      arr *t;
      double *tt;
      if((p=&n->graph())){
        if((t=p->find<arr>("time"))) *t += (double)(node->s)-2.;
        if((tt=p->find<double>("time"))) *tt += (double)(node->s)-2.;
      }
    }
    seqProblemSpecs->copy(changes, true);
    delete changes.isNodeOfParentGraph;
//    cout <<"SEQ PROBLEM: (s=" <<node->s <<")\n" <<*seqProblemSpecs <<endl;
  }

  cout <<"SEQ PROBLEM symbolic:\n" <<*seqProblemSpecs <<endl;
  seqProblem->parseTasks(*seqProblemSpecs, 1, 0);
  arr newSeq = seqProblem->getInitialization();
  cout <<"SEQ PROBLEM motion problem:\n";
  seqProblem->reportFull(true);
  rndGauss(newSeq, .1, true);

  Convert cvt(*seqProblem);

//  checkJacobianCP(cvt, seq, 1e-4);
//  checkHessianCP(cvt, seq, 1e-4);
//  exit(0);

  double newCost;
  if(!seqProblem->dim_g_h()){
    OptNewton opt(newSeq, cvt, OPT(verbose=2));
    opt.run();
    newCost = opt.fx;
  }else{
    OptConstrained opt(newSeq, NoArr, cvt, OPT(verbose=0));
    opt.run();
    newCost = opt.newton.fx;
  }

  if(!seq.N || newCost < seqCost){
    seqCost = newCost;
    seq=newSeq;
  }

//  seqProblem->reportFull(true);
  seqProblem->costReport(verbose>1);
  if(verbose>1) seqProblem->displayTrajectory(1, "SeqProblem", -.01);
#else
  NIY
#endif
}

void ManipulationTree_Node::solvePathProblem(uint microSteps, int verbose){
  Node *pathProblemNode = fol.KB.appendSubgraph({"PathProblem"}, {folState->isNodeOfParentGraph});
  pathProblemSpecs = &pathProblemNode->graph();

  //-- collect 'path nodes'
  ManipulationTree_NodeL treepath = getTreePath();

#if 0
  //-- add decisions to the path problem description
  pathProblem = new MotionProblem(startKinematics, true);
  pathProblem->setTiming(s*microSteps, 5.*s);
  pathProblem->k_order=2;
  NodeL komoRules = fol.KB.getNodes("PathProblemRule");
//  listWrite(komoRules, cout, "\n"); cout <<endl;
  for(ManipulationTree_Node *node:treepath) if(node->folDecision){
    CHECK(node->s > 0,"");
    node->folDecision->newClone(*pathProblemSpecs);
    forwardChaining_FOL(*pathProblemSpecs, komoRules); //, NULL, NoGraph, 4);
    pathProblem->parseTasks(*pathProblemSpecs, microSteps, (node->s-1)*microSteps);
    cout <<"PATH PROBLEM: (s=" <<node->s <<")\n" <<*pathProblemSpecs <<endl;
    pathProblemSpecs->clear();
  }

  path = pathProblem->getInitialization();
  pathProblem->reportFull(true);
  rndGauss(path, .1, true);

  Convert cvt(*pathProblem);

  if(!pathProblem->dim_g_h()){
//    optNewton(path, cvt, OPT(verbose=2, stopIters=100, maxStep=.1, stepInc=1.1, stepDec=0.7 , damping=1., allowOverstep=true));
    OptNewton opt(path, cvt, OPT(verbose=2));
    opt.run();
    pathCost = opt.fx;
  }else{
    OptConstrained opt(path, NoArr, cvt, OPT(verbose=0));
    opt.run();
    pathCost = opt.newton.fx;
  }

//  pathProblem->reportFull(true);
  pathProblem->costReport(verbose>1);
  if(verbose>1) pathProblem->displayTrajectory(1, "PathProblem", -.01);
#else
  pathProblem = new KOMO();
  KOMO& komo(*pathProblem);
  komo.setModel(startKinematics);
  komo.setTiming(s+1, microSteps, 5., 2);

  komo.setSquaredQAccelerations();

  for(ManipulationTree_Node *node:treepath) /*if(node->folDecision)*/{
//    CHECK(node->s > 0,"");
    komo.setAbstractTask(node->s, *node->folState);
  }

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
#endif
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
