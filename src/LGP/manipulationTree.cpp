#include "manipulationTree.h"
#include <MCTS/solver_PlainMC.h>

#define DEBUG(x) x
#define DEL_INFEASIBLE(x) //x

uint COUNT_kin=0;
uint COUNT_evals=0;
uint COUNT_poseOpt=0;
uint COUNT_seqOpt=0;
uint COUNT_pathOpt=0;

ManipulationTree_Node::ManipulationTree_Node(ors::KinematicWorld& kin, FOL_World& _fol)
  : parent(NULL), s(0),
    fol(_fol), folState(NULL), folDecision(NULL), folReward(0.), folAddToState(NULL),
    startKinematics(kin), effKinematics(),
    rootMC(NULL), mcStats(NULL),
    poseProblem(NULL), seqProblem(NULL), pathProblem(NULL),
    symCost(0.), poseCost(0.), poseConstraints(0.), seqCost(0.), seqConstraints(0.), pathCost(0.), pathConstraints(0.), effPoseReward(0.), costSoFar(0.),
    poseFeasible(false), seqFeasible(false), pathFeasible(false),
    inFringe1(false), inFringe2(false) {
  //this is the root node!
  fol.reset_state();
  folState = fol.createStateCopy();
}

ManipulationTree_Node::ManipulationTree_Node(ManipulationTree_Node* parent, MCTS_Environment::Handle& a)
  : parent(parent), fol(parent->fol),
     folState(NULL), folDecision(NULL), folReward(0.), folAddToState(NULL),
    startKinematics(parent->startKinematics), effKinematics(),
    rootMC(NULL), mcStats(NULL),
    poseProblem(NULL), seqProblem(NULL), pathProblem(NULL),
    symCost(0.), poseCost(0.), poseConstraints(0.), seqCost(0.), seqConstraints(0.), pathCost(0.), pathConstraints(0.), effPoseReward(0.), costSoFar(0.),
    poseFeasible(false), seqFeasible(false), pathFeasible(false),
    inFringe1(false), inFringe2(false) {
  s=parent->s+1;
  parent->children.append(this);
  fol.setState(parent->folState, parent->s);
  CHECK(a,"giving a 'NULL' shared pointer??");
  fol.transition(a);
  time=parent->time+fol.lastStepDuration;
  folReward = fol.lastStepReward;
  isTerminal = fol.successEnd;
  if(fol.deadEnd) isInfeasible=true;
  folState = fol.createStateCopy();
  folAddToState = NULL; //fresh creation -> notion to add
  folDecision = folState->getNode("decision");
  decision = a;
}

void ManipulationTree_Node::expand(){
  CHECK(!isExpanded,"");
  fol.setState(folState, s);
  auto actions = fol.get_actions();
  for(FOL_World::Handle& a:actions){
//    cout <<"  EXPAND DECISION: " <<*a <<endl;
    new ManipulationTree_Node(this, a);
  }
  if(!children.N) isTerminal=true;
  isExpanded=true;
}

arr ManipulationTree_Node::generateRootMCRollouts(uint num, int stepAbort, const mlr::Array<MCTS_Environment::Handle>& prefixDecisions){
  CHECK(!parent, "generating rollouts needs to be done by the root only");

  fol.reset_state();
//  cout <<"********\n *** MC from STATE=" <<*fol.state->isNodeOfParentGraph <<endl;
  if(!rootMC){
    rootMC = new PlainMC(fol);
    rootMC->verbose = 0;
  }

  arr R;

  for(uint k=0;k<num;k++){
    R.append( rootMC->generateRollout(stepAbort, prefixDecisions) );
  }

  return R;
}

void ManipulationTree_Node::addMCRollouts(uint num, int stepAbort){
  //-- collect decision path
  ManipulationTree_NodeL treepath = getTreePath();
  mlr::Array<MCTS_Environment::Handle> prefixDecisions(treepath.N-1);
  for(uint i=1;i<treepath.N;i++)
    prefixDecisions(i-1) = treepath(i)->decision;

//  cout <<"DECISION PATH = "; listWrite(prefixDecisions); cout <<endl;

  arr R = treepath.first()->generateRootMCRollouts(num, stepAbort, prefixDecisions);

  for(ManipulationTree_Node* n:treepath){
    if(!n->mcStats) n->mcStats = new MCStatistics;
    for(auto& r:R){
      n->mcStats->add(r);
      n->symCost = - n->mcStats->X.first();
    }
  }

//  mcStats->report();
//  auto a = rootMC->getBestAction();
//  cout <<"******** BEST ACTION " <<*a <<endl;
}

void ManipulationTree_Node::solvePoseProblem(){

  //reset the effective kinematics:
  if(parent && !parent->effKinematics.q.N){
    MLR_MSG("parent needs to have computed the pose first!");
    return;
  }
  if(!parent) effKinematics = startKinematics;
  else effKinematics = parent->effKinematics;

  //-- collect 'path nodes'
  ManipulationTree_NodeL treepath = getTreePath();

  poseProblem = new KOMO();
  KOMO& komo(*poseProblem);
  komo.setModel(effKinematics);
  komo.setTiming(1., 2, 5., 1, false);

  komo.setHoming(-1., -1., 1e-1); //gradient bug??
  komo.setSquaredQVelocities();
//  komo.setSquaredFixJointVelocities(-1., -1., 1e3);
  komo.setSquaredFixSwitchVelocities(-1., -1., 1e3);

  komo.setAbstractTask(0., *folState, true);
//  for(ors::KinematicSwitch *sw: poseProblem->MP->switches){
//    sw->timeOfApplication=2;
//  }

  DEBUG( FILE("z.fol") <<fol; )
  DEBUG( komo.MP->reportFull(true, FILE("z.problem")); )
  komo.reset();
  try{
    komo.run();
  } catch(const char* msg){
    cout <<"KOMO FAILED: " <<msg <<endl;
  }
  COUNT_evals += komo.opt->newton.evals;
  COUNT_kin += ors::KinematicWorld::setJointStateCount;
  COUNT_poseOpt++;

  DEBUG( komo.MP->reportFull(true, FILE("z.problem")); )

  Graph result = komo.getReport();
  DEBUG( FILE("z.problem.cost") <<result; )
  double cost = result.get<double>({"total","sqrCosts"});
  double constraints = result.get<double>({"total","constraints"});

  if(!pose.N || cost<poseCost){
    poseCost = cost;
    poseConstraints = constraints;
    poseFeasible = (constraints<.5);
    pose = komo.x;
  }

  if(!poseFeasible)
    labelInfeasible();

  effKinematics = *poseProblem->MP->configurations.last();

  for(ors::KinematicSwitch *sw: poseProblem->MP->switches){
//    CHECK_EQ(sw->timeOfApplication, 1, "need to do this before the optimization..");
    if(sw->timeOfApplication>=2) sw->apply(effKinematics);
  }
  effKinematics.topSort();
  DEBUG( effKinematics.checkConsistency(); )
  effKinematics.getJointState();
}

void ManipulationTree_Node::solveSeqProblem(int verbose){

  if(!s){ seqFeasible=true; return; } //there is no sequence to compute

  //-- collect 'path nodes'
  ManipulationTree_NodeL treepath = getTreePath();

  seqProblem = new KOMO();
  KOMO& komo(*seqProblem);
  komo.setModel(startKinematics);
  komo.setTiming(time+1., 2, 5., 1, false); //really + 1. phase??

  komo.setHoming(-1., -1., 1e-1); //gradient bug??
  komo.setSquaredQVelocities();
  komo.setSquaredFixJointVelocities(-1., -1., 1e3);
  komo.setSquaredFixSwitchVelocities(-1., -1., 1e3);

  for(ManipulationTree_Node *node:treepath){
    komo.setAbstractTask((node->parent?node->parent->time:0.), *node->folState, true);
  }

  DEBUG( FILE("z.fol") <<fol; )
  DEBUG( komo.MP->reportFull(true, FILE("z.problem")); )
  komo.reset();
  try{
    komo.run();
  } catch(const char* msg){
    cout <<"KOMO FAILED: " <<msg <<endl;
  }
  COUNT_evals += komo.opt->newton.evals;
  COUNT_kin += ors::KinematicWorld::setJointStateCount;
  COUNT_seqOpt++;

  DEBUG( komo.MP->reportFull(true, FILE("z.problem")); )
//  komo.checkGradients();

  Graph result = komo.getReport();
  DEBUG( FILE("z.problem.cost") <<result; )
  double cost = result.get<double>({"total","sqrCosts"});
  double constraints = result.get<double>({"total","constraints"});

  if(!seq.N || cost<seqCost){
    seqCost = cost;
    seqConstraints = constraints;
    seqFeasible = (constraints<.5);
    seq = komo.x;
  }

  if(!seqFeasible)
    labelInfeasible();
}

void ManipulationTree_Node::solvePathProblem(uint microSteps, int verbose){

  if(!s){ pathFeasible=true; return; } //there is no path to compute

  //-- collect 'path nodes'
  ManipulationTree_NodeL treepath = getTreePath();

  pathProblem = new KOMO();
  KOMO& komo(*pathProblem);
  komo.setModel(startKinematics);
  komo.setTiming(time+1., microSteps, 5., 2, false);

  komo.setHoming(-1., -1., 1e-2); //gradient bug??
  komo.setSquaredQAccelerations();
  komo.setSquaredFixJointVelocities(-1., -1., 1e3);
  komo.setSquaredFixSwitchVelocities(-1., -1., 1e3);

  for(ManipulationTree_Node *node:treepath){
    komo.setAbstractTask((node->parent?node->parent->time:0.), *node->folState, true);
  }

  DEBUG( FILE("z.fol") <<fol; )
  DEBUG( komo.MP->reportFull(true, FILE("z.problem")); )
  komo.reset();
  try{
    komo.run();
  } catch(const char* msg){
    cout <<"KOMO FAILED: " <<msg <<endl;
  }
  COUNT_evals += komo.opt->newton.evals;
  COUNT_kin += ors::KinematicWorld::setJointStateCount;
  COUNT_pathOpt++;

  DEBUG( komo.MP->reportFull(true, FILE("z.problem")); )
//  komo.checkGradients();

  Graph result = komo.getReport();
  DEBUG( FILE("z.problem.cost") <<result; )
  double cost = result.get<double>({"total","sqrCosts"});
  double constraints = result.get<double>({"total","constraints"});

  if(!path.N || cost<pathCost){
    pathCost = cost;
    pathConstraints = constraints;
    pathFeasible = (constraints<.5);
    path = komo.x;
  }

  if(!pathFeasible)
    labelInfeasible();
}

void ManipulationTree_Node::labelInfeasible(){
  isInfeasible = true;

  //-- remove children
//  ManipulationTree_NodeL tree;
//  getAllChildren(tree);
//  for(ManipulationTree_Node *n:tree) if(n!=this) delete n; //TODO: memory leak!
  DEL_INFEASIBLE( children.clear(); )

  //-- add INFEASIBLE flag to fol
  NodeL symbols = folDecision->parents;
  symbols.prepend( fol.KB.getNode({"INFEASIBLE"}));

//  cout <<"\n *** LABELLING INFEASIBLE: "; listWrite(symbols); cout <<endl;
  //-- find the right parent...
  ManipulationTree_Node* node = this;
  while(node->parent){
    bool stop=false;
    for(Node *fact:node->folState->list()){
      if(fact->keys.N && fact->keys.last()=="block"){
        if(tuplesAreEqual(fact->parents, symbols)){
          CHECK(fact->isOfType<bool>() && fact->keys.first()=="block", "");
          stop=true;
          break;
        }
      }
    }
    if(stop) break;
    node = node->parent;
  }

  if(!node->folAddToState){
    node->folAddToState = &fol.KB.newSubgraph({"ADD"}, {node->folState->isNodeOfParentGraph})->value;
  }
  new Node_typed<bool>(*node->folAddToState, {}, symbols, true);

//  ManipulationTree_Node *root=getRoot();
  node->recomputeAllFolStates();
  node->recomputeAllMCStats();

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

ManipulationTree_Node* ManipulationTree_Node::getRoot(){
  ManipulationTree_Node* n=this;
  while(n->parent) n=n->parent;
  return n;
}

void ManipulationTree_Node::getAllChildren(ManipulationTree_NodeL& tree){
  for(ManipulationTree_Node* c:children) c->getAllChildren(tree);
  tree.append(this);
}

ManipulationTree_Node *ManipulationTree_Node::treePolicy_random(){
  if(isInfeasible) return NULL;
  if(isTerminal) return NULL;
  if(children.N) return children.rndElem()->treePolicy_random();
  return this;
}

ManipulationTree_Node *ManipulationTree_Node::treePolicy_softMax(double temperature){
  if(isInfeasible) return NULL;
  if(isTerminal){
//    LOG(0) <<"stuck at terminal:" <<*this <<endl;
    return NULL;
  }
  if(children.N){
    arr Q(children.N);
    for(uint i=0;i<children.N;i++) Q(i) = children.elem(i)->symCost;
//    rndGauss(Q, .1, true);
    Q *= -temperature;
    Q = exp(Q);
    normalizeDist(Q);
    uint best = sampleMultinomial(Q); //.maxIndex();
    return children.elem(best)->treePolicy_softMax(temperature);
  }
  return this;
}

bool ManipulationTree_Node::recomputeAllFolStates(){
  if(!parent){ //this is root
    folState->copy(*fol.start_state);
    if(folAddToState) applyEffectLiterals(*folState, *folAddToState, {}, NULL);
  }else{
    fol.setState(parent->folState, parent->s);
    if(fol.is_feasible_action(decision)){
      fol.transition(decision);
      time=parent->time+fol.lastStepDuration;
      folReward = fol.lastStepReward;
      isTerminal = fol.successEnd;
      if(fol.deadEnd){
        if(!seqFeasible && !pathFeasible) //seq or path have already proven it feasible! Despite the logic...
          isInfeasible=true;
        return false;
      }
      folState->copy(*fol.state);
      if(folAddToState) applyEffectLiterals(*folState, *folAddToState, {}, NULL);
      folDecision = folState->getNode("decision");
    }else{
      if(!seqFeasible && !pathFeasible) //seq or path have already proven it feasible! Despite the logic...
        isInfeasible=true;
      return false;
    }
  }
  if(children.N){
    for(uint i=children.N-1;;){
      bool feasible = children(i)->recomputeAllFolStates();
      DEL_INFEASIBLE( if(!feasible) children.remove(i); )
      if(!i || !children.N) break;
      i--;
    }
  }
  return true;
}

void ManipulationTree_Node::recomputeAllMCStats(bool excludeLeafs){
  if(!mcStats) return;
  if(children.N || !excludeLeafs || isInfeasible)
    mcStats->clear();
  for(ManipulationTree_Node* ch:children){
    ch->recomputeAllMCStats();
    for(double x:ch->mcStats->X) mcStats->add(x);
  }
  if(mcStats->n)
    symCost = - mcStats->X.first();
  else
    symCost = 100.;
}

void ManipulationTree_Node::checkConsistency(){
  //-- check that the state->parent points to the parent's state
  if(parent){
    CHECK_EQ(parent->folState->isNodeOfParentGraph, folState->isNodeOfParentGraph->parents.scalar(), "");
    CHECK_EQ(&folDecision->container, folState, "");
  }

  //-- check that each child exactly matches a decision, in same order
  if(children.N){
    fol.setState(folState, s);
    auto actions = fol.get_actions();
    CHECK_EQ(children.N, actions.size(), "");
    uint i=0;
    for(FOL_World::Handle& a:actions){
//      cout <<"  DECISION: " <<*a <<endl;
      FOL_World::Handle& b = children(i)->decision;
      CHECK_EQ(*a, *b, "children do not match decisions");
      i++;
    }
  }

  for(auto* ch:children) ch->checkConsistency();
}

void ManipulationTree_Node::write(ostream& os, bool recursive) const{
  os <<"------- NODE -------\ns=" <<s <<" t=" <<time;
  if(decision) os <<" a= " <<*decision <<endl;
  else os <<" a=<ROOT>"<<endl;

  for(uint i=0;i<s+1;i++) os <<"  ";
  os <<" state= " <<*folState->isNodeOfParentGraph <<endl;
  for(uint i=0;i<s+1;i++) os <<"  ";  os <<" depth=" <<s <<endl;
  for(uint i=0;i<s+1;i++) os <<"  ";  os <<" poseCost=" <<poseCost <<endl;
  for(uint i=0;i<s+1;i++) os <<"  ";  os <<" poseReward=" <<effPoseReward <<endl;
  for(uint i=0;i<s+1;i++) os <<"  ";  os <<" seqCost=" <<seqCost <<endl;
  for(uint i=0;i<s+1;i++) os <<"  ";  os <<" pathCost=" <<pathCost <<endl;
  if(recursive) for(ManipulationTree_Node *n:children) n->write(os);
}

void ManipulationTree_Node::getGraph(Graph& G, Node* n) {
  if(!n){
    n = new Node_typed<bool>(G, {"a:<ROOT>"}, NodeL(), true);
  }else{
    n = new Node_typed<bool>(G, {STRING("a:"<<*decision)}, {n}, true);
  }
  graphIndex = n->index;
  n->keys.append(STRING("s:" <<s <<" t:" <<time <<' ' <<folState->isNodeOfParentGraph->keys.scalar()));
  if(mcStats && mcStats->n) n->keys.append(STRING("MC best:" <<mcStats->X.first() <<" n:" <<mcStats->n));
  n->keys.append(STRING("sym cost:" <<symCost <<" terminal:" <<isTerminal));
  n->keys.append(STRING("seq cost:" <<seqCost <<"seq con:" <<seqConstraints <<" feasible:" <<seqFeasible));
  n->keys.append(STRING("path cost:" <<pathCost <<"path con:" <<pathConstraints <<" feasible:" <<pathFeasible));
  n->keys.append(STRING("costSoFar:" <<costSoFar));
  if(folAddToState) n->keys.append(STRING("symAdd:" <<*folAddToState));

  G.getRenderingInfo(n).dotstyle="shape=box";
  if(isInfeasible){
    if(isTerminal)  G.getRenderingInfo(n).dotstyle <<" style=filled fillcolor=violet";
    else G.getRenderingInfo(n).dotstyle <<" style=filled fillcolor=red";
  }else if(isTerminal){
    if(seq.N) G.getRenderingInfo(n).dotstyle <<" style=filled fillcolor=cyan";
    else G.getRenderingInfo(n).dotstyle <<" style=filled fillcolor=blue";
  }else{
    if(seq.N) G.getRenderingInfo(n).dotstyle <<" style=filled fillcolor=green";
  }
//  if(inFringe1) G.getRenderingInfo(n).dotstyle <<" color=green";
  if(inFringe1) G.getRenderingInfo(n).dotstyle <<" peripheries=2";
  if(inFringe2) G.getRenderingInfo(n).dotstyle <<" peripheries=3";

//  n->keys.append(STRING("reward:" <<effPoseReward));
  for(ManipulationTree_Node *ch:children) ch->getGraph(G, n);
}

RUN_ON_INIT_BEGIN(manipulationTree)
ManipulationTree_NodeL::memMove = true;
RUN_ON_INIT_END(manipulationTree)
