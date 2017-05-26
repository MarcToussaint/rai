/*  ------------------------------------------------------------------
    Copyright 2016 Marc Toussaint
    email: marc.toussaint@informatik.uni-stuttgart.de
    
    This program is free software: you can redistribute it and/or modify
    it under the terms of the GNU General Public License as published by
    the Free Software Foundation, either version 3 of the License, or (at
    your option) any later version. This program is distributed without
    any warranty. See the GNU General Public License for more details.
    You should have received a COPYING file of the full GNU General Public
    License along with this program. If not, see
    <http://www.gnu.org/licenses/>
    --------------------------------------------------------------  */


#include "manipulationTree.h"
#include <MCTS/solver_PlainMC.h>

#define DEBUG(x) x
#define DEL_INFEASIBLE(x) //x

uint COUNT_kin=0;
uint COUNT_evals=0;
uint COUNT_poseOpt=0;
uint COUNT_seqOpt=0;
uint COUNT_pathOpt=0;

ManipulationTree_Node::ManipulationTree_Node(mlr::KinematicWorld& kin, FOL_World& _fol, uint levels)
  : parent(NULL), step(0),
    fol(_fol), folState(NULL), folDecision(NULL), folAddToState(NULL),
    startKinematics(kin), effKinematics(),
    L(levels),
    rootMC(NULL), mcStats(NULL),
    poseProblem(NULL), seqProblem(NULL), pathProblem(NULL),
    mcCount(0), mcCost(0.) {
  //this is the root node!
  fol.reset_state();
  folState = fol.createStateCopy();
  rootMC = new PlainMC(fol);
  rootMC->verbose = 0;
  cost = zeros(L);
  constraints = zeros(L);
  h = zeros(L);
  count = consts<uint>(0, L);
  feasible = consts<bool>(true, L);
}

ManipulationTree_Node::ManipulationTree_Node(ManipulationTree_Node* parent, MCTS_Environment::Handle& a)
  : parent(parent), fol(parent->fol),
     folState(NULL), folDecision(NULL), folAddToState(NULL),
    startKinematics(parent->startKinematics), effKinematics(),
    L(parent->L),
    rootMC(NULL), mcStats(NULL),
    poseProblem(NULL), seqProblem(NULL), pathProblem(NULL) {
  step=parent->step+1;
  parent->children.append(this);
  fol.setState(parent->folState, parent->step);
  CHECK(a,"giving a 'NULL' shared pointer??");
  ret = fol.transition(a);
  time = parent->time + ret.duration;
  isTerminal = fol.successEnd;
  if(fol.deadEnd) isInfeasible=true;
  folState = fol.createStateCopy();
  folAddToState = NULL; //fresh creation -> notion to add
  folDecision = folState->getNode("decision");
  decision = a;
  rootMC = parent->rootMC;
  cost = zeros(L);
  h = zeros(L);
  count = consts<uint>(0, L);
  feasible = consts<bool>(true, L);
  cost(0) = parent->cost(0) - ret.reward; //cost-so-far
  constraints = zeros(L);
//  h(0) = 0.; //heuristic
}

void ManipulationTree_Node::expand(){
  CHECK(!isExpanded && !children.N,"");
  if(isTerminal) return;
  fol.setState(folState, step);
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
//  cout <<"********\n *** MC from STATE=" <<*fol.state->isNodeOfGraph <<endl;
  if(!rootMC){
    rootMC = new PlainMC(fol);
    rootMC->verbose = 0;
  }

  arr R;

  for(uint k=0;k<num;k++){
    rootMC->initRollout(prefixDecisions);
    fol.setState(folState);
    double r = rootMC->finishRollout(stepAbort);
    R.append( r );
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

#if 0
  arr R = generateRootMCRollouts(num, stepAbort, prefixDecisions);
#else
  arr R;
  for(uint k=0;k<num;k++){
    rootMC->initRollout(prefixDecisions);
    fol.setState(folState);
    double r = rootMC->finishRollout(stepAbort);
    R.append( r );
  }
#endif

  for(ManipulationTree_Node* n:treepath){
    if(!n->mcStats) n->mcStats = new MCStatistics;
    for(auto& r:R){
      n->mcStats->add(r);
      n->mcCost = - n->mcStats->X.first();
    }
  }

  mcCount += num;
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
  komo.setSquaredFixSwitchedObjects(-1., -1., 1e3);

  komo.setAbstractTask(0., *folState);
//  for(mlr::KinematicSwitch *sw: poseProblem->switches){
//    sw->timeOfApplication=2;
//  }

  DEBUG( FILE("z.fol") <<fol; )
  DEBUG( komo.getReport(false, 1, FILE("z.problem")); )
  komo.reset();
  try{
    komo.run();
  } catch(const char* msg){
    cout <<"KOMO FAILED: " <<msg <<endl;
  }
  COUNT_evals += komo.opt->newton.evals;
  COUNT_kin += mlr::KinematicWorld::setJointStateCount;
  COUNT_poseOpt++;
  count(1)++;

  DEBUG( komo.getReport(false, 1, FILE("z.problem")); )

  Graph result = komo.getReport();
  DEBUG( FILE("z.problem.cost") <<result; )
  double cost_here = result.get<double>({"total","sqrCosts"});
  double constraints_here = result.get<double>({"total","constraints"});

  if(parent) cost_here += parent->cost(1);
  cost_here -= ret.reward; //add the symbolic transition cost

  if(!pose.N || cost_here<cost(1)){
    cost(1) = cost_here;
    constraints(1) = constraints_here;
    feasible(1) = (constraints_here<.5);
    pose = komo.x;
  }

  if(!feasible(1))
    labelInfeasible();

  effKinematics = *komo.configurations.last();

  for(mlr::KinematicSwitch *sw: komo.switches){
//    CHECK_EQ(sw->timeOfApplication, 1, "need to do this before the optimization..");
    if(sw->timeOfApplication>=2) sw->apply(effKinematics);
  }
  effKinematics.topSort();
  DEBUG( effKinematics.checkConsistency(); )
  effKinematics.getJointState();
}

void ManipulationTree_Node::solveSeqProblem(int verbose){

  if(!step){ feasible(2)=true; return; } //there is no sequence to compute

  //-- collect 'path nodes'
  ManipulationTree_NodeL treepath = getTreePath();

  seqProblem = new KOMO();
  KOMO& komo(*seqProblem);
  komo.setModel(startKinematics);
  komo.setTiming(time, 2, 5., 1, false);

  komo.setHoming(-1., -1., 1e-1); //gradient bug??
  komo.setSquaredQVelocities();
  komo.setSquaredFixJointVelocities(-1., -1., 1e3);
  komo.setSquaredFixSwitchedObjects(-1., -1., 1e3);

  for(ManipulationTree_Node *node:treepath){
    komo.setAbstractTask((node->parent?node->parent->time:0.), *node->folState);
  }

  DEBUG( FILE("z.fol") <<fol; )
  DEBUG( komo.getReport(false, 1, FILE("z.problem")); )
  komo.reset();
  try{
    komo.run();
  } catch(const char* msg){
    cout <<"KOMO FAILED: " <<msg <<endl;
  }
  COUNT_evals += komo.opt->newton.evals;
  COUNT_kin += mlr::KinematicWorld::setJointStateCount;
  COUNT_seqOpt++;
  count(2)++;

  DEBUG( komo.getReport(false, 1, FILE("z.problem")); )
//  komo.checkGradients();

  Graph result = komo.getReport();
  DEBUG( FILE("z.problem.cost") <<result; )
  double cost_here = result.get<double>({"total","sqrCosts"});
  double constraints_here = result.get<double>({"total","constraints"});

  cost_here -= ret.reward; //add the symbolic transition cost

  if(!seq.N || cost_here<cost(2)){
    cost(2) = cost_here;
    constraints(2) = constraints_here;
    feasible(2) = (constraints_here<.5);
    seq = komo.x;
  }

  if(!feasible(2))
    labelInfeasible();
}

void ManipulationTree_Node::solvePathProblem(uint microSteps, int verbose){

  if(!step){ feasible(3)=true; return; } //there is no path to compute

  //-- collect 'path nodes'
  ManipulationTree_NodeL treepath = getTreePath();

  pathProblem = new KOMO();
  KOMO& komo(*pathProblem);
  komo.setModel(startKinematics);
  komo.setTiming(time, microSteps, 5., 2, false);

  komo.setHoming(-1., -1., 1e-2); //gradient bug??
  komo.setSquaredQAccelerations();
  komo.setSquaredFixJointVelocities(-1., -1., 1e3);
  komo.setSquaredFixSwitchedObjects(-1., -1., 1e3);

  for(ManipulationTree_Node *node:treepath){
    komo.setAbstractTask((node->parent?node->parent->time:0.), *node->folState);
  }

  DEBUG( FILE("z.fol") <<fol; )
  DEBUG( komo.getReport(false, 1, FILE("z.problem")); )
  komo.reset();
  try{
    komo.run();
  } catch(const char* msg){
    cout <<"KOMO FAILED: " <<msg <<endl;
  }
  COUNT_evals += komo.opt->newton.evals;
  COUNT_kin += mlr::KinematicWorld::setJointStateCount;
  COUNT_pathOpt++;
  count(3)++;

  DEBUG( komo.getReport(false, 1, FILE("z.problem")); )
//  komo.checkGradients();

  Graph result = komo.getReport();
  DEBUG( FILE("z.problem.cost") <<result; )
  double cost_here = result.get<double>({"total","sqrCosts"});
  double constraints_here = result.get<double>({"total","constraints"});

  cost_here -= ret.reward; //add the symbolic transition cost

  if(!path.N || cost_here<cost(3)){
    cost(3) = cost_here;
    constraints(3) = constraints_here;
    feasible(3) = (constraints_here<.5);
    path = komo.x;
  }

  if(!feasible(3))
    labelInfeasible();
}

void ManipulationTree_Node::setInfeasible(){
  isInfeasible = true;
  for(ManipulationTree_Node *n:children) n->setInfeasible();
}

void ManipulationTree_Node::labelInfeasible(){
  setInfeasible();

  //-- remove children
//  ManipulationTree_NodeL tree;
//  getAllChildren(tree);
//  for(ManipulationTree_Node *n:tree) if(n!=this) delete n; //TODO: memory leak!
  DEL_INFEASIBLE( children.clear(); )

  //-- create a literal that is equal to the decision literal (tuple) plus an 'INFEASIBLE' prepended
  NodeL symbols = folDecision->parents;
  symbols.prepend( fol.KB.getNode({"INFEASIBLE"}));
//  cout <<"\n *** LABELLING INFEASIBLE: "; listWrite(symbols); cout <<endl;

  //-- find the right parent-of-generalization
  ManipulationTree_Node* branchNode = this;
  while(branchNode->parent){
    bool stop=false;
    for(Node *fact:branchNode->folState->list()){
      if(fact->keys.N && fact->keys.last()=="block"){
        if(tuplesAreEqual(fact->parents, symbols)){
          CHECK(fact->isOfType<bool>() && fact->keys.first()=="block", "");
          stop=true;
          break;
        }
      }
    }
    if(stop) break;
    branchNode = branchNode->parent;
  }

  //add the infeasible-literal as an 'ADD' command to the branch node
  if(!branchNode->folAddToState){
    branchNode->folAddToState = &fol.KB.newSubgraph({"ADD"}, {branchNode->folState->isNodeOfGraph})->value;
  }
  branchNode->folAddToState->newNode<bool>({}, symbols, true);

//  ManipulationTree_Node *root=getRoot();
  branchNode->recomputeAllFolStates();
//  node->recomputeAllMCStats(false);
  //TODO: resort all queues
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


bool ManipulationTree_Node::recomputeAllFolStates(){
  if(!parent){ //this is root
    folState->copy(*fol.start_state);
    if(folAddToState) applyEffectLiterals(*folState, *folAddToState, {}, NULL);
  }else{
    fol.setState(parent->folState, parent->step);
    if(fol.is_feasible_action(decision)){
      ret = fol.transition(decision);
      time = parent->time + ret.duration;
      isTerminal = fol.successEnd;
      if(fol.deadEnd){
        if(!feasible(2) && !feasible(3)) //seq or path have already proven it feasible! Despite the logic...
          isInfeasible=true;
        return false;
      }
      folState->copy(*fol.state);
      if(folAddToState) applyEffectLiterals(*folState, *folAddToState, {}, NULL);
      folDecision = folState->getNode("decision");
    }else{
      if(!feasible(2) && !feasible(3)) //seq or path have already proven it feasible! Despite the logic...
        isInfeasible=true;
      return false;
    }
  }
  if(children.N){
    for(uint i=children.N-1;;){
      DEL_INFEASIBLE( bool feasible = children(i)->recomputeAllFolStates(); )
      DEL_INFEASIBLE( if(!feasible) children.remove(i); )
      if(!i || !children.N) break;
      i--;
    }
  }
  DEBUG( if(!parent) FILE("z.fol") <<fol; )
  return true;
}

void ManipulationTree_Node::recomputeAllMCStats(bool excludeLeafs){
  if(!mcStats) return;
  if(!isTerminal){
    if(children.N || !excludeLeafs || isInfeasible)
      mcStats->clear();
  }
  for(ManipulationTree_Node* ch:children){
    ch->recomputeAllMCStats(excludeLeafs);
    for(double x:ch->mcStats->X) mcStats->add(x);
  }
  if(mcStats->n)
    mcCost = - mcStats->X.first();
  else
    mcCost = 100.;
}

void ManipulationTree_Node::checkConsistency(){
  //-- check that the state->parent points to the parent's state
  if(parent){
    CHECK_EQ(parent->folState->isNodeOfGraph, folState->isNodeOfGraph->parents.scalar(), "");
    CHECK_EQ(&folDecision->container, folState, "");
  }

  //-- check that each child exactly matches a decision, in same order
  if(children.N){
    fol.setState(folState, step);
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
  os <<"------- NODE -------\ns=" <<step <<" t=" <<time;
  if(decision) os <<" a= " <<*decision <<endl;
  else os <<" a=<ROOT>"<<endl;

  for(uint i=0;i<step+1;i++) os <<"  ";
  os <<" state= " <<*folState->isNodeOfGraph <<endl;
  for(uint i=0;i<step+1;i++) os <<"  ";  os <<" depth=" <<step <<endl;
  for(uint i=0;i<step+1;i++) os <<"  ";  os <<" poseCost=" <<cost(1) <<endl;
  for(uint i=0;i<step+1;i++) os <<"  ";  os <<" seqCost=" <<cost(2) <<endl;
  for(uint i=0;i<step+1;i++) os <<"  ";  os <<" pathCost=" <<cost(3) <<endl;
  if(recursive) for(ManipulationTree_Node *n:children) n->write(os);
}

void ManipulationTree_Node::getGraph(Graph& G, Node* n) {
  if(!n){
    n = G.newNode<bool>({"a:<ROOT>"}, NodeL(), true);
  }else{
    n = G.newNode<bool>({STRING("a:"<<*decision)}, {n}, true);
  }
  n->keys.append(STRING("s:" <<step <<" t:" <<time <<' ' <<folState->isNodeOfGraph->keys.scalar()));
  if(mcStats && mcStats->n) n->keys.append(STRING("MC best:" <<mcStats->X.first() <<" n:" <<mcStats->n));
  n->keys.append(STRING("sym  g:" <<cost(0) <<" h:" <<h(0) <<" f:" <<f() <<" terminal:" <<isTerminal));
  n->keys.append(STRING("MC   #" <<mcCount <<" f:" <<mcCost));
  n->keys.append(STRING("pose #" <<count(1) <<" f:" <<cost(1) <<" g:" <<constraints(1) <<" feasible:" <<feasible(1)));
  n->keys.append(STRING("seq  #" <<count(2) <<" f:" <<cost(2) <<" g:" <<constraints(2) <<" feasible:" <<feasible(2)));
  n->keys.append(STRING("path #" <<count(3) <<" f:" <<cost(3) <<" g:" <<constraints(3) <<" feasible:" <<feasible(3)));
  if(folAddToState) n->keys.append(STRING("symAdd:" <<*folAddToState));
  if(note.N) n->keys.append(note);

  G.getRenderingInfo(n).dotstyle="shape=box";
  if(isInfeasible){
    if(isTerminal)  G.getRenderingInfo(n).dotstyle <<" style=filled fillcolor=violet";
    else G.getRenderingInfo(n).dotstyle <<" style=filled fillcolor=red";
  }else if(isTerminal){
    if(count(2) || count(3)) G.getRenderingInfo(n).dotstyle <<" style=filled fillcolor=cyan";
    else G.getRenderingInfo(n).dotstyle <<" style=filled fillcolor=blue";
  }else{
    if(sum(count) - count(0)>0) G.getRenderingInfo(n).dotstyle <<" style=filled fillcolor=green";
  }
//  if(inFringe1) G.getRenderingInfo(n).dotstyle <<" color=green";
//  if(inFringe1) G.getRenderingInfo(n).dotstyle <<" peripheries=2";
//  if(inFringe2) G.getRenderingInfo(n).dotstyle <<" peripheries=3";

//  n->keys.append(STRING("reward:" <<effPoseReward));
  for(ManipulationTree_Node *ch:children) ch->getGraph(G, n);
}

void ManipulationTree_Node::getAll(ManipulationTree_NodeL& L){
  L.append(this);
  for(ManipulationTree_Node *ch:children) ch->getAll(L);
}

RUN_ON_INIT_BEGIN(manipulationTree)
ManipulationTree_NodeL::memMove = true;
RUN_ON_INIT_END(manipulationTree)
