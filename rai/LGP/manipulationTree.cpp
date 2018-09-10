/*  ------------------------------------------------------------------
    Copyright (c) 2017 Marc Toussaint
    email: marc.toussaint@informatik.uni-stuttgart.de

    This code is distributed under the MIT License.
    Please see <root-path>/LICENSE for details.
    --------------------------------------------------------------  */

#include "manipulationTree.h"
#include "bounds.h"

#include <MCTS/solver_PlainMC.h>
#include <KOMO/komo.h>
#include <Kin/switch.h>

#define DEBUG(x) //x
#define DEL_INFEASIBLE(x) //x

uint COUNT_kin=0;
uint COUNT_evals=0;
uint COUNT_node=0;
uintA COUNT_opt=consts<uint>(0, 4);
double COUNT_time=0.;
rai::String OptLGPDataPath;
ofstream *filNodes=NULL;

bool LGP_useHoming = true;

void MNode::resetData() {
  cost = zeros(L);
  constraints = zeros(L);
  count = consts<uint>(0, L);
  count(l_symbolic) = 1;
  feasible = consts<byte>(true, L);
  komoProblem = consts<KOMO*>(NULL, L);
  opt.resize(L);
  computeTime = zeros(L);
  bound=0.;
}

MNode::MNode(rai::KinematicWorld& kin, FOL_World& _fol, uint levels)
  : parent(NULL), step(0), time(0.), id(COUNT_node++),
    fol(_fol),
    startKinematics(kin),
    L(levels) {
  //this is the root node!
  fol.reset_state();
  folState = fol.createStateCopy();
  
  resetData();
  
  if(filNodes)(*filNodes) <<id <<' ' <<step <<' ' <<time <<' ' <<getTreePathString() <<endl;
}

MNode::MNode(MNode* parent, MCTS_Environment::Handle& a)
  : parent(parent), step(parent->step+1), id(COUNT_node++),
    fol(parent->fol),
    startKinematics(parent->startKinematics),
    L(parent->L) {
  parent->children.append(this);
  
  fol.setState(parent->folState, parent->step);
  CHECK(a,"giving a 'NULL' shared pointer??");
  ret = fol.transition(a);
  time = parent->time + ret.duration;
  isTerminal = fol.successEnd;
  if(fol.deadEnd) isInfeasible=true;
  folState = fol.createStateCopy();
  folDecision = folState->getNode("decision");
  decision = a;
  
  resetData();
  cost(l_symbolic) = parent->cost(l_symbolic) - 0.1*ret.reward; //cost-so-far
  bound = parent->bound - 0.1*ret.reward;
  
  if(filNodes)(*filNodes) <<id <<' ' <<step <<' ' <<time <<' ' <<getTreePathString() <<endl;
}

MNode::~MNode() {
  for(MNode *ch:children) delete ch;
  for(KOMO* k:komoProblem) if(k) delete k;
}

void MNode::expand(int verbose) {
  if(isExpanded) return; //{ LOG(-1) <<"MNode '" <<*this <<"' is already expanded"; return; }
  CHECK(!children.N,"");
  if(isTerminal) return;
  fol.setState(folState, step);
  int tmp=fol.verbose;
  fol.verbose=verbose;
  auto actions = fol.get_actions();
  fol.verbose=tmp;
  for(FOL_World::Handle& a:actions) {
    //    cout <<"  EXPAND DECISION: " <<*a <<endl;
    new MNode(this, a);
  }
  if(!children.N) isTerminal=true;
  isExpanded=true;
}

void MNode::optLevel(uint level, bool collisions) {
  komoProblem(level) = new KOMO();
  KOMO& komo(*komoProblem(level));

  if(komo.verbose>0){
    cout <<"########## OPTIM lev " <<level <<endl;
  }

  komo.fil = new ofstream(OptLGPDataPath + STRING("komo-" <<id <<'-' <<step <<'-' <<level));
  
  Skeleton S = getSkeleton({"touch", "above", "inside", "impulse",
                            "stable", "stableOn", "dynamic", "dynamicOn",
                            "push", "graspSlide", "liftDownUp"
                           });
  if(level==1 && parent) CHECK(parent->effKinematics.q.N, "I can't compute a pose when no pose was comp. for parent (I need the effKin)");
  skeleton2Bound(komo, BoundType(level), S, startKinematics, (parent?parent->effKinematics:startKinematics), collisions);

  //-- optimize
  DEBUG(FILE("z.fol") <<fol;);
  DEBUG(komo.getReport(false, 1, FILE("z.problem")););
  komo.reportProblem();
//  komo.animateOptimization = 1;

  try {
    //      komo.verbose=3;
    komo.run();
  } catch(const char* msg) {
    cout <<"KOMO FAILED: " <<msg <<endl;
  }
  COUNT_evals += komo.opt->newton.evals;
  COUNT_kin += rai::KinematicWorld::setJointStateCount;
  COUNT_opt(level)++;
  COUNT_time += komo.runTime;
  count(level)++;
  
  DEBUG(komo.getReport(false, 1, FILE("z.problem")););
//  cout <<komo.getReport(true) <<endl;
//  komo.reportProxies(cout, 0.);
//  komo.checkGradients();

  Graph result = komo.getReport((komo.verbose>0 && level>=2));
  DEBUG(FILE("z.problem.cost") <<result;);
  double cost_here = result.get<double>({"total","sqrCosts"});
  double constraints_here = result.get<double>({"total","constraints"});
  bool feas = (constraints_here<1.);
  
  //-- post process komo problem for level==1
  if(level==1) {
    cost_here -= 0.1*ret.reward; //account for the symbolic costs
    if(parent) cost_here += parent->cost(level); //this is sequentially additive cost
    
    effKinematics = *komo.configurations.last();
    
    for(rai::KinematicSwitch *sw: komo.switches) {
      //    CHECK_EQ(sw->timeOfApplication, 1, "need to do this before the optimization..");
      if(sw->timeOfApplication>=2) sw->apply(effKinematics);
    }
    
    effKinematics.reset_q();
    effKinematics.calc_q();
    DEBUG(effKinematics.checkConsistency();)
  } else {
    cost_here += cost(l_symbolic); //account for the symbolic costs
  }
  
  //-- read out and update bound
  //update the bound
  if(feas) {
    if(count(level)==1/*&& count({2,-1})==0 (also no higher levels)*/ || cost_here<bound) bound=cost_here;
  }
  
  if(count(level)==1 || cost_here<cost(level)) {
    cost(level) = cost_here;
    constraints(level) = constraints_here;
    feasible(level) = feas;
    opt(level) = komo.x;
    computeTime(level) = komo.runTime;
  }
  
  if(!feasible(level))
    labelInfeasible();
    
}

void MNode::setInfeasible() {
  isInfeasible = true;
  for(MNode *n:children) n->setInfeasible();
}

void MNode::labelInfeasible() {
  setInfeasible();
  
  //-- remove children
  //  MNodeL tree;
  //  getAllChildren(tree);
  //  for(MNode *n:tree) if(n!=this) delete n; //TODO: memory leak!
  DEL_INFEASIBLE(children.clear();)
  
  //-- create a literal that is equal to the decision literal (tuple) plus an 'INFEASIBLE' prepended
  NodeL symbols = folDecision->parents;
  symbols.prepend(fol.KB.getNode({"INFEASIBLE"}));
  CHECK(symbols(0), "INFEASIBLE symbol not define in fol");
  //  cout <<"\n *** LABELLING INFEASIBLE: "; listWrite(symbols); cout <<endl;
  
  //-- find the right parent-of-generalization
  MNode* branchNode = this;
  while(branchNode->parent) {
    bool stop=false;
    for(Node *fact:branchNode->folState->list()) {
      if(fact->keys.N && fact->keys.last()=="block") {
        if(tuplesAreEqual(fact->parents, symbols)) {
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
  if(!branchNode->folAddToState) {
    branchNode->folAddToState = &fol.KB.newSubgraph({"ADD"}, {branchNode->folState->isNodeOfGraph})->value;
  }
  branchNode->folAddToState->newNode<bool>({}, symbols, true);
  
  //  MNode *root=getRoot();
  branchNode->recomputeAllFolStates();
  //  node->recomputeAllMCStats(false);
  //TODO: resort all queues
}

MNodeL MNode::getTreePath() const {
  MNodeL path;
  MNode *node=(MNode*)this;
  for(; node;) {
    path.prepend(node);
    node = node->parent;
  }
  return path;
}

rai::String MNode::getTreePathString(char sep) const {
  MNodeL path = getTreePath();
  rai::String str;
  for(MNode *b : path) {
    if(b->decision) str <<*b->decision <<sep;
//    else str <<"ROOT" <<sep;
  }
  return str;
}

Skeleton MNode::getSkeleton(StringA predicateFilter,  bool finalStateOnly) const {
  rai::Array<Graph*> states;
  arr times;
  if(!finalStateOnly){
    for(MNode *node:getTreePath()) {
      times.append(node->time);
      states.append(node->folState);
    }
  }else{
    times.append(1.);
    states.append(this->folState);
  }
  
  //setup a done marker array
  uint maxLen=0;
  for(Graph *s:states) if(s->N>maxLen) maxLen = s->N;
  boolA done(states.N, maxLen);
  done = false;
  
  Skeleton skeleton;
  
  for(uint k=0; k<states.N; k++) {
    Graph& G = *states(k);
//    cout <<G <<endl;
    for(uint i=0; i<G.N; i++) {
      if(!done(k,i)) {
        Node *n = G(i);
        if(n->keys.N && n->keys.first()=="decision") continue; //don't pickup decision literals
        StringA symbols;
        for(Node *p:n->parents) symbols.append(p->keys.last());
        
        //check predicate filter
        if(!symbols.N
            || (predicateFilter.N && !predicateFilter.contains(symbols.first()))) continue;
            
        //trace into the future
        uint k_end=k+1;
        for(; k_end<states.N; k_end++) {
          Node *persists = getEqualFactInList(n, *states(k_end), true);
          if(!persists) break;
          done(k_end, persists->index) = true;
        }
        k_end--;
        if(k_end==states.N-1) {
          skeleton.append(SkeletonEntry({symbols, times(k), times.last()}));
        } else {
          skeleton.append(SkeletonEntry({symbols, times(k), times(k_end)}));
        }
      }
    }
  }
  
//  for(auto& s:skeleton) cout <<"SKELETON " <<s <<endl;

  return skeleton;
}

MNode* MNode::getRoot() {
  MNode* n=this;
  while(n->parent) n=n->parent;
  return n;
}

MNode *MNode::getChildByAction(Node *folDecision) {
  for(MNode *ch:children) {
    if(tuplesAreEqual(ch->folDecision->parents, folDecision->parents)) return ch;
  }
  LOG(-1) <<"a child with action '" <<*folDecision <<"' does not exist";
  return NULL;
}

void MNode::getAll(MNodeL& L) {
  L.append(this);
  for(MNode *ch:children) ch->getAll(L);
}

MNode *MNode::treePolicy_random() {
  if(isInfeasible) return NULL;
  if(isTerminal) return NULL;
  if(children.N) return children.rndElem()->treePolicy_random();
  return this;
}

bool MNode::recomputeAllFolStates() {
  if(!parent) { //this is root
    folState->copy(*fol.start_state);
    if(folAddToState) applyEffectLiterals(*folState, *folAddToState, {}, NULL);
  } else {
    fol.setState(parent->folState, parent->step);
    if(fol.is_feasible_action(decision)) {
      ret = fol.transition(decision);
      time = parent->time + ret.duration;
      isTerminal = fol.successEnd;
      if(fol.deadEnd) {
        if(!feasible(l_seq) && !feasible(l_path)) //seq or path have already proven it feasible! Despite the logic...
          isInfeasible=true;
        return false;
      }
      fol.state->index();
      folState->copy(*fol.state);
      if(folAddToState) applyEffectLiterals(*folState, *folAddToState, {}, NULL);
      folDecision = folState->getNode("decision");
    } else {
      if(!feasible(l_seq) && !feasible(l_path)) //seq or path have already proven it feasible! Despite the logic...
        isInfeasible=true;
      return false;
    }
  }
  if(children.N) {
    for(uint i=children.N-1;;) {
      DEL_INFEASIBLE(bool feasible = children(i)->recomputeAllFolStates();)
      DEL_INFEASIBLE(if(!feasible) children.remove(i);)
        if(!i || !children.N) break;
      i--;
    }
  }
  DEBUG(if(!parent) FILE("z.fol") <<fol;)
    return true;
}

void MNode::checkConsistency() {
  //-- check that the state->parent points to the parent's state
  if(parent) {
    CHECK_EQ(parent->folState->isNodeOfGraph, folState->isNodeOfGraph->parents.scalar(), "");
    CHECK_EQ(&folDecision->container, folState, "");
  }
  
  //-- check that each child exactly matches a decision, in same order
  if(children.N) {
    fol.setState(folState, step);
    auto actions = fol.get_actions();
    CHECK_EQ(children.N, actions.size(), "");
#ifndef RAI_NOCHECK
    uint i=0;
    for(FOL_World::Handle& a:actions) {
      //      cout <<"  DECISION: " <<*a <<endl;
      FOL_World::Handle& b = children(i)->decision;
      CHECK_EQ(*a, *b, "children do not match decisions");
      i++;
    }
#endif
  }
  
  for(auto* ch:children) ch->checkConsistency();
}

void MNode::write(ostream& os, bool recursive, bool path) const {
  os <<"------- NODE -------\ns=" <<step <<" t=" <<time;
  if(decision) os <<" a=" <<*decision <<endl;
  else os <<" a=<ROOT>"<<endl;
  
  os <<"\t state= " <<*folState->isNodeOfGraph <<endl;
  if(path) {
    os <<"\t decision path:";
    MNodeL _path = getTreePath();
    for(MNode *nn: _path)
        if(nn->decision) os <<*nn->decision <<' '; else os <<" <ROOT> ";
    os <<endl;
  }
  os <<"\t depth=" <<step <<endl;
  os <<"\t poseCost=" <<cost(l_pose) <<endl;
  os <<"\t seqCost=" <<cost(l_seq) <<endl;
  os <<"\t pathCost=" <<cost(l_path) <<endl;
  if(recursive) for(MNode *n:children) n->write(os);
}

void MNode::getGraph(Graph& G, Node* n, bool brief) {
  if(!n) {
    n = G.newNode<bool>({"a:<ROOT>"}, NodeL(), true);
  } else {
    n = G.newNode<bool>({STRING("a:"<<*decision)}, {n}, true);
  }
  
  if(!brief) {
    n->keys.append(STRING("s:" <<step <<" t:" <<time <<" bound:" <<bound <<" feas:" <<!isInfeasible <<" term:" <<isTerminal <<' ' <<folState->isNodeOfGraph->keys.scalar()));
    for(uint l=0; l<L; l++)
      n->keys.append(STRING("L" <<l <<" #:" <<count(l) <<" c:" <<cost(l) <<"|" <<constraints(l) <<" " <<(feasible(l)?'1':'0') <<" time:" <<computeTime(l)));
    if(folAddToState) n->keys.append(STRING("symAdd:" <<*folAddToState));
    if(note.N) n->keys.append(note);
  }
  
  G.getRenderingInfo(n).dotstyle="shape=box";
  if(isInfeasible) {
    if(isTerminal)  G.getRenderingInfo(n).dotstyle <<" style=filled fillcolor=violet";
    else G.getRenderingInfo(n).dotstyle <<" style=filled fillcolor=red";
  } else if(isTerminal) {
    if(count(l_seq) || count(l_path)) G.getRenderingInfo(n).dotstyle <<" style=filled fillcolor=cyan";
    else G.getRenderingInfo(n).dotstyle <<" style=filled fillcolor=blue";
  } else {
    if(sum(count) - count(l_symbolic)>0) G.getRenderingInfo(n).dotstyle <<" style=filled fillcolor=green";
  }
  //  if(inFringe1) G.getRenderingInfo(n).dotstyle <<" color=green";
  //  if(inFringe1) G.getRenderingInfo(n).dotstyle <<" peripheries=2";
  //  if(inFringe2) G.getRenderingInfo(n).dotstyle <<" peripheries=3";
  
  //  n->keys.append(STRING("reward:" <<effPoseReward));
  for(MNode *ch:children) ch->getGraph(G, n, brief);
}

RUN_ON_INIT_BEGIN(manipulationTree)
MNodeL::memMove = true;
RUN_ON_INIT_END(manipulationTree)
