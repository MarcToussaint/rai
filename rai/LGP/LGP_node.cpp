/*  ------------------------------------------------------------------
    Copyright (c) 2017 Marc Toussaint
    email: marc.toussaint@informatik.uni-stuttgart.de

    This code is distributed under the MIT License.
    Please see <root-path>/LICENSE for details.
    --------------------------------------------------------------  */

#include "LGP_node.h"
#include "bounds.h"

#include <MCTS/solver_PlainMC.h>
#include <KOMO/komo.h>
#include <Kin/switch.h>

#define DEBUG(x) //x
#define DEL_INFEASIBLE(x) //x

uint COUNT_kin=0;
uint COUNT_evals=0;
uint COUNT_node=0;
uintA COUNT_opt=consts<uint>(0, BD_max);
double COUNT_time=0.;
rai::String OptLGPDataPath;
ofstream *filNodes=NULL;

bool LGP_useHoming = true;

void LGP_Node::resetData() {
  cost = zeros(L);
  constraints = zeros(L);
  count = consts<uint>(0, L);
  count(BD_symbolic) = 1;
  feasible = consts<byte>(true, L);
  komoProblem = consts<KOMO*>(NULL, L);
  opt.resize(L);
  computeTime = zeros(L);
  highestBound=0.;
}

LGP_Node::LGP_Node(rai::KinematicWorld& kin, FOL_World& _fol, uint levels)
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

LGP_Node::LGP_Node(LGP_Node* parent, MCTS_Environment::Handle& a)
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
  cost(BD_symbolic) = parent->cost(BD_symbolic) - 0.1*ret.reward; //cost-so-far
  highestBound = parent->highestBound - 0.1*ret.reward;
  
  if(filNodes)(*filNodes) <<id <<' ' <<step <<' ' <<time <<' ' <<getTreePathString() <<endl;
}

LGP_Node::~LGP_Node() {
  for(LGP_Node *ch:children) delete ch;
  for(KOMO* k:komoProblem) if(k) delete k;
}

void LGP_Node::expand(int verbose) {
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
    new LGP_Node(this, a);
  }
  if(!children.N) isTerminal=true;
  isExpanded=true;
}

void LGP_Node::computeEndKinematics(){
  Skeleton S = getSkeleton({"touch", "above", "inside", "impulse",
                            "stable", "stableOn", "dynamic", "dynamicOn",
                            "push", "graspSlide", "liftDownUp"
                           });

  effKinematics.copy(startKinematics);
  KOMO tmp;
  tmp.setModel(startKinematics, false);
  tmp.setTiming(1., 1, 10., 1);
  tmp.setSkeleton(S);
//  tmp.reportProblem();
  for(rai::KinematicSwitch *s : tmp.switches) s->apply(effKinematics);
}

void LGP_Node::optBound(BoundType bound, bool collisions) {
  if(komoProblem(bound)) delete komoProblem(bound);
  komoProblem(bound) = new KOMO();
  KOMO& komo(*komoProblem(bound));

  if(komo.verbose>0){
    cout <<"########## OPTIM lev " <<bound <<endl;
  }

  komo.fil = new ofstream(OptLGPDataPath + STRING("komo-" <<id <<'-' <<step <<'-' <<bound));
  
  Skeleton S = getSkeleton({"touch", "above", "inside", "impulse",
                            "stable", "stableOn", "dynamic", "dynamicOn",
                            "push", "graspSlide", "liftDownUp"
                           });

  if(komo.verbose>1)  for(auto& s:S) cout <<"SKELETON " <<s <<endl;

  //ensure the effective kinematics are computed when BD_pose
//  if(bound==BD_pose && step>1){
//    if(!parent->effKinematics.q.N) parent->optBound(BD_pose, collisions);
//    CHECK(parent->effKinematics.q.N, "I can't compute a pose when no pose was comp. for parent (I need the effKin)");
//  }
  if(bound==BD_pose && parent){
    if(!parent->effKinematics.q.N) parent->computeEndKinematics();
  }
  arrA waypoints;
  if(bound==BD_seqPath){
    CHECK(komoProblem(BD_seq), "BD_seq needs to be computed before");
    waypoints = komoProblem(BD_seq)->getPath_q();
  }

  skeleton2Bound(komo, bound, S, startKinematics, (parent?parent->effKinematics:startKinematics), collisions,
                 waypoints);

//  if(level==BD_seq) komo.denseOptimization=true;

  //-- optimize
  DEBUG(FILE("z.fol") <<fol;);
  DEBUG(komo.getReport(false, 1, FILE("z.problem")););
  if(komo.verbose>1) komo.reportProblem();
  if(komo.verbose>2) komo.animateOptimization = komo.verbose-2;

  try {
    komo.run();
  } catch(const char* msg) {
    cout <<"KOMO CRASHED: " <<msg <<endl;
  }
  if(!komo.denseOptimization) COUNT_evals += komo.opt->newton.evals;
  COUNT_kin += rai::KinematicWorld::setJointStateCount;
  COUNT_opt(bound)++;
  COUNT_time += komo.runTime;
  count(bound)++;
  
  DEBUG(komo.getReport(false, 1, FILE("z.problem")););
//  cout <<komo.getReport(true) <<endl;
//  komo.reportProxies(cout, 0.);
//  komo.checkGradients();

  Graph result = komo.getReport((komo.verbose>0 && bound>=2));
  DEBUG(FILE("z.problem.cost") <<result;);
  double cost_here = result.get<double>({"total","sqrCosts"});
  double constraints_here = result.get<double>({"total","constraints"});
  bool feas = (constraints_here<1.);
  
  //-- post process komo problem for level==1
  if(bound==BD_pose) {
    cost_here -= 0.1*ret.reward; //account for the symbolic costs
    if(parent) cost_here += parent->cost(bound); //this is sequentially additive cost
    
    effKinematics = *komo.configurations.last();
    
    for(rai::KinematicSwitch *sw: komo.switches) {
      //    CHECK_EQ(sw->timeOfApplication, 1, "need to do this before the optimization..");
      if(sw->timeOfApplication>=2) sw->apply(effKinematics);
    }
    
    effKinematics.reset_q();
    effKinematics.calc_q();
    DEBUG(effKinematics.checkConsistency();)
  } else {
    cost_here += cost(BD_symbolic); //account for the symbolic costs
  }
  
  //-- read out and update bound
  //update the bound
  if(feas) {
    if(count(bound)==1/*&& count({2,-1})==0 (also no higher levels)*/ || cost_here<highestBound) highestBound=cost_here;
  }
  
  if(count(bound)==1 || cost_here<cost(bound)) {
    cost(bound) = cost_here;
    constraints(bound) = constraints_here;
    feasible(bound) = feas;
    opt(bound) = komo.x;
    computeTime(bound) = komo.runTime;
  }
  
  if(!feasible(bound))
    labelInfeasible();
    
}

void LGP_Node::setInfeasible() {
  isInfeasible = true;
  for(LGP_Node *n:children) n->setInfeasible();
}

void LGP_Node::labelInfeasible() {
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
  LGP_Node* branchNode = this;
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

MNodeL LGP_Node::getTreePath() const {
  MNodeL path;
  LGP_Node *node=(LGP_Node*)this;
  for(; node;) {
    path.prepend(node);
    node = node->parent;
  }
  return path;
}

rai::String LGP_Node::getTreePathString(char sep) const {
  MNodeL path = getTreePath();
  rai::String str;
  for(LGP_Node *b : path) {
    if(b->decision) str <<*b->decision <<sep;
//    else str <<"ROOT" <<sep;
  }
  return str;
}

Skeleton LGP_Node::getSkeleton(StringA predicateFilter,  bool finalStateOnly) const {
  rai::Array<Graph*> states;
  arr times;
  if(!finalStateOnly){
    for(LGP_Node *node:getTreePath()) {
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
          skeleton.append(SkeletonEntry({times(k), times.last(), symbols}));
        } else {
          skeleton.append(SkeletonEntry({times(k), times(k_end), symbols}));
        }
      }
    }
  }
  
//  for(auto& s:skeleton) cout <<"SKELETON " <<s <<endl;

  return skeleton;
}

LGP_Node* LGP_Node::getRoot() {
  LGP_Node* n=this;
  while(n->parent) n=n->parent;
  return n;
}

LGP_Node *LGP_Node::getChildByAction(Node *folDecision) {
  for(LGP_Node *ch:children) {
    if(tuplesAreEqual(ch->folDecision->parents, folDecision->parents)) return ch;
  }
  LOG(-1) <<"a child with action '" <<*folDecision <<"' does not exist";
  return NULL;
}

void LGP_Node::getAll(MNodeL& L) {
  L.append(this);
  for(LGP_Node *ch:children) ch->getAll(L);
}

LGP_Node *LGP_Node::treePolicy_random() {
  if(isInfeasible) return NULL;
  if(isTerminal) return NULL;
  if(children.N) return children.rndElem()->treePolicy_random();
  return this;
}

bool LGP_Node::recomputeAllFolStates() {
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
        if(!feasible(BD_seq) && !feasible(BD_path)) //seq or path have already proven it feasible! Despite the logic...
          isInfeasible=true;
        return false;
      }
      fol.state->index();
      folState->copy(*fol.state);
      if(folAddToState) applyEffectLiterals(*folState, *folAddToState, {}, NULL);
      folDecision = folState->getNode("decision");
    } else {
      if(!feasible(BD_seq) && !feasible(BD_path)) //seq or path have already proven it feasible! Despite the logic...
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

void LGP_Node::checkConsistency() {
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

void LGP_Node::write(ostream& os, bool recursive, bool path) const {
  os <<"------- NODE -------\ns=" <<step <<" t=" <<time;
  if(decision) os <<" a=" <<*decision <<endl;
  else os <<" a=<ROOT>"<<endl;
  
  os <<"\t state= " <<*folState->isNodeOfGraph <<endl;
  if(path) {
    os <<"\t decision path:";
    MNodeL _path = getTreePath();
    for(LGP_Node *nn: _path)
        if(nn->decision) os <<*nn->decision <<' '; else os <<" <ROOT> ";
    os <<endl;
  }
  os <<"\t depth=" <<step <<endl;
  os <<"\t poseCost=" <<cost(BD_pose) <<endl;
  os <<"\t seqCost=" <<cost(BD_seq) <<endl;
  os <<"\t pathCost=" <<cost(BD_path) <<endl;
  if(recursive) for(LGP_Node *n:children) n->write(os);
}

void LGP_Node::getGraph(Graph& G, Node* n, bool brief) {
  if(!n) {
    n = G.newNode<bool>({"a:<ROOT>"}, NodeL(), true);
  } else {
    n = G.newNode<bool>({STRING("a:"<<*decision)}, {n}, true);
  }
  
  if(!brief) {
    n->keys.append(STRING("s:" <<step <<" t:" <<time <<" bound:" <<highestBound <<" feas:" <<!isInfeasible <<" term:" <<isTerminal <<' ' <<folState->isNodeOfGraph->keys.scalar()));
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
    if(count(BD_seq) || count(BD_path)) G.getRenderingInfo(n).dotstyle <<" style=filled fillcolor=cyan";
    else G.getRenderingInfo(n).dotstyle <<" style=filled fillcolor=blue";
  } else {
    if(sum(count) - count(BD_symbolic)>0) G.getRenderingInfo(n).dotstyle <<" style=filled fillcolor=green";
  }
  //  if(inFringe1) G.getRenderingInfo(n).dotstyle <<" color=green";
  //  if(inFringe1) G.getRenderingInfo(n).dotstyle <<" peripheries=2";
  //  if(inFringe2) G.getRenderingInfo(n).dotstyle <<" peripheries=3";
  
  //  n->keys.append(STRING("reward:" <<effPoseReward));
  for(LGP_Node *ch:children) ch->getGraph(G, n, brief);
}

RUN_ON_INIT_BEGIN(manipulationTree)
MNodeL::memMove = true;
RUN_ON_INIT_END(manipulationTree)
