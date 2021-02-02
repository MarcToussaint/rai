/*  ------------------------------------------------------------------
    Copyright (c) 2011-2020 Marc Toussaint
    email: toussaint@tu-berlin.de

    This code is distributed under the MIT License.
    Please see <root-path>/LICENSE for details.
    --------------------------------------------------------------  */

#include "LGP_node.h"
#include "LGP_tree.h"
#include "bounds.h"

#include "../MCTS/solver_PlainMC.h"
#include "../KOMO/komo.h"
#include "../Kin/switch.h"
#include "../Optim/GraphOptim.h"
#include "../Gui/opengl.h"
#include "../Kin/viewer.h"

#define DEBUG(x) //x
#define DEL_INFEASIBLE(x) //x

uint COUNT_kin=0;
uint COUNT_evals=0;
uint COUNT_node=0;
uintA COUNT_opt=consts<uint>(0, BD_max);
double COUNT_time=0.;
rai::String OptLGPDataPath;
ofstream* filNodes=nullptr;

bool LGP_useHoming = true;

void LGP_Node::resetData() {
  cost = zeros(L);
  constraints = zeros(L);
  count = consts<uint>(0, L);
  count(BD_symbolic) = 1;
  feasible = consts<byte>(true, L);
  komoProblem.resize(L);
  opt.resize(L);
  computeTime = zeros(L);
  highestBound=0.;
}

LGP_Node::LGP_Node(LGP_Tree* _tree, uint levels)
  : parent(nullptr), tree(_tree), step(0), time(0.), id(COUNT_node++),
    fol(tree->fol),
    startKinematics(tree->kin),
    L(levels) {
  //this is the root node!
  fol.reset_state();
  folState = fol.createStateCopy();

  resetData();

  if(filNodes)(*filNodes) <<id <<' ' <<step <<' ' <<time <<' ' <<getTreePathString() <<endl;
}

LGP_Node::LGP_Node(LGP_Node* parent, MCTS_Environment::Handle& a)
  : parent(parent), tree(parent->tree), step(parent->step+1), id(COUNT_node++),
    fol(parent->fol),
    startKinematics(parent->startKinematics),
    L(parent->L) {
  parent->children.append(this);

  fol.setState(parent->folState, parent->step);
  CHECK(a, "giving a 'nullptr' shared pointer??");
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
  for(LGP_Node* ch:children) delete ch;
}

void LGP_Node::expand(int verbose) {
  if(isExpanded) return; //{ LOG(-1) <<"MNode '" <<*this <<"' is already expanded"; return; }
  CHECK(!children.N, "");
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

void LGP_Node::optBound(BoundType bound, bool collisions, int verbose) {
  if(komoProblem(bound)) komoProblem(bound).reset();
  komoProblem(bound) = make_shared<KOMO>();
  ptr<KOMO>& komo = komoProblem(bound);

  komo->verbose = rai::MAX(verbose, 0);

  if(komo->verbose>0) {
    cout <<"########## OPTIM lev " <<bound <<endl;
  }

  komo->logFile = new ofstream(OptLGPDataPath + STRING("komo-" <<id <<'-' <<step <<'-' <<bound));

  Skeleton S = getSkeleton();

  arrA waypoints;
  if(bound==BD_seqPath || bound==BD_seqVelPath) {
    CHECK(komoProblem(BD_seq), "BD_seq needs to be computed before");
    waypoints = komoProblem(BD_seq)->getPath_q();
  }

  auto comp = skeleton2Bound(komo, bound, S,
                             startKinematics,
                             collisions,
                             waypoints);

  CHECK(comp, "no compute object returned");

  if(komo->logFile) writeSkeleton(*komo->logFile, S, getSwitchesFromSkeleton(S, komo->world));

  if(komo->verbose>1) {
    writeSkeleton(cout, S, getSwitchesFromSkeleton(S, komo->world));
  }

  computes.append(comp);

  for(ptr<Objective>& o:tree->finalGeometryObjectives.objectives) {
    cout <<"FINAL objective: " <<*o <<endl;
    ptr<Objective> co = komo->addObjective({0.}, o->feat, {}, o->type);
    co->setCostSpecs(komo->T-1, komo->T-1);
    cout <<"FINAL objective: " <<*co <<endl;
  }

  if(komo->logFile) {
    komo->reportProblem(*komo->logFile);
    (*komo->logFile) <<komo->getProblemGraph(false);
  }

//  if(level==BD_seq) komo->denseOptimization=true;

  //-- optimize
  DEBUG(FILE("z.fol") <<fol;);
  DEBUG(komo->getReport(false, 1, FILE("z.problem")););
  if(komo->verbose>1) komo->reportProblem();
  if(komo->verbose>5) komo->animateOptimization = komo->verbose-5;

  try {
    if(bound != BD_poseFromSeq) {
      komo->run();
    } else {
      CHECK_EQ(step, komo->T-1, "");
      NIY//komo->run_sub({komo->T-2}, {});
    }
  } catch(std::runtime_error& err) {
    cout <<"KOMO CRASHED: " <<err.what() <<endl;
    komoProblem(bound).reset();
    return;
  }
//  COUNT_evals += komo->opt->newton.evals;
  COUNT_kin += rai::Configuration::setJointStateCount;
  COUNT_opt(bound)++;
  COUNT_time += komo->runTime;
  count(bound)++;

  DEBUG(komo->getReport(false, 1, FILE("z.problem")););
//  cout <<komo->getReport(true) <<endl;
//  komo->reportProxies(cout, 0.);
//  komo->checkGradients();

  Graph result = komo->getReport((komo->verbose>0 && bound>=2));
  DEBUG(FILE("z.problem.cost") <<result;);
  double cost_here = result.get<double>("sos");
  double constraints_here = result.get<double>("eq");
  constraints_here += result.get<double>("ineq");
  if(bound == BD_poseFromSeq) {
    cost_here = komo->sos;
    constraints_here = komo->ineq + komo->eq;
  }
  bool feas = (constraints_here<1.);

  if(komo->verbose>0) {
    cout <<"  RESULTS: cost: " <<cost_here <<" constraints: " <<constraints_here <<" feasible: " <<feas <<endl;
  }

  //-- post process komo problem for level==1
  if(bound==BD_pose) {
    cost_here -= 0.1*ret.reward; //account for the symbolic costs
    if(parent) cost_here += parent->cost(bound); //this is sequentially additive cost
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
    opt(bound) = komo->x;
    computeTime(bound) = komo->runTime;
  }

  if(!feasible(bound))
    labelInfeasible();
}


void LGP_Node::setInfeasible() {
  isInfeasible = true;
  for(LGP_Node* n:children) n->setInfeasible();
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
    for(Node* fact:branchNode->folState->list()) {
      if(fact->key=="block") {
        if(tuplesAreEqual(fact->parents, symbols)) {
          CHECK(fact->isOfType<bool>() && fact->key=="block", "");
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
    branchNode->folAddToState = &fol.KB.newSubgraph({"ADD"}, {branchNode->folState->isNodeOfGraph});
  }
  branchNode->folAddToState->newNode<bool>({}, symbols, true);

  //  MNode *root=getRoot();
  branchNode->recomputeAllFolStates();
  //  node->recomputeAllMCStats(false);
  //TODO: resort all queues
}

LGP_NodeL LGP_Node::getTreePath() const {
  LGP_NodeL path;
  LGP_Node* node=(LGP_Node*)this;
  for(; node;) {
    path.prepend(node);
    node = node->parent;
  }
  return path;
}

rai::String LGP_Node::getTreePathString(char sep) const {
  LGP_NodeL path = getTreePath();
  rai::String str;
  for(LGP_Node* b : path) {
    if(b->decision) str <<*b->decision <<sep;
//    else str <<"ROOT" <<sep;
  }
  return str;
}

extern rai::Array<SkeletonSymbol> skeletonModes;

Skeleton LGP_Node::getSkeleton(bool finalStateOnly) const {
  rai::Array<Graph*> states;
  arr times;
  if(!finalStateOnly) {
    for(LGP_Node* node:getTreePath()) {
      times.append(node->time);
      states.append(node->folState);
    }
  } else {
    times.append(1.);
    states.append(this->folState);
  }

  //setup a done marker array: which literal in each state is DONE
  uint maxLen=0;
  for(Graph* s:states) if(s->N>maxLen) maxLen = s->N;
  boolA done(states.N, maxLen);
  done = false;

  Skeleton skeleton;

  for(uint k=0; k<states.N; k++) {
    Graph& G = *states(k);
//    cout <<G <<endl;
    for(uint i=0; i<G.N; i++) {
      if(!done(k, i)) {
        Node* n = G(i);
        if(n->isGraph() && n->graph().findNode("%decision")) continue; //don't pickup decision literals
        StringA symbols;
        for(Node* p:n->parents) symbols.append(p->key);

        //check if there is a predicate
        if(!symbols.N) continue;

        //check if predicate is a SkeletonSymbol
        if(!rai::Enum<SkeletonSymbol>::contains(symbols.first())) continue;

        //trace into the future
        uint k_end=k+1;
        for(; k_end<states.N; k_end++) {
          Node* persists = getEqualFactInList(n, *states(k_end), true);
          if(!persists) break;
          done(k_end, persists->index) = true;
        }
        k_end--;

        rai::Enum<SkeletonSymbol> sym(symbols.first());
        if(k_end==states.N-1) {
          skeleton.append(SkeletonEntry({times(k), times.last(), sym, symbols({1, -1})}));
        } else {
          skeleton.append(SkeletonEntry({times(k), times(k_end), sym, symbols({1, -1})}));
        }
      }
    }
  }

  for(uint i=0; i<skeleton.N; i++) {
    SkeletonEntry& se =  skeleton.elem(i);
    if(skeletonModes.contains(se.symbol)){ //S(i) is about a switch
      if(se.phase1<times.last()){
        se.phase1 += 1.; //*** MODES EXTEND TO THE /NEXT/ TIME SLICE ***
      }else{
        se.phase1 = -1.;
      }
    }
  }


  return skeleton;
}

LGP_Node* LGP_Node::getRoot() {
  LGP_Node* n=this;
  while(n->parent) n=n->parent;
  return n;
}

LGP_Node* LGP_Node::getChildByAction(Node* folDecision) {
  for(LGP_Node* ch:children) {
    if(tuplesAreEqual(ch->folDecision->parents, folDecision->parents)) return ch;
  }
  LOG(-1) <<"a child with action '" <<*folDecision <<"' does not exist";
  return nullptr;
}

void LGP_Node::getAll(LGP_NodeL& L) {
  L.append(this);
  for(LGP_Node* ch:children) ch->getAll(L);
}

LGP_Node* LGP_Node::treePolicy_random() {
  if(isInfeasible) return nullptr;
  if(isTerminal) return nullptr;
  if(children.N) return children.rndElem()->treePolicy_random();
  return this;
}

bool LGP_Node::recomputeAllFolStates() {
  if(!parent) { //this is root
    folState->copy(*fol.start_state);
    if(folAddToState) applyEffectLiterals(*folState, *folAddToState, {}, nullptr);
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
      if(folAddToState) applyEffectLiterals(*folState, *folAddToState, {}, nullptr);
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
    LGP_NodeL _path = getTreePath();
    for(LGP_Node* nn: _path)
        if(nn->decision) os <<*nn->decision <<' '; else os <<" <ROOT> ";
    os <<endl;
  }
  os <<"\t depth=" <<step <<endl;
  os <<"\t poseCost=" <<cost(BD_pose) <<endl;
  os <<"\t seqCost=" <<cost(BD_seq) <<endl;
  os <<"\t pathCost=" <<cost(BD_path) <<endl;
  if(recursive) for(LGP_Node* n:children) n->write(os);
}

Graph LGP_Node::getInfo() const {
  Graph G;
  if(decision) G.newNode<rai::String>({"decision"}, {}, STRING(*decision));
  else         G.newNode<rai::String>({"decision"}, {}, "<ROOT>");
  G.newNode<rai::String>({"state"}, {}, STRING(*folState->isNodeOfGraph));
  G.newNode<rai::String>({"path"}, {}, getTreePathString());
  G.newNode<arr>({"boundsCost"}, {}, cost);
  G.newNode<arr>({"boundsConstraints"}, {}, constraints);
  G.newNode<boolA>({"boundsFeasible"}, {}, feasible);
  return G;
}

void LGP_Node::getGraph(Graph& G, Node* n, bool brief) {
  if(!n) {
    n = G.newNode<bool>({"a:<ROOT>"}, NodeL(), true);
  } else {
    n = G.newNode<bool>({STRING("a:"<<*decision)}, {n}, true);
  }

  if(!brief) {
    n->key <<(STRING("s:" <<step <<" t:" <<time <<" bound:" <<highestBound <<" feas:" <<!isInfeasible <<" term:" <<isTerminal <<' ' <<folState->isNodeOfGraph->key));
    for(uint l=0; l<L; l++)
      n->key <<(STRING(rai::Enum<BoundType>::name(l) <<" #:" <<count(l) <<" c:" <<cost(l) <<"|" <<constraints(l) <<" " <<(feasible(l)?'1':'0') <<" time:" <<computeTime(l)));
    if(folAddToState) n->key <<(STRING("symAdd:" <<*folAddToState));
    if(note.N) n->key <<(note);
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
  for(LGP_Node* ch:children) ch->getGraph(G, n, brief);
}

void LGP_Node::displayBound(ptr<OpenGL>& gl, BoundType bound) {
  rai::ConfigurationViewer V;
//  V.gl = gl;

  if(!komoProblem(bound)) {
    LOG(-1) <<"bound was not computed - cannot display";
  } else {
//    CHECK(!komoProblem(bound)->gl, "");
    rai::Enum<BoundType> _bound(bound);
    rai::String s;
    s <<"BOUND " <<_bound <<" at step " <<step;
//    komoProblem(bound)->gl = gl;
    V.setConfiguration(komoProblem(bound)->world, s);
    V.setPath(komoProblem(bound)->getPath_frames(), s, true);
    if(bound>=BD_path && bound<=BD_seqVelPath){
//      while(komoProblem(bound)->displayTrajectory(.1, true, false));
      while(V.playVideo(true, 1.*komoProblem(bound)->T/komoProblem(bound)->stepsPerPhase));
    }else{
//      while(komoProblem(bound)->displayTrajectory(-1., true, false));
      while(V.playVideo(true, 1.*komoProblem(bound)->T));
    }
//    komoProblem(bound)->gl.reset();
  }
}

RUN_ON_INIT_BEGIN(manipulationTree)
LGP_NodeL::memMove = true;
RUN_ON_INIT_END(manipulationTree)
