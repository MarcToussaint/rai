/*  ------------------------------------------------------------------
    Copyright (c) 2011-2024 Marc Toussaint
    email: toussaint@tu-berlin.de

    This code is distributed under the MIT License.
    Please see <root-path>/LICENSE for details.
    --------------------------------------------------------------  */

#include "LGP_node.h"
#include "LGP_tree.h"

#include "../MCTS/solver_PlainMC.h"
#include "../KOMO/komo.h"
#include "../KOMO/switch.h"
#include "../Gui/opengl.h"
#include "../Kin/viewer.h"
#include "../Optim/NLP_Solver.h"

#define DEBUG(x) //x
#define DEL_INFEASIBLE(x) //x

namespace rai {

template<> const char* rai::Enum<BoundType>::names []= {
  "symbolic",
  "pose",
  "seq",
  "path",
  "seqPath",
  "seqVelPath",
  "poseFromSub",
  "max",
  nullptr
};

rai::SkeletonTranscription skeleton2Bound2(BoundType boundType, rai::Skeleton& S, const rai::Configuration& C,
    const arrA& waypoints) {

  if(boundType==BD_pose)
    return S.nlp_finalSlice(C);

  if(boundType==BD_seq)
    return S.nlp_waypoints(C);

  if(boundType==BD_path)
    return S.nlp_path(C);

  if(boundType==BD_seqPath)
    return S.nlp_path(C, waypoints);

  HALT("should not be here!");

  return rai::SkeletonTranscription();
}

void LGP_Node::resetData() {
  skeleton.reset();
  cost = zeros(L);
  constraints = zeros(L);
  count = consts<uint>(0, L);
  count(BD_symbolic) = 1;
  feasible = consts<byte>(true, L);
  problem.resize(L);
  computeTime = zeros(L);
  highestBound=0.;
}

LGP_Node::LGP_Node(LGP_Tree& _tree, uint levels)
  : parent(nullptr), tree(_tree), step(0), time(0.), id(tree.COUNT_node++),
    fol(tree.fol),
    L(levels) {
  //this is the root node!
  fol.reset_state();
  folState = fol.createStateCopy();

  resetData();

  if(tree.filNodes)(*tree.filNodes) <<id <<' ' <<step <<' ' <<time <<' ' <<getTreePathString() <<endl;
}

LGP_Node::LGP_Node(LGP_Node* parent, TreeSearchDomain::Handle& a)
  : parent(parent), tree(parent->tree), step(parent->step+1), id(tree.COUNT_node++),
    fol(parent->fol),
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

  if(tree.filNodes)(*tree.filNodes) <<id <<' ' <<step <<' ' <<time <<' ' <<getTreePathString() <<endl;
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
  if(tree.filComputes)(*tree.filComputes) <<id <<'-' <<step <<'-' <<bound <<endl;
  ensure_skeleton();
  skeleton->collisions = collisions;
  skeleton->verbose = verbose;

  arrA waypoints;
  if(bound==BD_seqPath) {
    CHECK(problem(BD_seq).komo, "BD_seq needs to be computed before");
    waypoints = problem(BD_seq).komo->getPath_qAll();
  }

#if 1
  try {
    problem(bound) = skeleton2Bound2(bound, *skeleton, tree.kin, waypoints);
  } catch(std::runtime_error& err) {
    cout <<"CREATING KOMO FOR SKELETON CRASHED: " <<err.what() <<endl;
    if(tree.filComputes)(*tree.filComputes) <<"SKELETON->KOMO CRASHED:" <<*skeleton <<endl;
    feasible(bound) = false;
    labelInfeasible();
    return;
  }
#else
  if(komoProblem(bound)) komoProblem(bound).reset();
  komoProblem(bound) = make_shared<KOMO>();
  komoProblem(bound)->verbose = rai::MAX(verbose, 0);
  auto comp = skeleton2Bound(komoProblem(bound), bound, *skeleton,
                             startKinematics,
                             collisions,
                             waypoints);
  CHECK(comp, "no compute object returned");
#endif

  shared_ptr<KOMO>& komo = problem(bound).komo;
  shared_ptr<SolverReturn>& ret = problem(bound).ret;
  komo->opt.verbose = rai::MAX(verbose, 0);

  //-- verbosity...
  if(tree.verbose>1) {
    if(komo->opt.verbose>0) {
      cout <<"########## OPTIM lev " <<bound <<endl;
    }

    ofstream logFile(tree.OptLGPDataPath + STRING("komo-" <<id <<'-' <<step <<'-' <<bound));

    logFile <<getTreePathString() <<'\n' <<endl;
    skeleton->write(logFile, skeleton->getSwitches(komo->world));
    logFile <<'\n' <<komo->report(true, false);
    //logFile <<'\n' <<komo->getProblemGraph(false);

    if(komo->opt.verbose>1) {
      skeleton->write(cout, skeleton->getSwitches(komo->world));
    }

    DEBUG(FILE("z.fol") <<fol;);
    DEBUG(FILE("z.problem") <<komo->report(););

    if(komo->opt.verbose>1) cout <<komo->report(true, false) <<endl;
    if(komo->opt.verbose>5) komo->opt.animateOptimization = komo->opt.verbose-5;
  }

  //-- optimize
  try {
    ret = komo->optimize(0.);

//    NLP_Solver sol;
//    sol.setProblem(*problem(bound).nlp);
//    auto ret = sol.solve();
//    problem(bound).sol = sol.solve();

  } catch(std::runtime_error& err) {
    cout <<"KOMO CRASHED: " <<err.what() <<endl;
    if(tree.filComputes)(*tree.filComputes) <<"KOMO CRASHED"<<endl;
    problem(bound).komo.reset();
    feasible(bound) = false;
    labelInfeasible();
    return;
  }
  tree.COUNT_kin += Configuration::setJointStateCount;
  tree.COUNT_opt(bound)++;
  tree.COUNT_time += komo->timeTotal;
  count(bound)++;

  DEBUG(FILE("z.problem") <<komo->report(););
  Graph result = komo->report(false, true, (komo->opt.verbose>0 && bound>=2));
  DEBUG(FILE("z.problem.cost") <<result;);
//  cout <<komo->getCollisionPairs() <<endl;
  //if(bound==BD_seqPath || bound==BD_path) cout <<result <<endl;

  double cost_here = ret->sos;
  double constraints_here = ret->ineq + ret->eq;
  bool feas = (constraints_here<2.);

  if(komo->opt.verbose>0) {
    cout <<"  RESULTS: cost: " <<cost_here <<" constraints: " <<constraints_here <<" feasible: " <<feas <<endl;
  }

  //-- post process costs for level==1
  if(bound==BD_pose) {
    cost_here -= 0.1*this->ret.reward; //account for the symbolic costs
    if(parent) cost_here += parent->cost(bound); //this is sequentially additive cost
  } else {
    cost_here += cost(BD_symbolic); //account for the symbolic costs
  }

  //-- read out and update bound
  //update the bound
  if(feas) {
    if(count(bound)==1 || cost_here<highestBound) highestBound=cost_here;
  }

  if(count(bound)==1 || cost_here<cost(bound)) {
    cost(bound) = cost_here;
    constraints(bound) = constraints_here;
    feasible(bound) = feas;
    computeTime(bound) = komo->timeTotal;
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
          CHECK(fact->is<bool>() && fact->key=="block", "");
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
    branchNode->folAddToState = &fol.KB.addSubgraph("ADD", {branchNode->folState->isNodeOfGraph});
  }
  branchNode->folAddToState->add<bool>(0, true, symbols);

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

String LGP_Node::getTreePathString(char sep) const {
  LGP_NodeL path = getTreePath();
  String str;
  for(LGP_Node* b : path) {
    if(b->decision) str <<*b->decision <<sep;
//    else str <<"ROOT" <<sep;
  }
  return str;
}

extern Array<SkeletonSymbol> skeletonModes;

void LGP_Node::ensure_skeleton() {
  if(skeleton) return;

  skeleton = make_shared<Skeleton>();

  Array<Graph*> states;
  arr times;
  for(LGP_Node* node:getTreePath()) {
    states.append(node->folState);
    times.append(node->time);
  }

  skeleton->setFromStateSequence(states, times);
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
  DEBUG(if(!parent) FILE("z.fol") <<fol);
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
    CHECK_EQ(children.N, actions.N, "");
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
  os <<"------- NODE -------\ns:" <<step <<" t:" <<time;
  if(decision) os <<" a:" <<*decision <<endl;
  else os <<" a:<ROOT>"<<endl;

  os <<"\t state: " <<*folState->isNodeOfGraph <<endl;
  if(path) {
    os <<"\t decision path:";
    LGP_NodeL _path = getTreePath();
    for(LGP_Node* nn: _path)
        if(nn->decision) os <<*nn->decision <<' '; else os <<" <ROOT> ";
    os <<endl;
  }
  os <<"\t depth: " <<step <<endl;
  os <<"\t poseCost: " <<cost(BD_pose) <<'|' <<constraints(BD_pose) <<' ' <<(int)feasible(BD_pose) <<endl;
  os <<"\t seqCost: " <<cost(BD_seq) <<'|' <<constraints(BD_seq) <<' ' <<(int)feasible(BD_seq) <<endl;
  os <<"\t pathCost: " <<cost(BD_path) <<'|' <<constraints(BD_path) <<' ' <<(int)feasible(BD_path) <<endl;
  if(skeleton) os <<*skeleton;
  if(recursive) for(LGP_Node* n:children) n->write(os);
}

Graph LGP_Node::getInfo() const {
  Graph G;
  if(decision) G.add<String>("decision", STRING(*decision));
  else         G.add<String>("decision", "<ROOT>");
  G.add<String>("state", STRING(*folState->isNodeOfGraph));
  G.add<String>("path", getTreePathString());
  G.add<arr>("boundsCost", cost);
  G.add<arr>("boundsConstraints", constraints);
  G.add<boolA>("boundsFeasible", feasible);
  return G;
}

void LGP_Node::getGraph(Graph& G, Node* n, bool brief) {
  if(!n) {
    n = G.add<bool>("a:<ROOT>", true);
  } else {
    n = G.add<bool>({STRING("a:"<<*decision)}, true, {n});
  }

  if(!brief) {
    n->key <<STRING("\ns:" <<step <<" t:" <<time <<" bound:" <<highestBound <<" feas:" <<!isInfeasible <<" term:" <<isTerminal <<' ' <<folState->isNodeOfGraph->key);
    for(uint l=0; l<L; l++) if(count(l))
        n->key <<STRING('\n' <<Enum<BoundType>::name(l) <<" #:" <<count(l) <<" c:" <<cost(l) <<"|" <<constraints(l) <<" " <<(feasible(l)?'1':'0') <<" time:" <<computeTime(l));
    if(folAddToState) n->key <<STRING("\nsymAdd:" <<*folAddToState);
    if(note.N) n->key <<'\n' <<note;
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

  if(this==tree.focusNode) G.getRenderingInfo(n).dotstyle <<" peripheries=2";

  //  if(inFringe1) G.getRenderingInfo(n).dotstyle <<" color=green";
  //  if(inFringe1) G.getRenderingInfo(n).dotstyle <<" peripheries=2";
  //  if(inFringe2) G.getRenderingInfo(n).dotstyle <<" peripheries=3";

  //  n->keys.append(STRING("reward:" <<effPoseReward));
  for(LGP_Node* ch:children) ch->getGraph(G, n, brief);
}

void LGP_Node::displayBound(ConfigurationViewer& V, BoundType bound) {
  if(!problem(bound).komo) {
    LOG(-1) <<"bound was not computed - cannot display";
  } else {
    LOG(0) <<"hit 'q' in the ConfigurationViewer to continue";
    Enum<BoundType> _bound(bound);
    String s;
    s <<"BOUND " <<_bound <<" at step " <<step <<"\n" <<*skeleton;
    s <<"\n sos:" <<problem(bound).ret->sos <<" eq:" <<problem(bound).ret->eq <<" ineq:" <<problem(bound).ret->ineq;
    V.updateConfiguration(tree.kin, problem(bound).komo->timeSlices).view(false, s);
//    V.setMotion(range(), problem(bound).komo->getPath_X(), s, true);
    if(bound>=BD_path) {
      while(V.playVideo(true, 1.*problem(bound).komo->T/problem(bound).komo->stepsPerPhase));
    } else {
      while(V.playVideo(true, 1.*problem(bound).komo->T));
    }
  }
}

} //namespace

RUN_ON_INIT_BEGIN(manipulationTree)
rai::LGP_NodeL::memMove = true;
RUN_ON_INIT_END(manipulationTree)
