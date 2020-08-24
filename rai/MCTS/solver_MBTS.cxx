/*  ------------------------------------------------------------------
    Copyright (c) 2011-2020 Marc Toussaint
    email: toussaint@tu-berlin.de

    This code is distributed under the MIT License.
    Please see <root-path>/LICENSE for details.
    --------------------------------------------------------------  */

#include "solver_MBTS.h"

MBTS_Node::MBTS_Node(MBTS& MBTS, MCTS_Environment& world)
  : MBTS(MBTS), world(world), parent(nullptr), d(0), time(0.) {
  MBTS.size++;
  //this is the root node!
  world.reset_state();
  state = world.get_stateCopy();
//  folState = fol.createStateCopy();
}

MBTS_Node::MBTS_Node(MBTS_Node* parent, const MCTS_Environment::Handle& a)
  : MBTS(parent->MBTS), world(parent->world), action(a), parent(parent), d(parent->d+1) {
  MBTS.size++;
  if(d>MBTS.depth) MBTS.depth=d;
  parent->children.append(this);
  world.set_state(parent->state);
  CHECK(a, "giving a 'nullptr' shared pointer??");
  ret = world.transition(action);
  state = world.get_stateCopy();
  time = parent->time + ret.duration;
  isTerminal = world.is_terminal_state();
  g = parent->g + ret.reward; //cost-so-far
  h = 0.; //heuristic
}

void MBTS_Node::expand() {
  CHECK(!isExpanded && !children.N, "");
  if(isTerminal) return;
  FILE("z.1") <<world <<endl;
  world.set_state(state);
  FILE("z.2") <<world <<endl;
  auto actions = world.get_actions();
  for(const MCTS_Environment::Handle& a:actions) {
    new MBTS_Node(this, a);
  }
  isExpanded=true;
}

MBTS_NodeL MBTS_Node::getTreePath() {
  MBTS_NodeL path;
  MBTS_Node* node=this;
  for(; node;) {
    path.prepend(node);
    node = node->parent;
  }
  return path;
}

void MBTS_Node::getGraph(Graph& G, Node* n) {
  if(!n) {
    n = G.newNode<bool>({"a:<ROOT>"}, NodeL(), true);
  } else {
    n = G.newNode<bool>({STRING("a:"<<*action)}, {n}, true);
  }
  n->keys.append(STRING("d:" <<d <<" t:" <<time <<' '));
  n->keys.append(STRING("f:" <<g+h <<" g:" <<g <<" h:" <<h));
//  if(mcStats && mcStats->n) n->keys.append(STRING("MC best:" <<mcStats->X.first() <<" n:" <<mcStats->n));
//  n->keys.append(STRING("sym  #" <<mcCount <<" f:" <<symCost <<" terminal:" <<isTerminal));
//  n->keys.append(STRING("pose #" <<poseCount <<" f:" <<poseCost <<" g:" <<poseConstraints <<" feasible:" <<poseFeasible));
//  n->keys.append(STRING("seq  #" <<seqCount <<" f:" <<seqCost <<" g:" <<seqConstraints <<" feasible:" <<seqFeasible));
//  n->keys.append(STRING("path #" <<pathCount <<" f:" <<pathCost <<" g:" <<pathConstraints <<" feasible:" <<pathFeasible));
//  if(folAddToState) n->keys.append(STRING("symAdd:" <<*folAddToState));

  G.getRenderingInfo(n).dotstyle="shape=box";
  if(isInfeasible) {
    if(isTerminal)  G.getRenderingInfo(n).dotstyle <<" style=filled fillcolor=violet";
    else G.getRenderingInfo(n).dotstyle <<" style=filled fillcolor=red";
  } else if(isTerminal) {
//    if(seqCount || pathCount) G.getRenderingInfo(n).dotstyle <<" style=filled fillcolor=cyan";
//    else G.getRenderingInfo(n).dotstyle <<" style=filled fillcolor=blue";
  } else {
//    if(poseCount || seqCount || pathCount) G.getRenderingInfo(n).dotstyle <<" style=filled fillcolor=green";
  }
//  if(inFringe1) G.getRenderingInfo(n).dotstyle <<" color=green";
//  if(inFringe1) G.getRenderingInfo(n).dotstyle <<" peripheries=2";
//  if(inFringe2) G.getRenderingInfo(n).dotstyle <<" peripheries=3";

//  n->keys.append(STRING("reward:" <<effPoseReward));
  for(MBTS_Node* ch:children) ch->getGraph(G, n);
}

void MBTS_Node::getAll(MBTS_NodeL& L) {
  L.append(this);
  for(MBTS_Node* ch:children) ch->getAll(L);
}

void MBTS_Node::write(ostream& os, bool recursive) const {
  if(action) os <<" a= " <<*action;
  else os <<" a=<ROOT>";
  cout <<"d:" <<d <<" t:" <<time <<" f:" <<g+h <<" g:" <<g <<" h:" <<h <<endl;
  if(recursive) for(MBTS_Node* n:children) n->write(os);
}

//===========================================================================

MBTS::MBTS(MCTS_Environment& world, MBTS_Heuristic& heuristic, uint L)
  : root(nullptr), heuristic(heuristic), size(0), depth(0) {
  root = new NodeT(*this, world);
  queue.resize(L);
  queue(0).add(0., root);
}

bool MBTS::step(int level) {
  if(!queue(level).N) {
    if(level==0) LOG(-1) <<"MBTS: queue is empty -> failure?";
    return false;
  }
  MBTS_Node* next =  queue.pop();

  if(level>0 && next->parent) {
    CHECK(next->parent->isEvaluated(level-1), "");
  }

  //evaluate
  MBTS_Heuristic::Return ret = heuristic.evaluate(next, level);
  if(level==0 && ret.terminal) {
    solutions.append(next);
    return true;
  }
  next->g(level) = ret.g;
  next->h(level) = ret.h;
  if(!ret.feasible) next->labelInfeasible();
  next->isEvaluated(level)=true;

  //-- expand (feed same-level queue)
  if(level==0) {
    next->expand();
    for(MBTS_Node* ch:next->children) {
      queue(0).add(ch->g(level) + ch->h(level), ch, true);
    }
  }

  if(level==1) {
    for(MBTS_Node* ch:next->children) {
      queue(1).add(ch->g(level) + ch->h(level), ch, true);
    }
  }

  //-- propagate (feed higher level queues)

  return false;
}

void MBTS::reportQueue() {
  cout <<"MBTS QUEUE:" <<endl;
  for(const PriorityQueueEntry<MBTS_Node*>& n:queue) {
    cout <<"p=" <<n.p <<" f=" <<n.x->g+n.x->h <<" g=" <<n.x->g <<" h=" <<n.x->h <<" d=" <<n.x->d <<" a=" <<*n.x->action <<endl;
  }
}

RUN_ON_INIT_BEGIN(manipulationTree)
MBTS_NodeL::memMove = true;
RUN_ON_INIT_END(manipulationTree)
