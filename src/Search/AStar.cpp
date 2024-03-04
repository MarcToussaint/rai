/*  ------------------------------------------------------------------
    Copyright (c) 2011-2024 Marc Toussaint
    email: toussaint@tu-berlin.de

    This code is distributed under the MIT License.
    Please see <root-path>/LICENSE for details.
    --------------------------------------------------------------  */

#include "AStar.h"

rai::AStar::AStar(const std::shared_ptr<rai::TreeSearchNode>& _root, SearchMode _mode)
  : root(_root), mode(_mode) {
  root->ID = 0;
  mem.append(root);
  addToQueue(root.get());
}

void rai::AStar::step() {
  steps++;

  //pop
  TreeSearchNode* node = 0;
  if(mode==astar || mode==FIFO) {
    if(!queue.N) {
      LOG(-1) <<"AStar: queue is empty -> failure!";
      return;
    }
    node = queue.pop();
    if(mode==astar) {
      CHECK_GE(node->f_prio, currentLevel, "level needs to increase");
    }
    currentLevel = node->f_prio;
  } else if(mode==treePolicy) {
    node = selectByTreePolicy();
  }
  //    LOG(0) <<"looking at node '" <<*node <<"'";

  //widen
  TreeSearchNode* siblingToBeAdded = 0;
  if(node->needsWidening) {
    CHECK(node->parent, "");
    NodeP sibling = node->parent->transition(node->parent->children.N);
    if(sibling) {
      CHECK_EQ(sibling->parent, node->parent, "")
      CHECK_GE(sibling->f_prio, currentLevel, "sibling needs to have greater level")
      sibling->ID = mem.N;
      mem.append(sibling);
      siblingToBeAdded = sibling.get();
      //queue.add(sibling->f_prio, sibling.get(), false);
      if(node->parent->getNumDecisions()==-1) sibling->needsWidening=true;
    }
    node->needsWidening=false;
  }

  //compute
  if(!node->isComplete) {
    node->compute();
  }

  //depending on state -> drop, reinsert, save as solution, or expand
  if(!node->isFeasible) { //drop node completely

  } else if(!node->isComplete) { //send back to queue
    addToQueue(node);

  } else if(mode==astar && node->f_prio>currentLevel) { //send back to queue - might not be optimal anymore
    addToQueue(node);

  } else if(node->isTerminal) {  //save as solution
    solutions.append(node);

  } else { //expand or deepen
    CHECK(node->isComplete, "");
    CHECK(!node->isTerminal, "");

    //LOG(0) <<"expanding node '" <<*node <<"'";
    int n = node->getNumDecisions();
    uint createN = n;
    if(n==-1) { createN=1; } //infinity -> add only the first

    for(uint i=0; i<createN; i++) {
      NodeP child = node->transition(i);
      CHECK_EQ(child->parent, node, "")
      CHECK_GE(child->f_prio, currentLevel, "children needs to have greater level")
      child->ID = mem.N;
      mem.append(child);
      //child->compute();
      //if(!node->isFeasible) return false;
      addToQueue(child.get());
      if(n==-1) child->needsWidening=true;
    }

  }

  //remember inserting the sibling, FIFO style (also to allow for FIFO mode)
  if(siblingToBeAdded) {
    addToQueue(siblingToBeAdded);
  }
}

bool rai::AStar::run(int stepsLimit) {
  uint numSol=solutions.N;
  for(;;) {
    step();
    if(solutions.N>numSol) break;
    if(isEmpty()) break;
    if(stepsLimit>=0 && (int)steps>=stepsLimit) break;
  }
  if(verbose>0) {
    LOG(0) <<"# of new solution found: " <<solutions.N - numSol;
    report();
  }
  if(solutions.N>numSol) return true;
  return false;
}

void rai::AStar::report() {
  std::cout <<" iters: " <<steps
            <<" mem#: " <<mem.N
            <<" queue#: " <<queue.N <<endl;
  if(verbose>2) std::cout <<" queue: " <<queue <<std::endl;
  if(solutions.N) { std::cout <<" solutions: " <<solutions.modList(); std::cout <<endl; }
}

rai::TreeSearchNode* rai::AStar::selectByTreePolicy() {
  rai::TreeSearchNode* node = root.get();

  //-- TREE POLICY
  while(node->children.N
        //(int)node->children.N == node->getNumDecisions()  //# we are 'inside' the full expanded tree: children for each action -> UCB to select the most promising
        && !node->isTerminal) {                      //# we're not at a terminal yet

    // compute the UCB scores for all children
    arr scores(node->children.N);
    for(uint i=0; i<scores.N; i++) scores(i) = node->children(i)->treePolicyScore(i);
    //child->data_Q / child->data_n + beta * sqrt(2. * ::log(node->data_n)/child->data_n);

    // pick the child with highest
    node = node->children(argmax(scores));
  }

  return node;
}

void rai::AStar::addToQueue(TreeSearchNode* node) {
  if(mode==FIFO) queue.append(node);
  else queue.add(node->f_prio, node, true);
}
