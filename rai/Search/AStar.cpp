#include "AStar.h"

rai::AStar::AStar(const std::shared_ptr<rai::TreeSearchNode>& _root){
  root = _root;
  root->ID = 0;
  mem.append(root);
  queue.add(root->f_prio, root.get());
}

bool rai::AStar::step() {
  if(!queue.N) {
    LOG(-1) <<"AStar: queue is empty -> failure!";
    return true;
  }

  steps++;

  //pop
  TreeSearchNode* node =  queue.pop();
  //    LOG(0) <<"looking at node '" <<*node <<"'";

  //CHECK(!node->isComplete, "this node is already complete - should not be in the queue");
  CHECK_GE(node->f_prio, currentLevel, "level needs to increase");
  currentLevel = node->f_prio;

  //widen
  if(node->needsWidening){
    CHECK(node->parent, "");
    NodeP sibling = node->parent->transition(node->parent->n_children);
    if(sibling){
      CHECK_EQ(sibling->parent, node->parent, "")
      CHECK_GE(sibling->f_prio, currentLevel, "sibling needs to have greater level")
      sibling->ID = mem.N;
      mem.append(sibling);
      queue.add(sibling->f_prio, sibling.get(), false);
      if(node->parent->getNumDecisions()==-1) sibling->needsWidening=true;
    }
    node->needsWidening=false;
  }

  //compute
  if(!node->isComplete){
    node->compute();
    if(!node->isFeasible){ //drop node completely
      return false;
    }
    if(!node->isComplete){ //send back to queue
      queue.add(node->f_prio, node, true);
      return false;
    }
    if(node->f_prio>currentLevel){ //send back to queue - might not be optimal anymore
      queue.add(node->f_prio, node, true);
      return false;
    }
  }

  //terminal check
  if(node->isTerminal){
    solutions.append(node);
    return true;
  }

  //expand or deepen
  //    LOG(0) <<"expanding node '" <<*node <<"'";
  int n = node->getNumDecisions();
  uint createN = n;
  if(n==-1){ createN=1; } //infinity -> add only the first
  for(uint i=0;i<createN;i++) {
    NodeP child = node->transition(i);
    CHECK_EQ(child->parent, node, "")
    CHECK_GE(child->f_prio, currentLevel, "children needs to have greater level")
    child->ID = mem.N;
    mem.append(child);
    //child->compute();
    //if(!node->isFeasible) return false;
    queue.add(child->f_prio, child.get(), false);
    if(n==-1) child->needsWidening=true;
  }
  return false;
}

void rai::AStar::run(int stepsLimit) {
  for(;;) {
    if(step()) break;
    //      report();
    if(stepsLimit>=0 && (int)steps>=stepsLimit) break;
  }
  if(verbose>0){
    LOG(0) <<"==== DONE ===";
    report();
  }
}

void rai::AStar::report(){
  std::cout <<" iters: " <<steps
           <<" mem#: " <<mem.N
          <<" queue#: " <<queue.N <<endl;
  if(verbose>2) std::cout <<" queue: " <<queue <<std::endl;
  if(solutions.N) std::cout <<" solutions: " <<solutions.modList();
}
