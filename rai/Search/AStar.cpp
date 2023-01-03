#include "AStar.h"

rai::AStar::AStar(const std::shared_ptr<rai::TreeSearchNode>& _root){
    root = _root;
    mem.append(root);
    queue.add(root->f_prio, root.get());
}

bool rai::AStar::step() {
    if(!queue.N) {
        LOG(-1) <<"AStar: queue is empty -> failure!";
        return true;
    }

    //pop
    TreeSearchNode* node =  queue.pop();
    //    LOG(0) <<"looking at node '" <<*node <<"'";

    bool complete = node->compute();
    if(!complete){
        if(!node->isFeasible) return false;
        queue.add(node->f_prio, node, true);
        return false;
    }

    //goal check
    if(node->isTerminal){
        solutions.append(node);
        return true;
    }


    //expand
    //    LOG(0) <<"expanding node '" <<*node <<"'";
    int n = node->getNumActions();
    for(int i=0;i<n;i++) {
        NodeP child = node->transition(i);
        mem.append(child);
        child->compute();
        if(!node->isFeasible) return false;
        queue.add(child->f_prio, child.get(), true);
    }
    return false;
}

void rai::AStar::run() {
    for(;;) {
        if(step()) break;
        //      report();
        iters++;
    }
    if(verbose>0){
      LOG(0) <<"==== DONE ===";
      report();
    }
}

void rai::AStar::report(){
    std::cout <<" iters: " <<iters
        <<" mem#: " <<mem.N
       <<" queue#: " <<queue.N <<endl;
    if(verbose>2) std::cout <<" queue: " <<queue <<std::endl;
    if(solutions.N) std::cout <<" solutions: " <<solutions.modList();
}
