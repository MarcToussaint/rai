#include "TreeSearchNode.h"
#include "../Core/util.h"
#include "../Core/graph.h"

static uint TreeSearchNode_ID=0;

rai::TreeSearchNode::TreeSearchNode(TreeSearchNode* parent)
  : ID(TreeSearchNode_ID++), parent(parent) {
  if(parent) parent->n_children++;
}

std::shared_ptr<rai::TreeSearchNode> rai::TreeSearchNode::transitionRandomly(){ return transition(rnd(getNumDecisions())); }

void rai::printTree(const rai::Array<rai::TreeSearchNode*>& T){
  rai::Graph G;
  for(uint i=0;i<T.N;i++){
    TreeSearchNode *n = T(i);
    rai::NodeL par;
    if(n->parent) par.append(G.elem(n->parent->ID));
    rai::String name;
    name <<*n;
    rai::Graph& sub = G.newSubgraph(name, par, {});

    sub.newNode<bool>("complete", {}, n->isComplete);
    sub.newNode<bool>("feasible", {}, n->isFeasible);
    sub.newNode<double>("f_prio", {}, n->f_prio);

    if(n->isTerminal){
      G.getRenderingInfo(sub.isNodeOfGraph).dotstyle <<", shape=box";
    }
  }

  G.checkConsistency();
  G.write(FILE("z.tree"));
  G.writeDot(FILE("z.dot"));
  rai::system("dot -Tpdf z.dot > z.pdf");
}

void rai::printTree(const rai::Array<std::shared_ptr<rai::TreeSearchNode>>& T){
  rai::Array<rai::TreeSearchNode*> TT(T.N);
  for(uint i=0;i<T.N;i++) TT(i) = T(i).get();
  printTree(TT);
}

void rai::printTree(std::ostream& os, const rai::Array<std::shared_ptr<TreeSearchNode> >& T){

    rai::Graph G;
    for(uint i=0;i<T.N;i++){
        TreeSearchNode *n = T(i).get();
        rai::NodeL par;
        if(n->parent) par.append(G.elem(n->parent->ID));
        rai::Graph& sub = G.newSubgraph(n->name, par, {});

        sub.newNode<double>("level", {}, n->f_prio);
        sub.newNode<double>("n_children", {}, n->n_children);
        if(n->needsWidening) sub.newNode<bool>("needsWidening");

        if(n->isTerminal) G.getRenderingInfo(sub.isNodeOfGraph).dotstyle <<", shape=box"; //, style=rounded
        if(!n->isComplete) G.getRenderingInfo(sub.isNodeOfGraph).dotstyle <<", style=dashed";
        if(!n->isFeasible) G.getRenderingInfo(sub.isNodeOfGraph).dotstyle <<", color=red";
//        else if(n->isBest) G.getRenderingInfo(sub.isNodeOfGraph).dotstyle <<", color=orange";
    }

    G.checkConsistency();
    G.write(FILE("z.tree"));
    G.writeDot(FILE("z.dot"));
    rai::system("dot -Tpdf z.dot > z.pdf");
}
