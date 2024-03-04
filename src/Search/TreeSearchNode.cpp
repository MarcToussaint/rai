/*  ------------------------------------------------------------------
    Copyright (c) 2011-2024 Marc Toussaint
    email: toussaint@tu-berlin.de

    This code is distributed under the MIT License.
    Please see <root-path>/LICENSE for details.
    --------------------------------------------------------------  */

#include "TreeSearchNode.h"
#include "../Core/util.h"
#include "../Core/graph.h"

static uint TreeSearchNode_ID=0;

rai::TreeSearchNode::TreeSearchNode(TreeSearchNode* parent)
  : ID(TreeSearchNode_ID++), parent(parent) {
  if(parent) parent->children.append(this);
}

std::shared_ptr<rai::TreeSearchNode> rai::TreeSearchNode::transitionRandomly() { return transition(rnd(getNumDecisions())); }

/*void rai::printTree(const rai::Array<rai::TreeSearchNode*>& T){
  rai::Graph G;
  for(uint i=0;i<T.N;i++){
    TreeSearchNode *n = T(i);
    rai::NodeL par;
    if(n->parent) par.append(G.elem(n->parent->ID));
    rai::String name;
    name <<*n;
    rai::Graph& sub = G.addSubgraph(name, par);

    sub.add<bool>("complete", n->isComplete);
    sub.add<bool>("feasible", n->isFeasible);
    sub.add<double>("f_prio", n->f_prio);

    if(n->isTerminal){
      G.getRenderingInfo(sub.isNodeOfGraph).dotstyle <<", shape=box";
    }
  }

  G.checkConsistency();
  G.write(FILE("z.tree"));
  G.writeDot(FILE("z.dot"));
  rai::system("dot -Tpdf z.dot > z.pdf");
}*/

/*void rai::printTree(const rai::Array<std::shared_ptr<rai::TreeSearchNode>>& T){
  rai::Array<rai::TreeSearchNode*> TT(T.N);
  for(uint i=0;i<T.N;i++) TT(i) = T(i).get();
  printTree(TT);
}*/

void rai::printTree(const rai::Array<std::shared_ptr<TreeSearchNode>>& T) {
  rai::Graph G;
  for(uint i=0; i<T.N; i++) {
    TreeSearchNode* n = T(i).get();
    n->ID = i;
    rai::NodeL par;
    if(n->parent && n->parent->ID<G.N) par.append(G.elem(n->parent->ID));
    rai::Graph& sub = G.addSubgraph(n->name, par);

    sub.add<double>("level", n->f_prio);
    sub.add<double>("n_children", n->children.N);
    if(n->needsWidening) sub.add<bool>("needsWidening", true);
    n->data(sub);

    if(n->isTerminal) G.getRenderingInfo(sub.isNodeOfGraph).dotstyle <<", shape=box"; //, style=rounded
    if(!n->isComplete) G.getRenderingInfo(sub.isNodeOfGraph).dotstyle <<", style=dashed";
    if(!n->isFeasible) G.getRenderingInfo(sub.isNodeOfGraph).dotstyle <<", color=red";
  }

  G.checkConsistency();
  G.write(FILE("z.tree"));
  G.writeDot(FILE("z.dot"));
  rai::system("dot -Tpdf z.dot > z.pdf");
}
