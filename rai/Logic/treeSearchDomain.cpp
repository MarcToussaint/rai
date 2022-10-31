/*  ------------------------------------------------------------------
    Copyright (c) 2011-2020 Marc Toussaint
    email: toussaint@tu-berlin.de

    This code is distributed under the MIT License.
    Please see <root-path>/LICENSE for details.
    --------------------------------------------------------------  */

#include "treeSearchDomain.h"

#include "../Core/util.h"

#include "../Core/graph.h"

std::shared_ptr<const rai::TreeSearchDomain::SAO> rai::NoHandle;

static uint TreeSearchNode_ID=0;

rai::TreeSearchNode::TreeSearchNode(TreeSearchNode* parent)
  : ID(TreeSearchNode_ID++), parent(parent) {
}

std::shared_ptr<rai::TreeSearchNode> rai::TreeSearchNode::transitionRandomly(){ return transition(rnd(getNumActions())); }

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
