/*  ------------------------------------------------------------------
    Copyright (c) 2011-2024 Marc Toussaint
    email: toussaint@tu-berlin.de

    This code is distributed under the MIT License.
    Please see <root-path>/LICENSE for details.
    --------------------------------------------------------------  */

#pragma once

#include "../Core/util.h"
#include "../Search/TreeSearchNode.h"
#include <math.h>

namespace rai {

struct NodeGlobal {
  RAI_PARAM("LGP/", int, verbose, 1)
  RAI_PARAM("LGP/", double, level_c0, 1.)
  RAI_PARAM("LGP/", double, level_cP, 1.)
  RAI_PARAM("LGP/", double, level_w0, 10.)
  RAI_PARAM("LGP/", double, level_wP, 2.)
  RAI_PARAM("LGP/", double, level_eps, 0.)
};

NodeGlobal& info();

struct ComputeNode : TreeSearchNode {
  double c=0.;     //cost invested into completion of THIS node
  double l=-1.;    //lower bound (also feasibility) computed at completion -> f_prio
  double c_now=0., c_tot=0.;
  double baseLevel=0.;

  ComputeNode(ComputeNode* parent) : TreeSearchNode(parent) {}

  //-- core Astar methods
  virtual void compute();
  virtual void untimedCompute() { HALT("this or compute needs overload"); }

  virtual int getNumDecisions() = 0;
  std::shared_ptr<TreeSearchNode> transition(int i);

  virtual std::shared_ptr<ComputeNode> createNewChild(int i) = 0;

//    virtual double effortHeuristic(){ return 0.; }        //expected effort-to-go (FULL DOWN-STREAM TO LEAF NODE)
//    virtual double branchingHeuristic(){ return 1.; }

  virtual double computePenalty() {
    return ::pow(c/info().level_c0, info().level_cP);
  }
  virtual double branchingPenalty_child(int i) {
    if(getNumDecisions()>=0) return 0;
    HALT("need to overload this");
  }
  virtual double sample() { HALT("need to overload"); } //get a value (at a leaf)

  double level() {
    return baseLevel + computePenalty();
  }

  virtual void store(const char* path) const {}
  virtual void data(Graph& g) const;

  void backup_c(double c) {
    ComputeNode* n = this;
    while(n) {
      n->c_tot += c;
      n = dynamic_cast<ComputeNode*>(n->parent);
    }
  }
};
stdOutPipe(ComputeNode)

/*
  void printComputeTree(std::ostream& os, rai::Array<ComputeNode*> T){

    rai::Graph G;
    for(uint i=0;i<T.N;i++){
      ComputeNode *n = T(i);
      rai::NodeL par;
      if(n->parent) par.append(G.elem(n->parent->ID));
      rai::Graph& sub = G.newSubgraph(n->comp->name, par, {});

      sub.newNode<double>("c", {}, n->comp->c);
  //    sub.newNode<double>("c_children", {}, n->c_children);
      sub.newNode<double>("l", {}, n->comp->l);
  //    sub.newNode<double>("R", {}, n->R);
      sub.newNode<double>("D_n", {}, n->data_n);
      sub.newNode<double>("D_mean", {}, n->data_Y/n->data_n);
      sub.newNode<double>("D_ucb", {}, n->data_ucb);
      sub.newNode<double>("C_ucb", {}, n->mean_ucb);
      sub.newNode<double>("C_eff", {}, n->eff);
      sub.newNode<double>("C_tot", {}, n->comp_C);
      sub.newNode<double>("C_n", {}, n->comp_n);
      sub.newNode<bool>("closed", {}, n->isClosed);

  //    if(n->D.N) sub.newNode<arr>("data", {}, n->D);
      if(n->comp->isTerminal){
  //      sub.newNode<rai::String>("dotstyle", {}, ", color=red");
        G.getRenderingInfo(sub.isNodeOfGraph).dotstyle <<", shape=box";
      }
    }

    G.checkConsistency();
    G.write(FILE("z.tree"));
    G.writeDot(FILE("z.dot"));
    rai::system("dot -Tpdf z.dot > z.pdf");
  }
  */

}
