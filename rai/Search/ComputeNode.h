#pragma once

#include <Core/util.h>

namespace rai {

  struct ComputeNode {
    uint ID=0;
    rai::String name;
    bool isComplete = false;
    bool isTerminal = false;
    double c=0.;     //cost invested into completion of THIS node
    double l=-1.;    //lower bound (also feasibility) computed at completion -> f_prio

    virtual void compute(){ HALT("need to overload");  }
    virtual int getNumDecisions() = 0;
    virtual std::shared_ptr<ComputeNode> getNewChild(uint i) = 0;

    virtual double effortHeuristic(){ return 0.; }        //expected effort-to-go (FULL DOWN-STREAM TO LEAF NODE)
    virtual double branchingHeuristic(){ return 1.; }
    virtual double sample(){ HALT("need to overload"); }  //get a value (at a leaf)

    virtual double timedCompute(){
      double time = -rai::cpuTime();
      compute();
      time += rai::cpuTime();
      return time;
    }
    virtual void write(ostream& os) const{ os <<name; }
    virtual void store(const char* path) const {}
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
