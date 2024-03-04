/*  ------------------------------------------------------------------
    Copyright (c) 2011-2024 Marc Toussaint
    email: toussaint@tu-berlin.de

    This code is distributed under the MIT License.
    Please see <root-path>/LICENSE for details.
    --------------------------------------------------------------  */

#pragma once

#include "ComputeNode.h"

#include "../Core/array.h"

//===========================================================================
//
// wrapping the user provided ComputeNode with data for the solver

struct CT_Node {
  CT_Node* parent=0;
  std::shared_ptr<rai::ComputeNode> comp;

  bool childrenComplete=false; //all possible children are complete
  bool branchComplete=false; //all possible decsendants are complete
  uint R=0;        //#children
  uint c_children=0; //#compute invested in childrent \sum_i:ch ch->c
  uint n_children=0; //#complete children
  double comp_n=0.;
//  arr D;           //data at leafs

  double y_tot=0., y_num=0., y_ucb=0.;
  double mean_Y=0., mean_n=0., mean_ucb=0.;
  double eff=0.;
  double c_tot=0., c_soFar=0.;
  double score=0.;
  bool isSelected=false, isBest=false;

  rai::Array<shared_ptr<CT_Node>> children;

  CT_Node(CT_Node* _parent, shared_ptr<rai::TreeSearchNode> _comp);

  void write(ostream& os) const;
};
stdOutPipe(CT_Node)

//===========================================================================
// solver

struct ComputeTree_SolverOptions {
  enum SolverMethod { noMethod=0, SCE_Thresholded, SCE_RoundRobin, SCE_IterativeLimited };

  RAI_PARAM_ENUM("CT/", SolverMethod, method1, SCE_Thresholded)
  RAI_PARAM_ENUM("CT/", SolverMethod, method2, SCE_RoundRobin)
  RAI_PARAM("CT/", int, verbose, 1)
  RAI_PARAM("CT/", double, gamma, 1.)
  RAI_PARAM("CT/", double, beta, 1.)
  RAI_PARAM("CT/", double, epsilon, .1)
  RAI_PARAM("CT/", double, theta, .1)
  RAI_PARAM("CT/", double, rr_sampleFreq, 10.)
  RAI_PARAM("CT/", double, rr_computeFreq, 3.)
};

struct ComputeTree_Solver {
  CT_Node root;

  rai::Array<CT_Node*> all;
  rai::Array<CT_Node*> terminals;
  rai::Array<CT_Node*> nonTerminals;
  rai::Array<CT_Node*> solutions;

  //parameter
  ComputeTree_SolverOptions opt;

  //variables
  uint steps=0;
  double y_baseline = -1., y_now=-1, c_now=0.;
  double regret=0.;

  //reporting
  shared_ptr<ofstream> fil;
  uint filLast=0;

  ComputeTree_Solver(const shared_ptr<rai::ComputeNode>& _root);

  void step();
  void run(double costLimit) { costLimit += totalCost(); while(totalCost()<costLimit) step(); }
  void runTrivial(uint k, double maxEffortPerCompute=10.);
  void report();
  double totalCost() { return root.c_tot + opt.epsilon*root.y_num; }

 private:

  void query(CT_Node* n);
  CT_Node* select_Thresholded();
  CT_Node* select_RoundRobin();
  CT_Node* selectBestCompute_IterativeLimited();
  CT_Node* selectBestCompute_RoundRobin();

  void clearScores();
  CT_Node* getBestCompute();
  CT_Node* getBestExpand();
  CT_Node* getBestSample_Flat();
  CT_Node* getBestSample_UCT();

  CT_Node* getCheapestIncompleteChild(CT_Node* r);

  //--
  uint rr_sample=0, rr_compute=0, rr_compComp=0, rr_compExp=0;
  rai::Array<CT_Node*> lifo;
  uint limit_R=0;
  double limit_c=1.;
  rai::Array<CT_Node*> rr_computeFifo;

};

//===========================================================================
// helpers

void printTree(ostream& os, CT_Node& root);
template<class T> uint getDepth(T* n) {
  int i=0;
  while(n->parent) { n=n->parent; i++; }
  return i;
}
