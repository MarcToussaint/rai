/*  ------------------------------------------------------------------
    Copyright (c) 2011-2024 Marc Toussaint
    email: toussaint@tu-berlin.de

    This code is distributed under the MIT License.
    Please see <root-path>/LICENSE for details.
    --------------------------------------------------------------  */

#pragma once

#include "../Core/util.h"
#include "../Core/array.h"

namespace rai {

//===========================================================================

struct TreeSearchNode {
  uint ID=0;
  TreeSearchNode* parent=0;
  Array<TreeSearchNode*> children;
  rai::String name;
  //the derived constructor or compute() need to set or update these
  bool isComplete = false;
  bool isFeasible = true;
  bool isTerminal = false;
  double f_prio=0.;
  bool needsWidening = false;

  TreeSearchNode(TreeSearchNode* parent);
  virtual ~TreeSearchNode() {}

  //compute
  virtual void compute() = 0;

  //transition
  virtual int getNumDecisions() = 0;
  virtual std::shared_ptr<TreeSearchNode> transition(int action) = 0;
  virtual std::shared_ptr<TreeSearchNode> transitionRandomly();

  virtual double treePolicyScore(int i) { HALT("needs overload"); } //e.g. return UCB score of child

  //I/O
  virtual void write(std::ostream& os) const { os <<name; }
  virtual void report(std::ostream& os, int verbose) const { std::cerr <<"NOT OVERLOADED!" <<std::endl; }
  virtual void data(Graph& g) const {}
};
inline std::ostream& operator<<(std::ostream& os, const TreeSearchNode& D) { D.write(os); return os; }

//===========================================================================

void printTree(const rai::Array<std::shared_ptr<TreeSearchNode>>& T);

//===========================================================================

} //namespace
