/*  ------------------------------------------------------------------
    Copyright (c) 2011-2024 Marc Toussaint
    email: toussaint@tu-berlin.de

    This code is distributed under the MIT License.
    Please see <root-path>/LICENSE for details.
    --------------------------------------------------------------  */

#include "ComputeNode.h"
#include "../Core/graph.h"

namespace rai {
NodeGlobal& info() {
  static NodeGlobal singleton;
  return singleton;
}
}

void rai::ComputeNode::compute() {
  if(info().verbose>0) {
    LOG(0) <<"compute at " <<name <<" ...";
  }
  c_now = -rai::cpuTime();
  untimedCompute();
  c_now += rai::cpuTime();
  c += c_now;
  backup_c(c_now);
  if(l>1e9) isFeasible=false;
  f_prio = baseLevel + computePenalty();
  if(info().verbose>0) {
    if(isComplete) LOG(0) <<"computed " <<name <<" -> complete with c:" <<c <<" l:" <<l <<" level:" <<f_prio <<(isFeasible?" feasible":" INFEASIBLE") <<(isTerminal?" TERMINAL":0);
    else LOG(0) <<"computed " <<name <<" -> still incomplete with c:" <<c;
  }
}

std::shared_ptr<rai::TreeSearchNode> rai::ComputeNode::transition(int i) {
  auto child = createNewChild(i);
  // update level
  if(!child->parent) {
    CHECK_EQ(child->parent, this, "");
    CHECK_GE(children.N, uint(i+1), "");
    //if((uint)i!=children.N) LOG(0) <<"creating childBranch #" <<i <<" without lower branches first";
  }
  child->baseLevel = baseLevel + computePenalty();
  child->baseLevel += info().level_eps;
  child->baseLevel += branchingPenalty_child(i);
//  if(getNumDecisions()==-1){//infinite branching
//    child->baseLevel += ::pow(double(i)/info().level_w0, info().level_pw);
//  }
  child->f_prio = child->baseLevel;
  //if(info().verbose>0){
  //  LOG(0) <<"created node '" <<child->name <<"' ID:" <<child->ID <<" type: '" <<rai::niceTypeidName(typeid(*child)) <<"' baseLevel:" <<child->baseLevel;
  //}
  return child;
}

void rai::ComputeNode::data(rai::Graph& g) const {
  if(c>0.) g.add<double>("c", c);
  if(l>0.) g.add<double>("l", l);
}
