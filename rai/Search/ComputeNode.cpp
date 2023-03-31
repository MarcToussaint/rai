#include "ComputeNode.h"
#include "../Core/graph.h"

namespace rai{
NodeGlobal& info(){
  static NodeGlobal singleton;
  return singleton;
}
}

void rai::ComputeNode::compute(){
  if(info().verbose>0){
    LOG(0) <<"compute at " <<name <<" ...";
  }
  c_now = -rai::cpuTime();
  untimedCompute();
  c_now += rai::cpuTime();
  c += c_now;
  if(l>1e9) isFeasible=false;
  f_prio = baseLevel + computePenalty();
  if(info().verbose>0){
    if(isComplete) LOG(0) <<"computed " <<name <<" -> complete with c:" <<c <<" l:" <<l <<" level:" <<f_prio <<(isFeasible?" feasible":" INFEASIBLE") <<(isTerminal?" TERMINAL":0);
    else LOG(0) <<"computed " <<name <<" -> still incomplete with c:" <<c;
  }
}

std::shared_ptr<rai::TreeSearchNode> rai::ComputeNode::transition(int i){
  auto child = createNewChild(i);
  // update level
  if(!child->parent){
    child->parent = this;
    if((uint)i!=n_children) LOG(0) <<"creating childBranch #" <<i <<" without lower branches first";
    n_children++;
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

void rai::ComputeNode::data(rai::Graph& g){
  if(c>0.) g.add<double>("c", c);
  if(l>0.) g.add<double>("l", l);
}
