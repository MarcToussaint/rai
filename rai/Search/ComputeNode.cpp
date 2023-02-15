#include "ComputeNode.h"

void rai::ComputeNode::compute(){
  if(info().verbose>0){
    LOG(0) <<"compute at " <<name <<" ...";
  }
  c_now = -rai::cpuTime();
  untimedCompute();
  c_now += rai::cpuTime();
  c += c_now;
  if(l>1e9) isFeasible=false;
  f_prio = level();
  if(info().verbose>0){
    if(isComplete) LOG(0) <<"computed " <<name <<" -> complete with c:" <<c <<" l:" <<l <<" level:" <<f_prio;
    else LOG(0) <<"computed " <<name <<" -> still incomplete with c:" <<c;
  }
}

std::shared_ptr<rai::TreeSearchNode> rai::ComputeNode::transition(int i){
  auto child = createNewChild(i);
  // update level
  if(!child->parent){
    child->parent = this;
    CHECK_EQ((uint)i, n_children, "really?")
    n_children++;
  }
  child->baseLevel = level();
  child->baseLevel += info().level_eps;
  if(getNumDecisions()==-1){//infinite branching
    child->baseLevel += ::pow(double(i)/info().level_w0, info().level_pw);
  }
  child->f_prio = child->level();
  if(info().verbose>0){
    LOG(0) <<"created node '" <<child->name <<"' ID:" <<child->ID <<" type: '" <<rai::niceTypeidName(typeid(*child)) <<"' baseLevel:" <<child->baseLevel;
  }
  return child;
}
