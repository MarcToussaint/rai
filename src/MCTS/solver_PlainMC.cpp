#include "solver_PlainMC.h"
#include <Core/array-vector.h>

PlainMC::PlainMC(MCTS_Environment& world)
  : world(world), gamma(.9), verbose(2), topSize(10){
  reset();
  gamma = world.get_info_value(MCTS_Environment::getGamma);
  MT::FileToken fil("PlainMC.blackList");
  if(fil.exists()){
    blackList.read(fil.getIs());
  }
}

void PlainMC::reset(){
  A = ARRAY(world.get_actions());
  D.clear();
  D.resize(A.N);
}

void topAdd(double y, arr& x, uint topSize){
  if(x.N<topSize) x.insertInSorted(y, MT::greater);
  else if(y>x.last()){
    x.insertInSorted(y, MT::greater);
    x.popLast();
  }
}

void PlainMC::addRollout(int stepAbort){
  int step=0;
  world.reset_state();
  double R=0.;
  double discount=1.;

  MT::String decisionsString;

  // random first choice
  uint a = rnd(A.N);
  if(verbose>1) cout <<"****************** MC: first decision:" <<*A(a) <<endl;
  R += discount * world.transition(A(a)).second;
  decisionsString <<*A(a) <<' ';

  //-- rollout
  while(!world.is_terminal_state() && (stepAbort<0 || step++<stepAbort)){
    if(verbose>1) cout <<"****************** MC: random decision" <<endl;
    discount *= gamma;
#if 0
    R += discount * world.transition_randomly().second;
#else
    std::vector<MCTS_Environment::Handle> actions = world.get_actions();
    uint a = rand()%actions.size();
    R += discount * world.transition(actions[a]).second;
    decisionsString <<*actions[a] <<' ';
#endif
  }

  if(step>=stepAbort) R -= 100.;
  if(verbose>0) cout <<"****************** MC: terminal state reached; step=" <<step <<" Return=" <<R <<endl;

  for(const MT::String& black:blackList){
    if(decisionsString.startsWith(black)){
      if(verbose>0) cout <<"****************** MC: rollout was on BLACKLIST: " <<*black <<endl;
      return;
    }
  }

  //-- collect data
  D(a).n++;
  topAdd(R, D(a).X, topSize);
}

void PlainMC::report(){
  for(uint a=0;a<A.N;a++){
    cout <<"action=" <<*A(a) <<" n=" <<D(a).n <<" returns=" <<D(a).X <<endl;
  }
}

MCTS_Environment::Handle PlainMC::getBestAction(){
  arr Q(A.N);
  double Qmin=0.;
  if(world.get_info(world.hasMinReward)) Qmin = world.get_info_value(world.getMinReward);
  for(uint a=0;a<A.N;a++){
    if(D(a).X.N)
      Q(a) = D(a).X.first();
    else
      Q(a) = Qmin;
  }
  return A(Q.maxIndex());
}
