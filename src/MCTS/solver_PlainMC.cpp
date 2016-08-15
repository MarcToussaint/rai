#include "solver_PlainMC.h"

PlainMC::PlainMC(MCTS_Environment& world)
  : world(world), gamma(.9), verbose(2), topSize(10){
  reset();
  gamma = world.get_info_value(MCTS_Environment::getGamma);
  mlr::FileToken fil("PlainMC.blackList");
  if(fil.exists()){
    blackList.read(fil.getIs());
  }
}

void PlainMC::reset(){
  A = conv_stdvec2arr(world.get_actions());
  if(verbose>1){ cout <<"START decisions: "; listWrite(A); cout <<endl; }
  D.clear();
  D.resize(A.N);
}

void topAdd(double y, arr& x, uint topSize){
  if(x.N<topSize) x.insertInSorted(y, mlr::greater);
  else if(y>x.last()){
    x.insertInSorted(y, mlr::greater);
    x.popLast();
  }
}

void PlainMC::addRollout(int stepAbort){
  int step=0;
  world.reset_state();
  double R=0.;
  double discount=1.;

  mlr::String decisionsString;
  MCTS_Environment::TransitionReturn ret;

  // random first choice
  uint a = rnd(A.N);
  if(verbose>1) cout <<"****************** MC: first decision: " <<*A(a) <<endl;
  decisionsString <<*A(a) <<' ';
  ret = world.transition(A(a));
  R += discount * ret.reward;
  discount *= pow(gamma, ret.duration);  //  discount *= gamma;

  //-- rollout
  while(!world.is_terminal_state() && (stepAbort<0 || step++<stepAbort)){
    mlr::Array<MCTS_Environment::Handle> actions;
    actions = conv_stdvec2arr(world.get_actions()); //WARNING: conv... returns a reference!!
    if(verbose>2){ cout <<"Possible decisions: "; listWrite(actions); cout <<endl; }
    uint a = rand()%actions.N;
    if(verbose>1) cout <<"****************** MC: random decision: " <<*actions(a) <<endl;
    decisionsString <<*actions(a) <<' ';
    ret = world.transition(actions(a));
    R += discount * ret.reward;
    discount *= pow(gamma, ret.duration);    //    discount *= gamma;
  }

  if(step>=stepAbort) R -= 100.;
  if(verbose>0) cout <<"****************** MC: terminal state reached; step=" <<step <<" Return=" <<R <<endl;

  for(const mlr::String& black:blackList){
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
  cout <<"MC Planner report:" <<endl;
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
