#include "solver_PlainMC.h"
#include <Core/array-vector.h>

PlainMC::PlainMC(MCTS_Environment& world)
  : world(world), verbose(2){
  reset();
}

void PlainMC::reset(){
  A = ARRAY(world.get_actions());
  D.clear();
  D.resize(A.N);
}

void PlainMC::addRollout(int stepAbort){
  int step=0;
  world.reset_state();
  double R=0.;

  // random first choice
  uint a = rnd(A.N);
  if(verbose>1) cout <<"****************** MC: first decision:" <<*A(a) <<endl;
  R += world.transition(A(a)).second;

  //-- rollout
  while(!world.is_terminal_state() && (stepAbort<0 || step++<stepAbort)){
    if(verbose>1) cout <<"****************** MC: random decision" <<endl;
    R += world.transition_randomly().second;
  }

  if(step>=stepAbort) R -= 100.;
  if(verbose>0) cout <<"****************** MCTS: terminal state reached; step=" <<step <<" Return=" <<R <<endl;

  //-- collect data
  D(a).X.append(R);
}

void PlainMC::report(){
  for(uint a=0;a<A.N;a++){
    sort(D(a).X);
    arr& X=D(a).X;
    cout <<"action=" <<*A(a) <<" returns=" <<(X.N>10?X.sub(-10,-1):X) <<endl;
  }
}

MCTS_Environment::Handle PlainMC::getBestAction(){
  arr Q(A.N);
  for(uint a=0;a<A.N;a++){
    Q(a) = D(a).X.max();
  }
  return A(Q.maxIndex());
}
