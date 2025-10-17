#include "m_LocalGreedy.h"
#include "../Core/util.h"

LocalGreedy::LocalGreedy(shared_ptr<NLP> P, const arr& x_init) : P(P){
  if(x_init.N) x=x_init;
  else x=P->getInitializationSample();
  CHECK_EQ(x.N, P->dimension, "");

  f_x = P->eval_scalar(NoArr, NoArr, x);
  cout <<"--greedy-- " <<steps <<" f: " <<f_x <<endl;
}

bool LocalGreedy::step(){
  arr y = x + sigma*randn(P->dimension);
  double f_y = P->eval_scalar(NoArr, NoArr, y);
  bool accept = (f_y <= f_x);

  steps++;

  cout <<"--greedy-- " <<steps <<" f: " <<f_y <<(accept?"":" -- reject") <<endl;

  if(accept){
    if(f_x-f_y < 1e-4) tinySteps++; else tinySteps=0;
    x = y;
    f_x = f_y;
  }

  if(steps>maxSteps) return true;
  if(tinySteps>10) return true;
  return false;
}
