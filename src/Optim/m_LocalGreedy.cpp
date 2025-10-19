#include "m_LocalGreedy.h"
#include "../Core/util.h"

LocalGreedy::LocalGreedy(ScalarFunction _f, const arr& x_init) : f(_f){
  x=x_init;
  f_x = f(NoArr, NoArr, x);
  cout <<"--greedy-- " <<steps <<" f: " <<f_x <<endl;
}

bool LocalGreedy::step(){
  arr y = x + sigma*randn(x.N);
  double f_y = f(NoArr, NoArr, y);
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
