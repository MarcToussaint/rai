#include "m_EvoStrategies.h"

extern "C" {
#include "CMA/cmaes_interface.h" //by Nikolaus Hansen
}

namespace rai {

//===========================================================================

bool EvolutionStrategy::step(){
  steps++;
  arr samples = generateNewSamples();
  arr y = zeros(samples.d0);
  for(uint i=0;i<samples.d0;i++){
    y(i) = f(NoArr, NoArr, samples[i]);
  }
  uint i; double f_y;
  std::tie(f_y, i) = rai::min_arg(y);
  if(f_y<f_x){
    rejectedSteps=0;
    if(length(x-samples[i])<1e-3) tinySteps++; else tinySteps=0;
    f_x = f_y;
    x = samples[i];
    cout <<" f:" <<f_x <<" -- accept" <<endl;
  }else{
    rejectedSteps++;
    cout <<" f:" <<f_y <<" -- reject" <<endl;
  }

  update(samples, y);

  if(steps>500) return true;
  if(rejectedSteps>3*x.N) return true;
  if(tinySteps>5) return true;
  return false;
}

arr EvolutionStrategy::select(const arr& samples, const arr& y, uint mu){
  uintA ranking;
  ranking.setStraightPerm(y.N);
  std::sort(ranking.begin(), ranking.end(),
                   [&y](size_t i1, size_t i2) {return y.p[i1] < y.p[i2];});

  arr X(mu, samples.d1);
  for(uint i=0;i<mu;i++) X[i] = samples[ranking(i)];
  return X;
}

std::shared_ptr<SolverReturn> EvolutionStrategy::solve(){
  while(!step()){}
  shared_ptr<SolverReturn> ret = make_shared<SolverReturn>();
  ret->x = x;
  ret->f = f_x;
  ret->feasible=true;
  return ret;
}

//===========================================================================

struct CMA_self {
  cmaes_t evo;
};

CMAES::CMAES(ScalarFunction f, const arr& x_init) : EvolutionStrategy(f) {
  self = make_unique<CMA_self>();
  x = x_init;
  arr startDev = rai::consts<double>(sigmaInit, x.N);
  cmaes_init(&self->evo, x_init.N, x_init.p, startDev.p, 1, lambda, nullptr);

  cout <<"--cmaes-- " <<steps <<std::endl;
}

CMAES::~CMAES() {
  cmaes_exit(&self->evo);
}

arr CMAES::generateNewSamples(){
  arr samples(self->evo.sp.lambda, self->evo.sp.N);
  double* const* rgx = cmaes_SamplePopulation(&self->evo);
  for(uint i=0; i<samples.d0; i++) samples[i].setCarray(rgx[i], samples.d1);
  cout <<"--cmaes-- " <<steps <<std::flush;
  return samples;
}

void CMAES::update(const arr& samples, const arr& values){
  cmaes_UpdateDistribution(&self->evo, values.p);
}

arr CMAES::getBestEver() {
  return arr(self->evo.rgxbestever, self->evo.sp.N, false);
}

arr CMAES::getCurrentMean() {
  return arr(self->evo.rgxmean, self->evo.sp.N, false);
}

//===========================================================================

} //namespace
