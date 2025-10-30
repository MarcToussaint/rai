#include "m_EvoStrategies.h"

extern "C" {
#include "CMA/cmaes_interface.h" //by Nikolaus Hansen
}

namespace rai {

//===========================================================================

bool EvolutionStrategy::step(){
  steps++;

  arr samples = generateNewSamples();

  //-- evaluate
  arr y(samples.d0);
  for(uint i=0;i<samples.d0;i++){
    y(i) = f(NoArr, NoArr, samples[i]);
    evals++;
  }

  uint i; double f_y;
  std::tie(f_y, i) = rai::min_arg(y);
  if(f_y<f_x){
    rejectedSteps=0;
    if(x.N && length(x-samples[i])<1e-3) tinySteps++; else tinySteps=0;
    f_x = f_y;
    x = samples[i];
    cout <<" f:" <<f_x <<" -- accept" <<endl;
  }else{
    rejectedSteps++;
    cout <<" f:" <<f_y <<" -- reject" <<endl;
  }

  update(samples, y);

  if(evals>opt->stopEvals) return true;
  if(rejectedSteps>int(3*x.N)) return true;
  if(tinySteps>5) return true;
  return false;
}

std::tuple<arr, arr> select_best_mu(const arr& samples, const arr& values, uint mu){
  CHECK_EQ(samples.d0, values.N, "");
  uintA ranking;
  ranking.setStraightPerm(values.N);
  std::stable_sort(ranking.begin(), ranking.end(),
                   [&values](size_t i1, size_t i2) {return values.p[i1] < values.p[i2];});

  arr X(mu, samples.d1);
  arr y(mu);
  for(uint i=0;i<mu;i++){ X[i] = samples[ranking(i)]; y(i) = values(ranking(i)); }
  return std::tuple(X, y);
}

uintA pick_best_mu(const arr& samples, const arr& values, uint mu){
  CHECK_EQ(samples.d0, values.N, "");
  uintA ranking;
  ranking.setStraightPerm(values.N);
  std::stable_sort(ranking.begin(), ranking.end(),
                   [&values](size_t i1, size_t i2) {return values.p[i1] < values.p[i2];});
  ranking.resizeCopy(mu);
  return ranking;
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

CMAES::CMAES(ScalarFunction f, const arr& x_init, shared_ptr<OptOptions> opt) : EvolutionStrategy(f, x_init, opt) {
  self = make_unique<CMA_self>();
  arr startDev = rai::consts<double>(sigmaInit, x.N);
  cmaes_init(&self->evo, x_init.N, x_init.p, startDev.p, 1, lambda, nullptr);

  cout <<"--cmaes-- " <<evals <<std::endl;
}

CMAES::~CMAES() {
  cmaes_exit(&self->evo);
}

arr CMAES::generateNewSamples(){
  CHECK_EQ(x.N, uint(self->evo.sp.N), "")
  double* const* rgx = cmaes_SamplePopulation(&self->evo);

  // arr samples(self->evo.sp.lambda, self->evo.sp.N);
  // for(uint i=0; i<samples.d0; i++) samples[i].setCarray(rgx[i], samples.d1);

  arr samples;
  samples.setCarray(rgx, self->evo.sp.lambda, self->evo.sp.N);

  cout <<"--cmaes-- " <<evals <<std::flush;
  return samples;
}

void CMAES::overwriteSamples(const arr& Xnew, arr& Xold){
  double **sam = self->evo.rgrgx;
  for(uint i=0;i<Xnew.d0;i++){
    for(uint j=0;j<Xnew.d1;j++){ sam[i][j] = Xnew(i,j);  Xold(i,j)=Xnew(i,j); }
  }
}

void CMAES::update(arr& samples, arr& values){
  CHECK_EQ(values.N, self->evo.sp.lambda, "")
  cmaes_UpdateDistribution(&self->evo, values.p);
}

arr CMAES::getBestEver() {
  return arr(self->evo.rgxbestever, self->evo.sp.N, false);
}

arr CMAES::getCurrentMean() {
  return arr(self->evo.rgxmean, self->evo.sp.N, false);
}


//===========================================================================

arr ES_mu_plus_lambda::generateNewSamples(){
  arr X = replicate(mean, lambda);

  double sig = sigma;
  if(sigmaDecay>0.){
    // plot [0:1000] 1/(1+(.01*x)**2), exp(-(.01*x)**2), exp(-.01*x), 1/(1+.01*x)
    sig *= 1./(1.+rai::sqr(sigmaDecay*steps));
    // sig *= 1./(1.+sigmaDecay*steps);
    // sig *= ::exp(-sigmaDecay*steps);
  }
  rndGauss(X, sig, true);
  cout <<"--es-- " <<evals <<std::flush;
  return X;
}

void ES_mu_plus_lambda::update(arr& Xnew, arr& y){
  if(elite_X.N){ Xnew.append(elite_X); y.append(elite_y); }
  std::tie(Xnew,y) = select_best_mu(Xnew, y, mu);
  arr meanX =  ::mean(Xnew);
  mean = meanX + .5*(meanX-mean);

  elite_X = Xnew;
  elite_y = y;
}

//===========================================================================

GaussEDA::GaussEDA(ScalarFunction f, const arr& x_init, shared_ptr<OptOptions> opt)
    : EvolutionStrategy(f, x_init, opt){
  mean = x;
  cov = rai::sqr(sigmaInit)*eye(x.N);
}

arr GaussEDA::generateNewSamples(){
  arr C;
  lapack_cholesky(C, cov);
  arr z = randn(lambda, x.N) * ~C;

  if(sigmaDecay>0.){
    z *= 1./(1.+rai::sqr(sigmaDecay*steps));
  }

  arr X = replicate(mean, lambda);
  X += z;
  // cout <<"--GaussEDA-- " <<evals <<std::flush;
  return X;
}

void GaussEDA::update(arr& Xnew, arr& y){
  if(elite_X.N){ Xnew.append(elite_X); y.append(elite_y); }

  std::tie(Xnew,y) = select_best_mu(Xnew, y, mu);
  elite_X = Xnew;
  elite_y = y;

  arr meanX = ::mean(Xnew);
  arr covY = covar(Xnew);

  arr delta = meanX-mean;

  mean = meanX + momentum*delta;

  cov = (1.-beta)*cov + beta*covY;
  cov += .4 * (delta*~delta);

  double sig2 = trace(cov);

  cout <<"--GaussEDA-- " <<evals <<" sig2: " <<sig2 <<std::flush;
}

} //namespace
