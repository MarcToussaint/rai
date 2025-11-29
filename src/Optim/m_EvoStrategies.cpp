#include "m_EvoStrategies.h"

namespace rai {

//===========================================================================

void EvolutionStrategy::update_best(const arr& X, const arr& F){
  uint best_i = argmin(F);
  double f_y = F(best_i);

  if(f_y>=best_f){
    rejectedSteps++;
    if(opt->verbose>1) cout <<" f:" <<f_y <<" -- reject" <<endl;
  }else{
    rejectedSteps=0;
    if(best_x.N && length(best_x-X[best_i])<1e-4) tinySteps++; else tinySteps=0;
    best_x = X[best_i];
    best_f = F(best_i);
    if(opt->verbose>1) cout <<" f:" <<best_f <<" -- accept" <<endl;
  }
}

arr EvolutionStrategy::evaluateSamples(const arr& X){
  arr F(X.d0);
  for(uint i=0;i<X.d0;i++){
    F(i) = f(NoArr, NoArr, X[i]);
    evals++;
  }

  update_best(X, F);

  return F;
}

bool EvolutionStrategy::step(){
  steps++;

  if(opt->verbose>1) cout <<"--" <<method <<"-- " <<evals <<std::flush;

  arr X = generateSamples();

  arr F = evaluateSamples(X);

  update(X, F);

  if(evals>opt->stopEvals) return true;
  // if(rejectedSteps>int(3*best_x.N)) return true;
  // if(tinySteps>5) return true;
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
  if(opt->verbose>0) cout <<"--" <<method <<" done-- "<< evals <<' ' <<best_f <<std::endl;

  shared_ptr<SolverReturn> ret = make_shared<SolverReturn>();
  ret->x = best_x;
  ret->f = best_f;
  ret->feasible=true;
  return ret;
}

//===========================================================================

arr ES_mu_plus_lambda::generateSamples(){
  arr X = replicate(mean, lambda);

  double sig = sigma;
  if(sigmaDecay>0.){
    // plot [0:1000] 1/(1+(.01*x)**2), exp(-(.01*x)**2), exp(-.01*x), 1/(1+.01*x)
    sig *= 1./(1.+rai::sqr(sigmaDecay*steps));
    // sig *= 1./(1.+sigmaDecay*steps);
    // sig *= ::exp(-sigmaDecay*steps);
  }
  rndGauss(X, sig, true);
  if(opt->verbose>1) cout <<"--es-- " <<evals <<std::flush;
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
    : EvolutionStrategy("gaussEDA", f, x_init, opt){
  mean = best_x;
  cov = rai::sqr(sigmaInit)*eye(best_x.N);
}

arr GaussEDA::generateSamples(){
  arr C = lapack_cholesky(cov);
  arr z = randn(lambda, best_x.N) * ~C;

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

  if(opt->verbose>1) cout <<"--GaussEDA-- " <<evals <<" sig2: " <<sig2 <<std::flush;
}

} //namespace
