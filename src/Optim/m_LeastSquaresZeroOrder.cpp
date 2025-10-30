#include "m_LeastSquaresZeroOrder.h"
#include "m_EvoStrategies.h"
#include "../Core/util.h"

/* ideas:

stepMax
covariant noise
*/

namespace rai {

void zero_non_sos_objectives(const ObjectiveTypeA& featureTypes, arr& phi){
  CHECK_EQ(featureTypes.N, phi.N, "");
  for(uint i=0;i<phi.N;i++) if(featureTypes.elem(i)!=OT_sos) phi.elem(i)=0.;
}

auto decompose(const arr& phi, const arr& J, ObjectiveTypeA featureTypes){
  uintAA I(OT_sos+1);
  for(uint i=0;i<featureTypes.N;i++) I(featureTypes.elem(i)).append(i);

  arr phi_sos = phi.pick(I(OT_sos));
  arr phi_f = phi.pick(I(OT_f));
  arr Jsos = J.pick(I(OT_sos));
  arr Jf = J.pick(I(OT_f));
  return std::tuple(phi_sos,phi_f, Jsos, Jf);
}

LeastSquaredZeroOrder::LeastSquaredZeroOrder(shared_ptr<NLP> P, const arr& x_init, std::shared_ptr<OptOptions> _opt)
 : P(P), opt(_opt){
  if(x_init.N) x=x_init;
  else x=P->getInitializationSample();
  CHECK_EQ(x.N, P->dimension, "");

  // for(auto& ot: P->featureTypes){
  //   if(ot==OT_f) hasLinTerms=true;
  //   else CHECK_EQ(ot, OT_sos, "");
  // }

  data_X.resize(0, P->dimension);
  data_Phi.resize(0, P->featureTypes.N);

  J.resize(data_Phi.d1, data_X.d1). setZero();
  phi0.resize(data_Phi.d1) .setZero();
  Hinv.setId(P->dimension);
  Hinv /= damping;

  alpha = .5;
  // method="rank1";
  method="linReg";

  //evaluate
  P->evaluate(phi_x, NoArr, x);
  phi2_x = sumOfSqr(phi_x);
  cout <<"--lszo-- " <<steps <<" f: " <<phi2_x <<std::endl;
}

arr LeastSquaredZeroOrder::generateNewSamples(){
#if 0
  if(elite_X.d0){
    //uint pick = rnd.uni_int(0,elite_X.d0-1);
    uint pick = 0;
    x = elite_X[pick];
    phi_x = elite_Phi[pick];
    // phi2_x = sumOfSqr(phi_x);
  }
#endif

  //-- compute delta
  arr delta;
  Hinv = inverse_SymPosDef(~J*J + damping*eye(J.d1));
  delta = Hinv * (~J * phi_x); //(phi0+J*x));

  delta *= -alpha;

  //-- restrict stepsize
  double maxDelta = absMax(delta);
  if(opt->stepMax>0. && maxDelta>opt->stepMax) {
    delta *= opt->stepMax/maxDelta;
    maxDelta = opt->stepMax;
  }

  //-- proposed step
  arr y_mean = x + delta;

  arr X(lambda, y_mean.N);

  for(uint k=0;k<lambda;k++){
    arr y = X[k];
    y = y_mean;

    //-- add noise
    if(method=="linReg"){
      arr z;
      if(covariantNoise){
        arr C;
        double Hsig2 = trace(Hinv)/double(Hinv.d0);
        lapack_cholesky(C, Hinv);
        z = C * randn(P->dimension);
        z *= noiseRatio/sqrt(Hsig2)*length(delta);
        // z *= noiseRatio * length(delta);
        // z *= noiseRatio * alpha;
      }else{
        z = randn(x.N);
      z *= noiseRatio * length(delta);
      }
      // sigma *= 1./(1.+sqr(.001*steps));
      // sigma += noiseAbs;
      y += z;
    }else if(method=="rank1"){
      // double sigma = .2*length(delta);
      double sigma = 1e-2;
      y += sigma*randn(P->dimension);
    }
  }

  return X;
}

void LeastSquaredZeroOrder::update(arr& X, arr& Phi){
  for(uint i=0;i<Phi.d0;i++) zero_non_sos_objectives(P->featureTypes, Phi[i].noconst());

  //store
  data_X.append(X);
  data_Phi.append(Phi);

  //prune data
  if(pruneData){
    uint n = data_X.d0;
    uint N = 2.*dataRatio*data_X.d1;
    if(n > N){
      data_X.delRows(0, n-N);
      data_Phi.delRows(0, n-N);
      CHECK_EQ(data_X.d0, N, "");
      CHECK_EQ(data_Phi.d0, N, "");
    }
  }

  //update J
  if(steps>2){
    if(method=="rank1"){
      updateJ_rank1(J, x, data_X[-2], phi_x, data_Phi[-2]);

    }else if(method=="linReg"){
      std::tie(J, phi0) = updateJ_linReg(data_X, data_Phi, x, dataRatio);
    }
  }

  //update Hinv
  // if(steps>2){
  //   arr Hinv_now = inverse_SymPosDef(~J*J + damping*eye(J.d1));
  //   if(steps<20){
  //     Hinv = Hinv_now;
  //   }else{
  //     double beta=.1;
  //     Hinv = (1.-beta) * Hinv + beta*Hinv_now;
  //   }
  // }

  if(elite_X.N){ X.append(elite_X); Phi.append(elite_Phi); }

  arr Phi2 = ::sum(::sqr(Phi), 1);
  uintA picks = pick_best_mu(X, Phi2, mu);
  elite_X = X.pick(picks);
  elite_Phi = Phi.pick(picks);

  arr y = elite_X[0];
  arr phi_y = elite_Phi[0];
  double phi2_y = sumOfSqr(phi_y);
  cout <<"--lszo-- " <<steps <<" f: " <<phi2_y <<std::flush;

  //-- update x (reject?)
  if(phi2_y>=phi2_x){
    rejectedSteps++;
    alpha *= stepDec;
    if(alpha<alpha_min) alpha = alpha_min;
    cout <<" -- reject (alpha: " <<alpha <<")" <<endl;
  }else{
    rejectedSteps=0;
    if(length(x-y)<1e-4) tinySteps++; else tinySteps=0;
    x = y;
    phi_x = phi_y;
    phi2_x = phi2_y;
    alpha *= stepInc;
    if(alpha>1.) alpha=1.;
    cout <<" -- accept (alpha: " <<alpha <<")" <<endl;
  }
}

bool LeastSquaredZeroOrder::step(){
  steps++;

  arr X = generateNewSamples();

  //-- evaluate
  arr Phi(lambda, P->featureTypes.N);
  for(uint k=0;k<X.d0;k++){
    P->evaluate(Phi[k].noconst(), NoArr, X[k]);
  }

  update(X, Phi);

  if((int)steps>maxIters) return true;
  if(rejectedSteps>3*x.N) return true;
  if(tinySteps>5) return true;
  return false;
}

void LeastSquaredZeroOrder::updateJ_rank1(arr& J, const arr& x, const arr& x_last, const arr& phi, const arr& phi_last){
  arr y = phi - phi_last;
  arr d = x - x_last;
  double d2 = sumOfSqr(d)+1e-3;
  double alpha = .5;
  J = J * (eye(J.d1) - (alpha/d2) * (d^d));
  J += (alpha/d2) * (y^d);
}

std::tuple<arr,arr> updateJ_linReg(const arr& Xraw, const arr& Y, const arr& x_now, double dataRatio){
  uint n=Xraw.d0, d=Xraw.d1;
  arr X = catCol({ones(n,1), Xraw});
  arr I = eye(d+1);// I(0,0)=0.;
  arr W = ones(n);
  if(true){
    double K = dataRatio;
    if(n > K*d){ //get weights
      W = Xraw - match(~x_now, Xraw.dim());
      W = ::sum(::sqr(W), 1);
      arr Wcopy = W;
      double W0 = Wcopy.nthElement_nonConst(K*d) + 1e-6;
      W = exp(-W/W0);
    }
  }
  if(false){
    arr D = ~X*(W%X);
    arr s;
    lapack_EigenDecomp(D, s, NoArr);
    cout <<" [log(det(data)): " <<::sum(log(s))/double(s.N) <<']' <<std::flush;
  }
  double lambdaReg = 1e-6;
  arr J = ~Y*(W%X) * inverse_SymPosDef(~X*(W%X) + lambdaReg*I);
  arr phi0 = J.col(0);
  J.delColumns(0);
  return std::tuple(J, phi0);
}

//===========================================================================

LSGaussEDA::LSGaussEDA(shared_ptr<NLP> P, const arr& x_init, std::shared_ptr<OptOptions> _opt)
    : P(P), opt(_opt){

  mean = x_init;
  CHECK_EQ(mean.N, P->dimension, "");

  cov = rai::sqr(sigmaInit)*eye(mean.N);
}

arr LSGaussEDA::generateNewSamples(){
  arr C;
  lapack_cholesky(C, cov);
  arr z = randn(lambda, mean.N) * ~C;

  arr X = replicate(mean, lambda);
  X += z;
  return X;
}

void LSGaussEDA::update(arr& X, arr& Phi){
  for(uint i=0;i<Phi.d0;i++) zero_non_sos_objectives(P->featureTypes, Phi[i].noconst());

  if(elite_X.N){ X.append(elite_X); Phi.append(elite_y); }

  arr Phi2 = ::sum(::sqr(Phi), 1);
  uintA picks = pick_best_mu(X, Phi2, mu);
  elite_X = X.pick(picks);
  elite_y = Phi.pick(picks);

  //linreg:
  arr J, phi0;
  std::tie(J, phi0) = updateJ_linReg(X, Phi, mean, 10.);

  //update GN-mu
  arr H = ~J*J;
  arr Hinv = inverse_SymPosDef(H + 1e-3*eye(H.d0));
  arr mu_GN = - Hinv * (~J * phi0);

#if 1
  mean += .1 * (mu_GN - mean);
  cov = (1-beta) * cov + beta * Hinv;
#else
  //update cov
  arr H = inverse_SymPosDef(cov);
  double beta2=.5;
  H += beta2*(~J*J);
  cov = inverse_SymPosDef(H);

  X = elite_X;

  arr meanX = ::mean(X);
  arr covY = covar(X);

  arr delta = meanX-mean;

  if(length(delta)<1e-4) tinySteps++; else tinySteps=0;

  mean = meanX + momentum*delta;

  cov = (1.-beta)*cov + beta*covY;
  cov += .4 * (delta*~delta);
#endif

  double sig2 = trace(cov);

  cout <<"--GaussEDA-- " <<steps <<" sig2: " <<sig2 <<endl;
}

bool LSGaussEDA::step(){
  steps++;

  arr X = generateNewSamples();

  //-- evaluate
  arr Phi(lambda, P->featureTypes.N);
  for(uint k=0;k<X.d0;k++){
    P->evaluate(Phi[k].noconst(), NoArr, X[k]);
  }

  update(X, Phi);

  if((int)steps*lambda>1000) return true;
  if(rejectedSteps>3*mean.N) return true;
  if(tinySteps>5) return true;
  return false;
}
} //namespace
