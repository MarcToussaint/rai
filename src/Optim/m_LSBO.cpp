#include "m_LSBO.h"
#include "m_EvoStrategies.h"
#include "../Core/util.h"

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

//===========================================================================

LeastSquaresBlackboxOpt::LeastSquaresBlackboxOpt(shared_ptr<NLP> _P, const arr& x_init, std::shared_ptr<OptOptions> _opt)
 : GenericBO("lsdf", _P, x_init, _opt){
  if(x_init.N) x=x_init;
  else x=P->getInitializationSample();
  CHECK_EQ(x.N, P->dimension, "");

  best_x = x;

  bool hasF=false, hasSos=false;
  for(auto& ot: P->featureTypes){
    if(ot==OT_f) hasF=true;
    if(ot==OT_sos) hasSos=true;
  }
  CHECK(hasSos, "problem has NO sos terms -- LSBO no applicable")
  if(hasF) LOG(-1) <<"problem has f terms -- will be zeroed";

  data_X.resize(0, P->dimension);
  data_Phi.resize(0, P->featureTypes.N);

  J.resize(data_Phi.d1, data_X.d1). setZero();
  phi0.resize(data_Phi.d1) .setZero();
  Hinv.setId(P->dimension);
  Hinv /= minDamping;

  alpha = .5;
  beta = minDamping;
  // method="rank1";
  method="linReg";
  data_radius = 0.;

  //evaluate

  // P->evaluate(best_phi, NoArr, best_x);
  // update_best();
  // best_f = sumOfSqr(best_phi);
  // data_X.append(X);
  // data_Phi.append(Phi);
  if(opt->verbose>0) cout <<"--lsdf-- " <<evals <<" f: " <<best_f <<std::endl;
  // arr X = x;
  // X.reshape(1,-1);
  // update(X, evaluateSamples(X));
}

arr LeastSquaresBlackboxOpt::generateSamples(){
  if(data_X.d0==0){
    arr X = x;
    return X.reshape(1,-1);
  }

  //-- compute delta
  arr delta;
  Hinv = inverse_SymPosDef(~J*J + beta*eye(J.d1));
  delta = Hinv * (~J * (phi0+J*x)); //phi_x

  delta *= -1.;
  //-- restrict stepsize
  delta *= alpha;
  double l_delta = length(delta);
  if(opt->stepMax>0. && l_delta >opt->stepMax) {
    delta *= opt->stepMax/l_delta ;
  }
  if(opt->verbose>1) cout <<" (delta: " <<length(delta) <<')';

  //-- proposed step
  arr y_mean = x + delta;

  //-- generate lambda noisy versions
  arr X = match(y_mean, {(uint)lambda, y_mean.N});
  if(data_X.d0>10) {
    X +=  noiseRatio * length(delta) * randn(X.dim());
  } else {
    X += .1 * randn(X.dim());
  }

#if 0
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
      } else
      if(data_X.d0>10) {
        z = randn(x.N);
        z *= noiseRatio * length(delta);
      } else {
        z = .1 * randn(x.N);
      }
      y += z;
    }else if(method=="rank1"){
      // double sigma = .2*length(delta);
      double sigma = 1e-2;
      y += sigma*randn(P->dimension);
    }
  }
#endif

  return X;
}

void LeastSquaresBlackboxOpt::pruneData(){
  uint n = data_X.d0;
  uint N = 2.*fabs(weightRatio)*data_X.d1;
  arr dists(n);
  for(uint i=0;i<n;i++) dists(i) = length(x - data_X[i]);
  uintA order = dists.argsort();
  data_radius = dists.elem(order(n/2));
  if(data_X.d0 == N+1){
    data_X.delRows(dists(-1), 1);
    data_Phi.delRows(dists(-1), 1);
  }else if(data_X.d0 > N){
    data_X = data_X.pick(order({0,N}));
    data_Phi = data_Phi.pick(order({0,N}));
    CHECK_EQ(data_X.d0, N, "");
    CHECK_EQ(data_Phi.d0, N, "");
  }
}

void LeastSquaresBlackboxOpt::update(arr& X, arr& Phi, arr& F){
  for(uint i=0;i<Phi.d0;i++) zero_non_sos_objectives(P->featureTypes, Phi[i].noconst());

  //-- store
  data_X.append(X);
  data_Phi.append(Phi);

  //-- prune data
  pruneData();

  //-- update J
  if(data_X.d0>2){
    if(method=="rank1"){
      updateJ_rank1(J, x, data_X[-2], best_phi, data_Phi[-2]);

    }else if(method=="linReg"){
      std::tie(J, phi0) = updateJ_linReg(data_X, data_Phi, x, weightRatio);
    }
  }

  //update Hinv
  // if(steps>2){
  //   arr Hinv_now = inverse_SymPosDef(~J*J + beta*eye(J.d1));
  //   if(steps<20){
  //     Hinv = Hinv_now;
  //   }else{
  //     double beta=.1;
  //     Hinv = (1.-beta) * Hinv + beta*Hinv_now;
  //   }
  // }

  //-- update x
  x = best_x;

  //-- update alpha
  if(rejectedSteps){
    // alpha *= opt->stepDec;
    // if(alpha<alpha_min) alpha = alpha_min;
    beta *= 1.5;
    if(beta>maxDamping) beta=maxDamping;
  }else{
    // alpha *= opt->stepInc;
    // if(alpha>1.) alpha=1.;
    beta *= 0.5;
    if(beta<minDamping) beta=minDamping;
  }

  if(opt->verbose>1) cout <<" (alpha: " <<alpha <<" beta: " <<beta <<" data_r: " <<data_radius <<')';
}

void LeastSquaresBlackboxOpt::updateJ_rank1(arr& J, const arr& x, const arr& x_last, const arr& phi, const arr& phi_last){
  arr y = phi - phi_last;
  arr d = x - x_last;
  double d2 = sumOfSqr(d)+1e-3;
  double alpha = .5;
  J = J * (eye(J.d1) - (alpha/d2) * (d^d));
  J += (alpha/d2) * (y^d);
}

double squared_exponential_kernel(double d_sqr){
  return exp(-d_sqr);
}

double Epanechnikov_quadratic_kernel(double d_sqr){
  if(d_sqr<1.) return 0.75*(1.-d_sqr);
  return 0.;
}

std::tuple<arr,arr> updateJ_linReg(const arr& Xraw, const arr& Y, const arr& x_now, double weightRatio){
  uint n=Xraw.d0;
  arr W = ones(n);
  if(weightRatio>0.){
    uint N = weightRatio*Xraw.d1;
    if(n > N){ //get weights
      W = Xraw - match(~x_now, Xraw.dim());
      W = ::sum(::sqr(W), 1);
      arr Wcopy = W;
      double w0 = Wcopy.nthElement_nonConst(N) + 1e-6;
      // W = exp(-W/w0);
      // for(double& w:W) w = squared_exponential_kernel(w/w0);
      for(double& w:W) w = Epanechnikov_quadratic_kernel(w/w0);
    }
  }
#if 0
    arr D = ~Xraw*(W%Xraw);
    arr s;
    lapack_EigenDecomp(D, s, NoArr);
    cout <<" [log(det(data)): " <<::sum(log(s))/double(s.N) <<']' <<std::flush;
#endif
  arr X = catCol({ones(n,1), Xraw});
  arr I = 1e-6 * eye(Xraw.d1+1);// I(0,0)=0.;
  arr J = ~Y*(W%X) * inverse_SymPosDef(~X*(W%X) + I);
  arr phi0 = J.col(0);
  J.delColumns(0);
  return std::tuple(J, phi0);
}

//===========================================================================

LS_CMA::LS_CMA(shared_ptr<NLP> P, const arr& x_init, std::shared_ptr<OptOptions> _opt)
    : GenericBO("ls-cma", P, x_init, _opt), cma(P, x_init, _opt), lsdf(P, x_init, _opt){
}

arr LS_CMA::generateSamples(){
  X_cma = cma.generateSamples();
  lsdf.opt->stepMax = 5.*cma.getSigma();
  X_lsdf = lsdf.generateSamples();

  return (X_cma, X_lsdf);
  //return X_cma;
}

void LS_CMA::update(arr& X, arr& Phi, arr& F){
  { //overwrite CMA samples if LSBO better
    bool needsOverwrite=false;
    for(uint i=0;i<X_lsdf.d0;i++){
      uint j = X_cma.d0+i;
      if(F(j)<F(i)){
        F(i) = F(j);
        X_cma[i] = X[j];
        needsOverwrite=true;
      }
    }
    if(needsOverwrite){
      cma.overwriteSamples(X_cma);
    }
    F.resizeCopy(X_cma.d0);
    cma.update(X_cma, NoArr, F);
  }

  lsdf.update_best(X, Phi, F);
  lsdf.x = cma.getCurrentMean(); //determines data-weighting in LWR
  lsdf.update(X, Phi, F);
  lsdf.x = cma.getCurrentMean();
  // cma.overwriteMean(lsdf.x);
}

//===========================================================================

} //namespace
