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

void GenericDF::update_best(const arr& X, const arr& Phi){
  //-- update best
  arr F = ::sum(::sqr(Phi), 1);
  uint best_i = argmin(F);
  double f_y = F(best_i);

  if(opt->verbose>1) cout <<"--" <<method <<"-- " <<evals <<" f: " <<f_y <<std::flush;

  //-- update x (reject?)
  if(f_y>=best_f){
    rejectedSteps++;
    if(opt->verbose>1) cout <<" -- reject";
  }else{
    rejectedSteps=0;
    if(best_x.N){
      if(length(best_x-X[best_i])<1e-4) tinySteps++; else tinySteps=0;
    }
    best_x = X[best_i];
    best_phi = Phi[best_i];
    best_f = F(best_i);
    if(opt->verbose>1) cout <<" -- accept";
  }
}

arr GenericDF::evaluateSamples(const arr& X){
  arr Phi(X.d0, P->featureTypes.N);
  for(uint k=0;k<X.d0;k++){
    P->evaluate(Phi[k].noconst(), NoArr, X[k]);
    evals++;
  }

  for(uint i=0;i<Phi.d0;i++) zero_non_sos_objectives(P->featureTypes, Phi[i].noconst());

  update_best(X, Phi);

  return Phi;
}

bool GenericDF::step(){

  arr X = generateSamples(lambda);

  arr Phi = evaluateSamples(X);

  update(X, Phi);

  if(opt->verbose>1) cout <<endl;

  if((int)evals>opt->stopEvals) return true;
  if(rejectedSteps>3*P->dimension) return true;
  if(tinySteps>5) return true;
  return false;
}

shared_ptr<SolverReturn> GenericDF::solve(){
  while(!step()){}
  if(opt->verbose>0) cout <<"--" <<method <<" done-- " <<evals <<" f: " <<best_f <<std::endl;
  shared_ptr<SolverReturn> ret = make_shared<SolverReturn>();
  ret->x = best_x;
  arr err = P->summarizeErrors(best_phi);
  ret->f = err(OT_f);
  ret->sos = err(OT_sos);
  ret->eq = err(OT_eq);
  ret->ineq = err(OT_ineq);
  ret->feasible=true;
  return ret;
}

//===========================================================================

auto decompose(const arr& phi, const arr& J, ObjectiveTypeA featureTypes){
  uintAA I(OT_sos+1);
  for(uint i=0;i<featureTypes.N;i++) I(featureTypes.elem(i)).append(i);

  arr phi_sos = phi.pick(I(OT_sos));
  arr phi_f = phi.pick(I(OT_f));
  arr Jsos = J.pick(I(OT_sos));
  arr Jf = J.pick(I(OT_f));
  return std::tuple(phi_sos,phi_f, Jsos, Jf);
}

LeastSquaresDerivativeFree::LeastSquaresDerivativeFree(shared_ptr<NLP> _P, const arr& x_init, std::shared_ptr<OptOptions> _opt)
 : GenericDF("lsdf", _P,_opt){
  if(x_init.N) x=x_init;
  else x=P->getInitializationSample();
  CHECK_EQ(x.N, P->dimension, "");

  best_x = x;

  bool hasF=false, hasSos=false;
  for(auto& ot: P->featureTypes){
    if(ot==OT_f) hasF=true;
    if(ot==OT_sos) hasSos=true;
  }
  CHECK(hasSos, "problem has NO sos terms -- LSDF no applicable")
  if(hasF) LOG(-1) <<"problem has f terms -- will be zeroed";

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
  P->evaluate(best_phi, NoArr, best_x);
  best_f = sumOfSqr(best_phi);
  if(opt->verbose>0) cout <<"--lsdf-- " <<evals <<" f: " <<best_f <<std::endl;
}

arr LeastSquaresDerivativeFree::generateSamples(uint lambda){
  //-- compute delta
  arr delta;
  Hinv = inverse_SymPosDef(~J*J + damping*eye(J.d1));
  delta = Hinv * (~J * (phi0+J*x)); //phi_x

  delta *= -alpha;

  //-- restrict stepsize
  double maxDelta = absMax(delta);
  if(opt->stepMax>0. && maxDelta>opt->stepMax) {
    delta *= opt->stepMax/maxDelta;
    maxDelta = opt->stepMax;
  }

  //-- proposed step
  arr y_mean = x + delta;

  //-- generate lambda noisy versions
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

void LeastSquaresDerivativeFree::update(arr& X, arr& Phi){
  //-- store
  data_X.append(X);
  data_Phi.append(Phi);

  //-- prune data
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

  //-- update J
  if(data_X.d0>2){
    if(method=="rank1"){
      updateJ_rank1(J, x, data_X[-2], best_phi, data_Phi[-2]);

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

  //-- update x
  x = best_x;

  //-- update alpha
  if(rejectedSteps){
    alpha *= stepDec;
    if(alpha<alpha_min) alpha = alpha_min;
  }else{
    alpha *= stepInc;
    if(alpha>1.) alpha=1.;
  }

  if(opt->verbose>1) cout <<"(alpha: " <<alpha <<")";
}

void LeastSquaresDerivativeFree::updateJ_rank1(arr& J, const arr& x, const arr& x_last, const arr& phi, const arr& phi_last){
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

LS_CMA::LS_CMA(shared_ptr<NLP> P, const arr& x_init, std::shared_ptr<OptOptions> _opt)
    : GenericDF("ls-cma", P, _opt), cma(P->f_scalar(), x_init, _opt), lsdf(P, x_init, _opt){
}

arr LS_CMA::generateSamples(uint lambda){
  X_cma = cma.generateNewSamples();
  lsdf.opt->stepMax = 1.*cma.getSigma();
  X_lsdf = lsdf.generateSamples(ls_lambda);

  return (X_cma, X_lsdf);
}

void LS_CMA::update(arr& X, arr& Phi){
  // for(uint i=0;i<Phi.d0;i++) zero_non_sos_objectives(P->featureTypes, Phi[i].noconst());

  arr F = ::sum(::sqr(Phi), 1);

  { //overwrite CMA samples if LSDF better
    for(uint i=0;i<X_lsdf.d0;i++){
      uint j = X_cma.d0+i;
      if(F(j)<F(i)){
        X_cma[i] = X[j];
        F(i) = F(j);
      }
    }
    cma.overwriteSamples(X_cma);
    F.resizeCopy(X_cma.d0);
    cma.update(X_cma, F);
  }

  lsdf.update_best(X,Phi);
  lsdf.update(X, Phi);
  // lsdf.x = cma.getCurrentMean();
  cma.overwriteMean(lsdf.x);
}

//===========================================================================

} //namespace
