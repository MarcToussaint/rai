#include "m_LeastSquaresZeroOrder.h"
#include "../Core/util.h"

/* ideas:

stepMax
covariant noise
*/

namespace rai {

LeastSquaredZeroOrder::LeastSquaredZeroOrder(shared_ptr<NLP> P, const arr& x_init, std::shared_ptr<OptOptions> _opt) : P(P), opt(_opt){
  if(x_init.N) x=x_init;
  else x=P->getInitializationSample();
  CHECK_EQ(x.N, P->dimension, "");

  for(auto& ot: P->featureTypes){
    if(ot==OT_f) hasLinTerms=true;
    else CHECK_EQ(ot, OT_sos, "");
  }

  data_X.resize(0, P->dimension);
  data_Phi.resize(0, P->featureTypes.N);

  J.resize(data_Phi.d1, data_X.d1). setZero();

  alpha = .5;
  // method="rank1";
  method="linReg";

  //evaluate
  P->evaluate(phi_x, NoArr, x);
  phi2_x = sumOfSqr(phi_x);
  cout <<"--lszo-- " <<steps <<" f: " <<phi2_x <<std::endl;
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

bool LeastSquaredZeroOrder::step(){
  steps++;

  //-- compute delta
  arr delta;
  arr Hinv = inverse_SymPosDef(~J*J + damping*eye(J.d1));
  if(!hasLinTerms){
    delta = Hinv * (~J * phi_x);
  }else{
    arr phi_sos, phi_f, Jsos, Jf;
    std::tie(phi_sos, phi_f, Jsos, Jf) = decompose(phi_x, J, P->featureTypes);
    delta = Hinv * (~Jsos * phi_sos + 0.5 * ::sum(Jf,0));
  }

  //noise directly on delta
#if 0
  arr C;
  lapack_cholesky(C, Hinv);
  arr z = C * randn(P->dimension);
  delta += noiseRatio*z;
#endif

  delta *= -alpha;

  //-- restrict stepsize
  double maxDelta = absMax(delta);
  if(opt->stepMax>0. && maxDelta>opt->stepMax) {
    delta *= opt->stepMax/maxDelta;
    maxDelta = opt->stepMax;
  }

  //-- proposed step
  arr y = x + delta;

  //-- add noise
  if(method=="linReg"){
    arr z;
    if(covariantNoise){
      arr C;
      double Hsig2 = trace(Hinv)/double(Hinv.d0);
      lapack_cholesky(C, Hinv);
      z = C * randn(P->dimension);
      // z *= noiseRatio/sqrt(Hsig2)*length(delta);
      z *= noiseRatio * length(delta);
      // z *= noiseRatio * alpha;
    }else{
      z = randn(x.N);
      z *= noiseRatio*length(delta);
    }
    // sigma *= 1./(1.+rai::sqr(.001*steps));
    // sigma += noiseAbs;
    y += z;
  }else if(method=="rank1"){
    // double sigma = .2*length(delta);
    double sigma = 1e-2;
    y += sigma*randn(P->dimension);
  }

  //-- evaluate
  arr phi_y;
  P->evaluate(phi_y, NoArr, y);
  double phi2_y = sumOfSqr(phi_y);

  cout <<"--lszo-- " <<steps <<" f: " <<phi2_y <<std::flush;

  //store
  data_X.append(y);
  data_Phi.append(phi_y);

  if(pruneData){
    uint n=data_X.d0;
    uint N = dataRatio*data_X.d1;
    if(n > N){
      data_X.delRows(0, n-N);
      data_Phi.delRows(0, n-N);
      CHECK_EQ(data_X.d0, N, "");
      CHECK_EQ(data_Phi.d0, N, "");
    }
  }

  if(steps>2){
    if(method=="rank1"){
      updateJ_rank1(J, x, data_X[-2], phi_x, data_Phi[-2]);

    }else if(method=="linReg"){
      updateJ_linReg(J, data_X, data_Phi);
    }

  }

  //-- reject
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

void LeastSquaredZeroOrder::updateJ_linReg(arr& J, const arr& Xraw, const arr& Y){
  uint n=Xraw.d0, d=Xraw.d1;
  arr X = rai::catCol({ones(n,1), Xraw});
  arr I = eye(d+1);
  arr W = ones(n);
  if(true){
    double K = dataRatio;
    if(n > K*d){ //get weights
      W = data_X - match(~x, data_X.dim());
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
  J = ~Y*(W%X) * inverse_SymPosDef(~X*(W%X) + lambdaReg*I);
  J.delColumns(0);
}

} //namespace
