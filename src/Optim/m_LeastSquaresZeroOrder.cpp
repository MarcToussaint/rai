#include "m_LeastSquaresZeroOrder.h"
#include "../Core/util.h"

LeastSquaredZeroOrder::LeastSquaredZeroOrder(shared_ptr<NLP> P, const arr& x_init) : P(P){
  if(x_init.N) x=x_init;
  else x=P->getInitializationSample();
  CHECK_EQ(x.N, P->dimension, "");

  for(auto& ot: P->featureTypes) CHECK_EQ(ot, OT_sos, "");

  data_X.resize(0, P->dimension);
  data_Phi.resize(0, P->featureTypes.N);
  data_Phi2.resize(0, P->featureTypes.N);

  J.resize(data_Phi.d1, data_X.d1). setZero();

  sigma = 1e-4;
  // method="rank1";
  method="linReg";
}

bool LeastSquaredZeroOrder::step(){
  //noise
  if(steps>2){
    sigma = .1*length(x-data_X[-1]);
  }
  x += sigma*randn(P->dimension);

  //evaluate
  P->evaluate(phi_x, NoArr, x);
  phi2_x = sumOfSqr(phi_x);

  cout <<"--lszo-- " <<steps <<" f: " <<phi2_x <<endl;

  //store
  data_X.append(x);
  data_Phi.append(phi_x);
  data_Phi2.append(sqr(phi_x));

  if(steps>2){
    if(method=="rank1"){
      updateJ_rank1(J, x, data_X[-2], phi_x, data_Phi[-2]);
      x -= .5*inverse_SymPosDef(~J*J + lambda*eye(J.d1)) * ~J * phi_x;
    }

    if(method=="linReg"){
      // if len(data_X)>0:
      updateJ_linReg(J, data_X, data_Phi);
      x -= .5*inverse_SymPosDef(~J*J + lambda*eye(J.d1)) * ~J * phi_x;
    }
  }

  steps++;
  if(steps>300) return true;
  return false;
}

void LeastSquaredZeroOrder::updateJ_rank1(arr& J, const arr& x, const arr& x_last, const arr& phi, const arr& phi_last){
  arr y = phi - phi_last;
  arr d = x - x_last;
  double d2 = sumOfSqr(d)+1e-2;
  double alpha = 1.;
  J = J * (eye(J.d1) - (alpha/d2) * (d^d));
  J += (alpha/d2) * (y^d);
}

void LeastSquaredZeroOrder::updateJ_linReg(arr& J, const arr& Xraw, const arr& Y, double lambdaReg){
  arr X = rai::catCol({ones(Xraw.d0,1), Xraw});
  arr I = eye(X.d1);
  I(0,0) = 0.;
  arr W=ones(X.d0);
  if(false){ //get weights
    double quantile=.5;
    W = sum(sqr(data_Phi), 1);
    arr Wcopy = W;
    Wcopy.sort();
    double W0 = Wcopy(quantile*W.N);
    W = exp(-W/W0);
    W /= max(W);
  }
  // W = diag(W);
  lambdaReg = 1e-2;
  J = ~Y*(W%X) * inverse_SymPosDef(~X*(W%X) + lambdaReg*I);
  J.delColumns(0);
}
