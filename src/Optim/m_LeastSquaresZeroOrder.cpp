#include "m_LeastSquaresZeroOrder.h"
#include "../Core/util.h"

LeastSquaredZeroOrder::LeastSquaredZeroOrder(shared_ptr<NLP> P, const arr& x_init) : P(P){
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

  //-- propose step
  arr delta;
  if(!hasLinTerms){
    delta = inverse_SymPosDef(~J*J + damping*eye(J.d1)) * (~J * phi_x);
  }else{
    arr phi_sos, phi_f, Jsos, Jf;
    std::tie(phi_sos, phi_f, Jsos, Jf) = decompose(phi_x, J, P->featureTypes);
    delta = inverse_SymPosDef(~Jsos*Jsos + damping*eye(Jsos.d1)) * (~Jsos * phi_sos + 0.5 * sum(Jf,0));
  }
  // delta /= length(delta);

  arr y = x - alpha*delta;

  //noise
  if(method=="linReg"){
    double sigma = .2*alpha*length(delta);
    // sigma *= 1./(1.+rai::sqr(.001*steps));
    y += sigma*randn(P->dimension);
  }else if(method=="rank1"){
    // double sigma = .2*alpha*length(delta);
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

  if(steps>2){
    if(method=="rank1"){
      updateJ_rank1(J, x, data_X[-2], phi_x, data_Phi[-2]);

    }else if(method=="linReg"){
      updateJ_linReg(J, data_X, data_Phi);
    }

  }

  //-- reject
  if(phi2_y>=phi2_x){
    rejectSteps++;
    alpha *= .5;
    if(alpha<alpha_min) alpha = alpha_min;
    cout <<" -- reject (alpha: " <<alpha <<")" <<endl;
  }else{
    rejectSteps=0;
    if(length(x-y)<1e-3) tinySteps++; else tinySteps=0;
    x = y;
    phi_x = phi_y;
    phi2_x = phi2_y;
    alpha *= 1.2;
    if(alpha>1.) alpha=1.;
    cout <<" -- accept (alpha: " <<alpha <<")" <<endl;
  }

  if(steps>500) return true;
  if(rejectSteps>3*x.N) return true;
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
    double K = 1.;
    if(n > K*d){ //get weights
      W = data_X - match(~x, data_X.dim());
      W = sum(sqr(W), 1);
      arr Wcopy = W;
      double W0 = Wcopy.nthElement_nonConst(K*d) + 1e-6;
      W = exp(-W/W0);
    }
  }
  if(false){
    arr D = ~X*(W%X);
    arr s;
    lapack_EigenDecomp(D, s, NoArr);
    cout <<" [log(det(data)): " <<sum(log(s))/double(s.N) <<']' <<std::flush;
  }
  double lambdaReg = 1e-6;
  J = ~Y*(W%X) * inverse_SymPosDef(~X*(W%X) + lambdaReg*I);
  J.delColumns(0);
}
