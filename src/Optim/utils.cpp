/*  ------------------------------------------------------------------
    Copyright (c) 2011-2024 Marc Toussaint
    email: toussaint@tu-berlin.de

    This code is distributed under the MIT License.
    Please see <root-path>/LICENSE for details.
    --------------------------------------------------------------  */

#include "utils.h"
#include "lagrangian.h"
#include "../Core/util.h"

#include <math.h>

//===========================================================================

Conv_ScalarFunction2NLP::Conv_ScalarFunction2NLP(shared_ptr<ScalarFunction> f) : f(f){
  dimension = f->dim;
  featureTypes.resize(1) = OT_f;
}

void Conv_ScalarFunction2NLP::evaluate(arr& phi, arr& J, const arr& x) {
  double y = f->f(J, NoArr, x);
  phi = {y};
  if(!!J) J.reshape(1, x.N);
}

void Conv_ScalarFunction2NLP::getFHessian(arr& H, const arr& x) {
  f->f(NoArr, H, x);
}

//===========================================================================

double Conv_NLP2ScalarProblem::f(arr& g, arr& H, const arr& x) {
  arr phi, J;
  P->evaluate(phi, J, x);

  CHECK_EQ(phi.N, P->featureTypes.N, "");
  CHECK_EQ(phi.N, J.d0, "");
  CHECK_EQ(x.N, J.d1, "");

  double f=0.;
  for(uint i=0; i<phi.N; i++) {
    if(P->featureTypes.p[i]==OT_sos) f += rai::sqr(phi.p[i]);
    else if(P->featureTypes.p[i]==OT_f) f += phi.p[i];
    else HALT("this must be an unconstrained problem!")
    }

  if(!!g) { //gradient
    arr coeff=zeros(phi.N);
    for(uint i=0; i<phi.N; i++) {
      if(P->featureTypes.p[i]==OT_sos) coeff.p[i] += 2.* phi.p[i];
      else if(P->featureTypes.p[i]==OT_f) coeff.p[i] += 1.;
    }
    g = comp_At_x(J, coeff);
    g.reshape(x.N);
  }

  if(!!H) { //hessian: Most terms are of the form   "J^T  diag(coeffs)  J"
    arr coeff=zeros(phi.N);
    double hasF=false;
    for(uint i=0; i<phi.N; i++) {
      if(P->featureTypes.p[i]==OT_sos) coeff.p[i] += 2.;
      else if(P->featureTypes.p[i]==OT_f) hasF=true;
    }
    arr tmp = J;
    if(!isSparseMatrix(tmp)) {
      for(uint i=0; i<phi.N; i++) tmp[i] *= sqrt(coeff.p[i]);
    } else {
      arr sqrtCoeff = sqrt(coeff);
      tmp.sparse().rowWiseMult(sqrtCoeff);
    }
    H = comp_At_A(tmp); //Gauss-Newton type!

    if(hasF) { //For f-terms, the Hessian must be given explicitly, and is not \propto J^T J
      arr fH;
      P->getFHessian(fH, x);
      if(fH.N) H += fH;
    }

    if(!H.special) H.reshape(x.N, x.N);
  }

  return f;
}

//===========================================================================

Conv_NLP_SlackLeastSquares::Conv_NLP_SlackLeastSquares(std::shared_ptr<NLP> _P) : P(_P) {
  dimension = P->dimension;
  bounds = P->bounds;

  //pick constraints
  for(uint i=0; i<P->featureTypes.N; i++) {
    ObjectiveType f = P->featureTypes(i);
    if(f==OT_eq || f==OT_ineq) pick.append(i);
  }
  featureTypes.resize(pick.N) = OT_sos;
}

void Conv_NLP_SlackLeastSquares::evaluate(arr& phi, arr& J, const arr& x) {
  arr Pphi, PJ;
  P->evaluate(Pphi, PJ, x);
  phi = Pphi.pick(pick);
  J = PJ.pick(pick);
  for(uint i=0; i<pick.N; i++) {
    if(P->featureTypes(pick(i))==OT_ineq) {
      if(phi(i)<0.) { phi(i)=0.; J[i]=0.; } //ReLu for g
    } else if(P->featureTypes(pick(i))==OT_eq) {
      if(phi(i)<0.) { phi(i)*=-1.; J[i]*=-1.; } //make positive
    } else {
      NIY;
    }
  }
}

//===========================================================================

NLP_LinTransformed::NLP_LinTransformed(std::shared_ptr<NLP> _P, const arr& _A, const arr& _b) : P(_P), A(_A), b(_b) {
  CHECK_EQ(A.d0, P->dimension, "");
  CHECK_EQ(b.N, P->dimension, "");
  dimension = A.d1;
  featureTypes = P->featureTypes;
//  Ainv = inverse(A);
  bounds = zeros(2, dimension);
//  bounds[0] = Ainv*(P->bounds[0]-b);
//  bounds[1] = Ainv*(P->bounds[1]-b);
}

arr NLP_LinTransformed::getInitializationSample(){
  arr x = P->getInitializationSample();
  return Ainv * x;
}

void NLP_LinTransformed::evaluate(arr& phi, arr& J, const arr& x) {
  arr y = A*x+b;
  P->evaluate(phi, J, y);
  J = J*A;
}


//===========================================================================
//
// helpers
//

void accumulateInequalities(arr& y, arr& J, const arr& yAll, const arr& JAll) {
  y.resize(1).setZero();
  if(!!J) J.resize(1, JAll.d1).setZero();

  for(uint i=0; i<yAll.N; i++) {
    if(yAll.elem(i)>0.) {
      y.scalar() += yAll.elem(i);
      if(!!J && !!JAll) J[0] += JAll[i];
    }
  }
}

void displayFunction(ScalarFunction& f, bool wait, double lo, double hi) {
  arr X, Y;
  X = rai::grid(2, lo, hi, 100);
  Y.resize(X.d0);
  for(uint i=0; i<X.d0; i++) {
    double fx=f.f(NoArr, NoArr, X[i]);
    Y(i) = ((fx==fx && fx<10.)? fx : 10.);
  }
  Y.reshape(101, 101);
  FILE("z.fct") <<(~Y).modRaw();
//  plot()->Gnuplot();  plot()->Surface(Y);  plot()->update(true);
//  write(LIST<arr>(Y), "z.fct");
  gnuplot("reset; set xlabel 'x'; set ylabel 'y'; splot [-1:1][-1:1] 'z.fct' matrix us ($1/50-1):($2/50-1):3 w l", wait, true);
}

RUN_ON_INIT_BEGIN(optimization)
ObjectiveTypeA::memMove=true;
RUN_ON_INIT_END(optimization)
