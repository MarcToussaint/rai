/*  ------------------------------------------------------------------
    Copyright (c) 2011-2020 Marc Toussaint
    email: toussaint@tu-berlin.de

    This code is distributed under the MIT License.
    Please see <root-path>/LICENSE for details.
    --------------------------------------------------------------  */

#include "RidgeRegression.h"
#include "../Core/util.h"

arr beta_true;

double NormalSdv(const double& a, const double& b, double sdv) {
  double d=(a-b)/sdv;
  double norm = 1./(::sqrt(RAI_2PI)*sdv);
  return norm*::exp(-.5*d*d);
}

//===========================================================================

arr ridgeRegression(const arr& X, const arr& y, double lambda, arr& bayesSigma, const arr& weighted, arr& zScores) {
  if(lambda<0.) lambda = rai::getParameter<double>("lambda", 1e-10);

  CHECK((y.nd==1 || y.nd==2) && X.nd==2 && y.d0==X.d0, "wrong dimensions");
  arr Xt = ~X;
  if(!!weighted) Xt = Xt % weighted;
  arr XtX = Xt*X;
  for(uint i=1; i<XtX.d0; i++) XtX(i, i) += lambda;
  XtX(0, 0) += 1e-10; //don't regularize beta_0 !!
  arr beta = lapack_Ainv_b_sym(XtX, Xt*y);
  if(!!bayesSigma) {
    lapack_inverseSymPosDef(bayesSigma, XtX);
    double sigma2 = sqrt(sumOfSqr(X*beta-y)/(X.d0-1));
    bayesSigma *= sigma2;
  }
  if(!!zScores) {
    zScores.resize(beta.N);
    double sigma = sumOfSqr(X*beta-y)/(y.N - X.d1 - 1.);
    arr XtXinv;
    lapack_inverseSymPosDef(XtXinv, XtX);
    for(uint i=0; i<beta.N; i++) {
      zScores(i) = fabs(beta(i)) / (sigma * sqrt(XtXinv(i, i)));
    }
  }
  return beta;
}

//===========================================================================

arr evaluateBayesianRidgeRegressionSigma(const arr& X, const arr& bayesSigma) {
  arr s(X.d0);
  for(uint i=0; i<s.N; i++) s.elem(i) = (~X[i] * bayesSigma * X[i]).scalar();
  return s;
}

//===========================================================================

RidgeRegression::RidgeRegression(const arr& X, const arr& y, double lambda, const arr& weighted, int verbose) {
  if(lambda<0.) lambda = rai::getParameter<double>("lambda", 1e-10);
  CHECK((y.nd==1 || y.nd==2) && X.nd==2 && y.d0==X.d0, "wrong dimensions");

  arr Xt = ~X;
  if(!!weighted) Xt = Xt % weighted;
  XtX_I = Xt*X;
  for(uint i=1; i<XtX_I.d0; i++) XtX_I(i, i) += lambda;
  XtX_I(0, 0) += 1e-10; //don't regularize beta_0 !!
  beta = lapack_Ainv_b_sym(XtX_I, Xt*y);
  sigmaSqr = sumOfSqr(X*beta-y)/double(y.N/*-beta.N*/); //beta.N are the degrees of freedom that we substract (=1 for const model)
  if(verbose>0) {
    cout <<"Ridge Regression: #data=" <<X.d0 <<" #features=" <<X.d1 <<" #outputs=" <<(y.nd==2?y.d1:1) <<endl;
    cout <<"   mean error (sdv)=" <<sqrt(sigmaSqr) <<endl;
    if(y.nd==2)
      cout <<"   multi-output mean errors (sdv)=" <<sqrt(getMultiOutputSquaredErrors(X, y)) <<endl;
  }
}

arr RidgeRegression::getBetaSigmaMatrix() {
  lapack_inverseSymPosDef(betaSigmaMatrix, XtX_I);
  betaSigmaMatrix *= sigmaSqr;
  return betaSigmaMatrix;
}

arr RidgeRegression::getBetaZscores() {
  arr zScores(beta.N);
  arr betaSigmaMatrix = getBetaSigmaMatrix();
  for(uint i=0; i<beta.N; i++) zScores(i) = fabs(beta(i)) / ::sqrt(betaSigmaMatrix(i, i));
  return zScores;
}

arr RidgeRegression::getMultiOutputSquaredErrors(const arr& X, const arr& y) {
  arr err = X*beta-y;
  err *= err; //elem-wise
  return sum(err, 0)/double(X.d0/*-X.d1*/); //beta.N are the degrees of freedom that we substract (=1 for const model)
}

arr RidgeRegression::evaluate(const arr& X, arr& bayesSigma2) {
  if(!!bayesSigma2) {
    bayesSigma2.resize(X.d0);
    if(!betaSigmaMatrix.N) getBetaSigmaMatrix();
    for(uint i=0; i<X.d0; i++) bayesSigma2(i) = (~X[i] * betaSigmaMatrix * X[i]).scalar();
  }
  return X*beta;
}

//===========================================================================

double DefaultKernelFunction::k(const arr& x1, const arr& x2, arr& gx1, arr& Hx1) {
  if(!type) {
    type = (KernelType) rai::getParameter<uint>("ML/KernelType", 1);
    switch(type) {
      case readFromCfg: HALT("???");  break;
      case Gauss:
        hyperParam1 = ARR(rai::sqr(rai::getParameter<double>("ML/KernelWidth")));
        hyperParam2 = ARR(rai::sqr(rai::getParameter<double>("ML/PriorSdv")));
        break;
    }
  }
  double k = hyperParam2.scalar()*::exp(-sqrDistance(x1, x2)/hyperParam1.scalar());
  double a = -2.*k/hyperParam1.scalar();
  if(!!gx1) gx1 = a * (x1-x2);
  if(!!Hx1) Hx1 = (-2.*a/hyperParam1.scalar())*((x1-x2)^(x1-x2)) + a*eye(x1.N);
  return k;
};
DefaultKernelFunction defaultKernelFunction;

//===========================================================================

KernelRidgeRegression::KernelRidgeRegression(const arr& X, const arr& y, KernelFunction& kernel, double lambda, double mu)
  :X(X), mu(mu), kernel(kernel) {
  if(lambda<0.) lambda = rai::getParameter<double>("lambda", 1e-10);
  uint n=X.d0;

  //-- compute kernel matrix
  arr kernelMatrix(n, n);
  for(uint i=0; i<n; i++) for(uint j=0; j<i; j++) {
      kernelMatrix(i, j) = kernelMatrix(j, i) = kernel.k(X[i], X[j]);
    }
  for(uint i=0; i<n; i++) kernelMatrix(i, i) = kernel.k(X[i], X[i]);

  kernelMatrix_lambda=kernelMatrix;
  for(uint i=0; i<n; i++) kernelMatrix_lambda(i, i) += lambda;

  //-- compute alpha
  alpha = lapack_Ainv_b_sym(kernelMatrix_lambda, y-mu);

  sigmaSqr = sumOfSqr(kernelMatrix*alpha-y)/double(y.N/*-beta.N*/); //beta.N are the degrees of freedom that we substract (=1 for const model)
}

arr KernelRidgeRegression::evaluate(const arr& Z, arr& bayesSigma2) {
  arr kappa(Z.d0, X.d0);
  for(uint i=0; i<Z.d0; i++) for(uint j=0; j<X.d0; j++) kappa(i, j) = kernel.k(Z[i], X[j]);
  if(!!bayesSigma2) {
    if(!invKernelMatrix_lambda.N) invKernelMatrix_lambda = inverse_SymPosDef(kernelMatrix_lambda);
    bayesSigma2.resize(Z.d0);
    for(uint i=0; i<Z.d0; i++) {
      bayesSigma2(i) = kernel.k(Z[i], Z[i]);
      bayesSigma2(i) -= scalarProduct(kappa[i], invKernelMatrix_lambda*kappa[i]);
    }
  }
  return mu + kappa * alpha;
}

double KernelRidgeRegression::evaluate(const arr& x, arr& g, arr& H, double plusSigma, bool onlySigma) {
  arr kappa(X.d0);
  arr Jkappa(X.d0, x.N);
  arr Hkappa(X.d0, x.N, x.N);
  for(uint i=0; i<X.d0; i++) kappa(i) = kernel.k(x, X[i], Jkappa[i](), Hkappa[i]());

  double fx = 0.;
  if(!!g) g = zeros(x.N);
  if(!!H) H = zeros(x.N, x.N);

  if(!onlySigma) {
    fx += mu + scalarProduct(alpha, kappa);
    if(!!g) g += ~alpha * Jkappa;
    if(!!H) H += ~alpha * Hkappa;
  }

  if(plusSigma) {
//    arr gx, Hx;
//    fx += plusSigma * ;
////    if(!!g) g += plusSigma*(gx + g2);
////    if(!!H) H += plusSigma*(gx + g2);

    if(!invKernelMatrix_lambda.N) invKernelMatrix_lambda = inverse_SymPosDef(kernelMatrix_lambda);

    arr Kinv_k = invKernelMatrix_lambda*kappa;
    arr J_Kinv_k = ~Jkappa*Kinv_k;
    double k_Kinv_k = kernel.k(x, x) - scalarProduct(kappa, Kinv_k);
    fx += plusSigma * ::sqrt(k_Kinv_k);
    if(!!g) g -= (plusSigma/sqrt(k_Kinv_k)) * J_Kinv_k;
    if(!!H) H -= (plusSigma/(k_Kinv_k*sqrt(k_Kinv_k))) * (J_Kinv_k^J_Kinv_k) + (plusSigma/sqrt(k_Kinv_k)) * (~Jkappa*invKernelMatrix_lambda*Jkappa + ~Kinv_k*Hkappa);
  }

  return fx;
}

ScalarFunction KernelRidgeRegression::getF(double plusSigma) {
  return [this, plusSigma](arr& g, arr& H, const arr& x) -> double{
    return this->evaluate(x, g, H, plusSigma, false);
  };
}

//===========================================================================

KernelLogisticRegression::KernelLogisticRegression(const arr& X, const arr& y, KernelFunction& _kernel, double _lambda, double _mu)
  :X(X), lambda(_lambda), mu(_mu), kernel(_kernel) {
  if(lambda<0.) lambda = rai::getParameter<double>("lambda", 1e-10);
  uint n=X.d0;

  //-- compute kernel matrix
  arr kernelMatrix(n, n);
  for(uint i=0; i<n; i++) for(uint j=0; j<i; j++) {
      kernelMatrix(i, j) = kernelMatrix(j, i) = kernel.k(X[i], X[j]);
    }
  for(uint i=0; i<n; i++) kernelMatrix(i, i) = kernel.k(X[i], X[i]);

  //-- iterate Newton steps on training data
  arr f(n), p(n), Z(n), w(n);
  double logLike;
  f.setUni(mu);
  for(uint k=0; k<100; k++) {
    p = exp(f);
    Z = 1.+p;
    p /= Z;
    w = p % (1.-p);

    //compute logLikelihood
    logLike=0.;
    for(uint i=0; i<n; i++) logLike += rai::indicate(y(i)==1.)*f(i) - log(Z(i));
    LOG(1) <<"log-likelihood = " <<logLike;

    kernelMatrix_lambda = kernelMatrix;
    for(uint i=0; i<n; i++) kernelMatrix_lambda(i, i) += 2.*lambda/w(i);

    //compute the update
    arr f_old=f;
    alpha = lapack_Ainv_b_sym(kernelMatrix_lambda, f - (p-y)/w - mu);
    f = mu + kernelMatrix * alpha;
    for(uint i=0; i<f.N; i++) clip(f.elem(i), -100., 100.);  //constrain the discriminative values to avoid NANs...

    if(maxDiff(f, f_old)<1e-5) break;
  }
}

arr KernelLogisticRegression::evaluateF(const arr& Z, arr& bayesSigma2) {
  arr kappa(Z.d0, X.d0);
  for(uint i=0; i<Z.d0; i++) for(uint j=0; j<X.d0; j++) kappa(i, j) = kernel.k(Z[i], X[j]);
  if(!!bayesSigma2) {
    if(!invKernelMatrix_lambda.N) invKernelMatrix_lambda = inverse_SymPosDef(kernelMatrix_lambda);
    bayesSigma2.resize(Z.d0);
    for(uint i=0; i<Z.d0; i++) {
      bayesSigma2(i) = kernel.k(Z[i], Z[i]);
      bayesSigma2(i) -= scalarProduct(kappa[i], invKernelMatrix_lambda*kappa[i]);
    }
  }
  return mu + kappa * alpha;
}

arr KernelLogisticRegression::evaluate(const arr& Z, arr& p_bayes, arr& p_hi, arr& p_lo) {
  arr kappa(Z.d0, X.d0);
  for(uint i=0; i<Z.d0; i++) for(uint j=0; j<X.d0; j++) kappa(i, j) = kernel.k(Z[i], X[j]);
  arr f = mu + kappa * alpha;
  for(uint i=0; i<f.N; i++) clip(f.elem(i), -100., 100.);  //constrain the discriminative values to avoid NANs...
  arr p = exp(f); p/=1.+p;

  if(!!p_bayes || !!p_hi || !!p_lo) { //take sigma of discriminative function to estimate p_bayes, p_up and p_lo
    if(!invKernelMatrix_lambda.N) invKernelMatrix_lambda = inverse_SymPosDef(kernelMatrix_lambda);
    arr s(Z.d0);
    for(uint i=0; i<Z.d0; i++) {
      s(i) = kernel.k(Z[i], Z[i]);
      s(i) -= scalarProduct(kappa[i], invKernelMatrix_lambda*kappa[i]);
    }
    s /= 2.*lambda; //TODO: why?? why not for KRR?
    for(uint i=0; i<s.N; i++) clip(s.elem(i), -100., 100.);  //constrain the discriminative values to avoid NANs...
    if(!!p_bayes) { p_bayes = exp(f/sqrt(1.+s*RAI_PI/8.)); p_bayes /= 1.+p_bayes; }
    s = sqrt(s);
    if(!!p_hi) { p_hi = exp(f+s); p_hi /= 1.+p_hi; }
    if(!!p_lo) { p_lo = exp(f-s); p_lo /= 1.+p_lo; }
  }
  return p;
}

//===========================================================================

arr logisticRegression2Class(const arr& X, const arr& y, double lambda, arr& bayesSigma2) {
  if(lambda<0.) lambda = rai::getParameter<double>("lambda", 1e-10);

  CHECK_EQ(y.nd, 1, "");
  uint n=y.N, d=X.d1;
  arr Xt;
  transpose(Xt, X);

  arr I;
  I.setDiag(lambda, X.d1);
  //I(0, 0)=1e-10; on classification is makes sense to include the bias in regularization, I think... (rescaling one beta only changes the slope of the sigmoid, not the decision boundary)

  arr f(n), p(n), Z(n), w(n), beta_update;
  double logLike, lastLogLike=0., alpha=1.;
  arr beta(d);
  beta.setZero();
  for(uint k=0; k<100; k++) {
    f = X*beta;
    for(uint i=0; i<f.N; i++) clip(f.elem(i), -100., 100.);  //constrain the discriminative values to avoid NANs...
    p = exp(f);
    Z = 1.+p;
    for(uint i=0; i<n; i++) p(i) /= Z(i);
    w = p % (1.-p);

    //compute logLikelihood
    logLike=0.;
    for(uint i=0; i<n; i++) logLike += rai::indicate(y(i)==1.)*f(i) - log(Z(i));
    logLike /= n;
    LOG(1) <<"log-likelihood = " <<logLike;

    //optionally reject the update
    if(k && logLike<lastLogLike) {
      //cout <<"REJECT" <<endl;
      beta -= alpha*beta_update;
      alpha *= .1;
      beta += alpha*beta_update;
      if(alpha*absMax(beta_update)<1e-5) break;
      continue;
    } else {
      alpha = pow(alpha, .8);
    }
    lastLogLike=logLike;

    beta_update = lapack_Ainv_b_sym(Xt*(w%X) + 2.*I, Xt*(y-p) - 2.*I*beta);   //beta update equation
    beta += alpha*beta_update;

//    RAI_MSG("logReg iter= " <<k <<" negLogLike= " <<-logLike/n <<" beta_update= " <<absMax(beta_update) <<" alpha= " <<alpha);

    if(alpha*absMax(beta_update)<1e-5) break;
  }
  if(!!bayesSigma2) {
    lapack_inverseSymPosDef(bayesSigma2, Xt*(w%X) + 2.*I);
  }
  return beta;
}

arr logisticRegressionMultiClass(const arr& X, const arr& y, double lambda) {
  if(lambda<0.) lambda = rai::getParameter<double>("lambda", 1e-10);

  CHECK(y.nd==2 && y.d0==X.d0, "");
  uint n=y.d0, d=X.d1, M=y.d1;
  arr Xt;
  transpose(Xt, X);

  arr XtWX, I;
  I.setDiag(lambda, X.d1);
  I(0, 0)=1e-10;

  arr f(n, M), p(n, M), Z(n), w(n), beta_update;
  double logLike, lastLogLike=0., alpha=1.;
  arr beta(d, M);
  beta.setZero();
  for(uint k=0; k<100; k++) {
    f = X*beta;
    for(uint i=0; i<f.N; i++) clip(f.elem(i), -100., 100.);  //constrain the discriminative values to avoid NANs...
    p = exp(f);
    Z = sum(p, 1);
    for(uint i=0; i<n; i++) p[i]() /= Z(i);
//    w = p % (1.-p);

    //compute logLikelihood
    logLike=0.;
    for(uint i=0; i<n; i++) logLike += scalarProduct(f[i], y[i]) - log(Z(i));

    logLike=0.;
    for(uint i=0; i<n; i++) {
      p[i]() /= sum(p[i]); //normalize the exp(f(x)) along each row
      for(uint c=0; c<M; c++) logLike += y(i, c)*log(p(i, c));
    }

    //optionally reject the update
    if(k && logLike<lastLogLike) {
      //cout <<"REJECT" <<endl;
      beta -= alpha*beta_update;
      alpha *= .1;
      beta += alpha*beta_update;
      if(alpha*absMax(beta_update)<1e-5) break;
      continue;
    } else {
      alpha = pow(alpha, .8);
    }
    lastLogLike=logLike;

    //construct the Hessian matrix as block matrix of size d*M-times-d*M (the beta is of size d*M)
    XtWX.resize(beta.N, beta.N);
    XtWX.setZero();
    for(uint c1=0; c1<M; c1++) for(uint c2=0; c2<M; c2++) {
        for(uint i=0; i<n; i++) w(i) = p(i, c1)*(rai::indicate(c1==c2)-p(i, c2));
        XtWX.setMatrixBlock(Xt*diagProduct(w, X) + 2.*rai::indicate(c1==c2)*I, c1*d, c2*d);
      }
    //compute the beta update
    arr tmp = ~(Xt*(y-p) - 2.*I*beta); //the gradient as M-times-d matrix
    tmp.reshape(M*d);                  //... as one big vector
    beta_update = lapack_Ainv_b_sym(XtWX, tmp); //multiply the inv Hessian
    beta_update.reshape(M, d);         //... as M-times-d matrix
    beta_update = ~beta_update;        //... and back as d-times-M matrix

    beta += alpha*beta_update;

    cout <<"logReg iter= " <<k <<" logLike= " <<logLike/n <<" beta_update= " <<absMax(beta_update) <<" alpha= " <<alpha <<endl;
    if(alpha*absMax(beta_update)<1e-5) break;
  }
  return beta;
}

