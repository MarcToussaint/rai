/*  ------------------------------------------------------------------
    Copyright (c) 2011-2020 Marc Toussaint
    email: toussaint@tu-berlin.de

    This code is distributed under the MIT License.
    Please see <root-path>/LICENSE for details.
    --------------------------------------------------------------  */

#include "gaussianProcess.h"

#define RAI_GP_DEBUG 0

/** prior of 0 */
double const_0(const arr& x, const void* p) {return 0.;}

GaussianProcess::GaussianProcess() :
  mu(0.),
  mu_func(const_0),
  priorP(nullptr),
  obsVar(.1),
  kernelP(nullptr)
{}

/** set Gauss cov function, its parameters, and GP prior
 */
void GaussianProcess::setGaussKernelGP(
  void* _kernelP,
  double _mu) {
  mu = _mu;
  mu_func = const_0;
  priorP = nullptr;
  kernelP=_kernelP;
  cov=GaussKernel;
  dcov=dGaussKernel;
  covF_D=GaussKernelF_D;
  covD_D=GaussKernelD_D;
  covDD_F=GaussKernelDD_F;
  covDD_D=GaussKernelDD_D;
}

/** set Gauss cov function, its parameters, and GP prior
 */
void GaussianProcess::setGaussKernelGP(
  void* _kernelP,
  double(*_mu)(const arr&, const void*),
  void* _priorP) {
  mu_func = _mu;
  priorP = _priorP;
  mu = 0;
  kernelP=_kernelP;
  cov=GaussKernel;
  dcov=dGaussKernel;
  covF_D=GaussKernelF_D;
  covD_D=GaussKernelD_D;
  covDD_F=GaussKernelDD_F;
  covDD_D=GaussKernelDD_D;
}

void GaussianProcess::recompute(const arr& _XX, const arr& _YY) {
  X.referTo(_XX);
  Y.referTo(_YY);
  recompute();
}

void GaussianProcess::recompute() {
  uint i, j, N=Y.N, dN=dY.N;
  arr gram, xi, xj, Mu_func;
  gram.resize(N+dN, N+dN);
  if(!gram.N) return;
  for(i=0; i<N; i++) {
    xi.referToDim(X, i);
    gram(i, i) = cov(kernelP, xi, xi);
    Mu_func.append(mu_func(xi, priorP));
  }
  for(i=1; i<N; i++) {
    xi.referToDim(X, i);
    for(j=0; j<i; j++) {
      xj.referToDim(X, j);
      gram(i, j) = gram(j, i) = cov(kernelP, xi, xj);
    }
  }
  if(dN) { //derivative observations
    for(i=0; i<dN; i++) { xi.referToDim(dX, i); gram(N+i, N+i) = covD_D(dI(i), dI(i), kernelP, xi, xi); }
    for(i=0; i<dN; i++) {
      xi.referToDim(dX, i);
      for(j=0; j<N; j++) {
        xj.referToDim(X, j);
        gram(N+i, j) = gram(j, N+i) = covF_D(dI(i), kernelP, xj, xi);
      }
      for(j=0; j<i; j++) {
        xj.referToDim(dX, j);
        gram(N+i, N+j) = gram(N+j, N+i) = covD_D(dI(i), dI(j), kernelP, xi, xj);
      }
    }
  }
  gram = gram + obsVar * eye(gram.d0);
  inverse_SymPosDef(Ginv, gram);
  if(!dN) {
    if(N) GinvY = Ginv * (Y-Mu_func-mu); else GinvY.clear();
  } else {
    arr Yfull; Yfull.append(Y-Mu_func-mu); Yfull.append(dY);
    GinvY = Ginv * Yfull;
  }
  //cout <<"gram=" <<gram <<" Ginv=" <<Ginv <<endl;
}

void GaussianProcess::appendObservation(const arr& x, double y) {
  uint N=X.d0;
  X.append(x); //append it to the data
  Y.append(y);
  X.reshape(N+1, x.N);
  Y.reshape(N+1);
#if RAI_GP_DEBUG
  arr iG=Ginv;
  recompute();
  double err=maxDiff(iG, Ginv);
  CHECK(err<1e-6, "mis-updated inverse Gram matrix" <<err <<endl <<iG <<Ginv);
#endif
}

void GaussianProcess::appendDerivativeObservation(const arr& x, double y, uint i) {
  uint N=dX.d0;
  dX.append(x); //append it to the data
  dY.append(y);
  dI.append(i);
  dX.reshape(N+1, x.N);
  dY.reshape(N+1);
  dI.reshape(N+1);
}

void GaussianProcess::appendGradientObservation(const arr& x, const arr& nabla) {
  for(uint i=0; i<nabla.N; i++) appendDerivativeObservation(x, nabla(i), i);
}

double GaussianProcess::max_var() {
  return cov(kernelP, ARR(0.), ARR(0.));
}

void GaussianProcess::evaluate(const arr& x, double& y, double& sig, bool calcSig) {
  uint i, N=Y.N, dN=dY.N;
  /*static*/ arr k, xi, Ginvk; //danny: why was there a static
  if(N+dN==0) { //no data
    y = mu_func(x, priorP) + mu;
    sig=::sqrt(cov(kernelP, x, x));
    return;
  }
  if(k.N!=N+dN) k.resize(N+dN);
  for(i=0; i<N; i++) { xi.referToDim(X, i); k(i)=cov(kernelP, x, xi); }
  //derivative observations
  for(i=0; i<dN; i++) { xi.referToDim(dX, i); k(N+i)=covF_D(dI(i), kernelP, x, xi); }

  y = scalarProduct(k, GinvY) + mu_func(x, priorP) + mu;
  if(calcSig) {
    innerProduct(Ginvk, Ginv, k);
    sig = cov(kernelP, x, x) - scalarProduct(k, Ginvk);
    //if(sig<=10e-10) {
    //cout << "---" << endl;
    //cout << "x==" << x << endl;
    //cout << "k==" << k << endl;
    //cout << "Ginv==" << Ginv << endl;
    //cout << "kGinvk==" << scalarProduct(k, Ginvk) << endl;
    //sig=::sqrt(sig);
    //cout << "sig==" << sig << endl;
    //}
    //else
    sig = ::sqrt(sig);
  }
}

double GaussianProcess::log_likelihood() {
  arr gram;
  inverse_SymPosDef(gram, Ginv);
  return (-.5*~Y*GinvY - .5*log(length(gram)) - (X.N+dX.N)/2 * log(2*RAI_PI))(0); // actually a degenerated array of size 1x1
}

/** vector of covariances between test point and N+dN observation points */
void GaussianProcess::k_star(const arr& x, arr& k) {
  uint i, N=Y.N, dN=dY.N;
  arr xi;

  if(k.N!=N+dN) k.resize(N+dN);
  for(i=0; i<N; i++) { xi.referToDim(X, i); k(i)=cov(kernelP, x, xi); }
  for(i=0; i<dN; i++) { xi.referToDim(dX, i); k(N+i)=covF_D(dI(i), kernelP, x, xi); }
}

/** vector of covariances between test point and N+dN  observation points */
void GaussianProcess::dk_star(const arr& x, arr& k) {
  uint i, j, N=Y.N, dN=dY.N, d=x.N;
  arr xi;

  if(k.N!=N+dN) k.resize(N+dN, d);
  for(j=0; j<d; ++j) {
    for(i=0; i<N; i++) {
      xi.referToDim(X, i);
      k(i, j)=covD_F(j, kernelP, x, xi);
    }
    for(i=0; i<dN; i++) {
      xi.referToDim(dX, i);
      k(N+i, j)=covD_D(j, dI(i), kernelP, x, xi);
    }
  }
}

/*****
 *
\[
\nabla f(x) (i) =  \frac{
     \partial (  \vec{k}  K^{-1} Y_{all} ) }{
     \partial x_i }
\]

which translates to
\[
 \frac{
   \partial k(x, \vec{X_j}) }{
   \partial x_i }
   K^{-1} Y_{all},
    for j \in {1...N}            % function value observations
\]
and
\[
 \frac{
   \partial^2 k(x, \vec{X_j}) }{
   \partial x_i  \partial x_l }
   K^{-1} Y_{all},
   j \in {N...dN}           % derivative observations
\]

where
<li>
\(k\) is the covariance function,
<li>
\(k\) is a vector \(1..N+dN\) with \(\vec{k}_j=k(x, \vec{X}j)\),
<li>
\(K\) is the Gram matrix \((\vec{X^TX}-\sigma^2\vec{I})^{-1}\)(augmented with noise),
<li>
\(Y_{all}\) is the vector of all observed responses,
<li>
\(\vec{X_j}\) is the vector of coordinates of the j-th data point,
<li>
\(x_l\) is the the same component in which the derivative in \(X_j\) has been
   observed
<li>
\(K^{-1} Y_{all}\) is a column vector;
<li>
the resulting \(\nabla f(x) \) is vector of the dimensionality of the GP
*
*/
void GaussianProcess::gradient(arr& grad, const arr& x) {
  CHECK(X.N || dX.N, "can't recompute gradient without data");
  CHECK((X.N && x.N==X.d1) || (dX.N && x.N==dX.d1), "dimensions don't match!");
  uint i, d, N=Y.N, dN=dY.N, dim;
  dim = X.d1?X.d1:dX.d1;
  arr dk(dim);
  /*static*/ arr xi, dxi; //danny: why was there a static?
  grad.resize(x.N);
  grad.setZero();
  // take the gradient in the function value observations
  for(i=0; i<N; i++) {
    xi.referToDim(X, i);
    dcov(dk, kernelP, x, xi);
    grad += GinvY(i) * dk;
    //cout << dk << endl;
  }
  // derivative observations
  for(i=0; i<dN; i++) {
    dxi.referToDim(dX, i);
    dk.setZero();
    for(d=0; d<dim; ++d) {
      dk(d) = covD_D(d, dI(i), kernelP, x, dxi);
    }
    grad += GinvY(i+N) * dk;
  }
}

/*****
 * The hessian (of the posterior mean \(f\) ) is a \(d\times d\) matrix of
\[
\frac {
\partial^2 f(\vec{x}) }{
\partial x_i, x_j, x_k} =
\partial^2 ( \vec{k}  K^{-1} Y_{all} ) }{
\partial x_i, x_j, x_k} =
\]

and translates to
\[
 \frac{
   \partial^2 k(x, \vec{X_n}) }{
   \partial x_i, x_j }
   K^{-1} Y_{all},
    n \in {1...N}            % function value observations
\]
and
\[
 \frac{
   \partial^3 k(x, \vec{X_n}) }{
   \partial x_l \partial x_i \partial x_j }
   K^{-1} Y_{all},
   n \in {N...dN}           % derivative observations
\]

where
<li>
\(k\) is the covariance function,
<li>
\(k\) is a vector \(1..N+dN\) with \(\vec{k}_j=k(x, \vec{X}j)\),
<li>
\(K\) is the Gram matrix \((\vec{X^TX}-\sigma^2\vec{I})^{-1}\)(augmented with noise),
<li>
\(Y_{all}\) is the vector of all observed responses,
<li>
\(\vec{X_n}\) is the vector of coordinates of the n-th data point,
<li>
\(x_l\) is the the same component in which the derivative in \(X_n\) has been
   observed
<li>
\(K^{-1} Y_{all}\) is a column vector;
<li>
the resulting \(\nabla f(x) \) is vector of the dimensionality of the GP

see also gradient()

pseudocode:
H:=H(3x3xd)
d:=GP dimension
for n \in \{1..N\}
  for i, j \in \{ 1..d\}
    H_{i, j, n} = covD_D(i, j, ..., \vec{x}, \vec{X_n})
  end
end
for n \in \{N+1..N+dN\}
  for i, j \in \{ 1..d\}
    H_{i, j, n} = covDD_D(i, j, dI(n), ..., \vec{x}, \vec{X_n})
  end
end
*
*/
void GaussianProcess::hessianPos(arr& hess, const arr& x) {
  //Danny: I think that this is wrong.. Or at least numerical Hessian checking fails
  CHECK(X.N || dX.N, "can't recompute Hessian without data");
  CHECK((X.N && x.N==X.d1) || (dX.N && x.N==dX.d1), "dimensions don't match!");
  uint i, j, n, N=Y.N, dN=dY.N, dim;
  dim = X.d1?X.d1:dX.d1;
  arr d2k(N+dN, dim, dim);
  /*static*/ arr xn, dxn; //danny: why was there a static
  d2k.setZero();
  hess.resize(dim, dim);
  hess.setZero();
  // function value observations
  for(n=0; n<N; n++) {
    xn.referToDim(X, n);
    for(i=0; i<dim; i++) {
      for(j=0; j<dim; j++) {
        d2k(n, i, j)=covD_D(i, j, kernelP, x, xn);
      }
    }
    //TODO: add inv gram
    hess += GinvY(n) * d2k[n];
  }
  // derivative observations
  for(n=0; n<dN; n++) {
    dxn.referToDim(dX, n);
    for(i=0; i<dim; i++) {
      for(j=0; j<dim; j++) {
        d2k(n, i, j)=covDD_D(i, j, dI(n), kernelP, x, dxn);
      }
    }
    //TODO: add inv gram
    hess += GinvY(n+N) * d2k[n];
  }
}

void GaussianProcess::gradientV(arr& grad, const arr& x) {
  arr k, dk;
  k_star(x, k);
  dk_star(x, dk);
  grad = -2.0*~k*Ginv*dk;
}

void GaussianProcess::evaluate(const arr& X, arr& Y, arr& S) {
  uint i;
  arr xi;
  Y.resize(X.d0); S.resize(X.d0);
  for(i=0; i<X.d0; i++) { xi.referToDim(X, i); evaluate(xi, Y(i), S(i)); }
}
