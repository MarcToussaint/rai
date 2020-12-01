/*  ------------------------------------------------------------------
    Copyright (c) 2011-2020 Marc Toussaint
    email: toussaint@tu-berlin.de

    This code is distributed under the MIT License.
    Please see <root-path>/LICENSE for details.
    --------------------------------------------------------------  */

#include "algos.h"
#include "array.ipp"

#include <cmath>
#include <cstdlib>

#ifdef RAI_SHARK
#  define Array rai::Array
#endif

#ifndef CHECK_EPS
#  define CHECK_EPS 1e-6
#endif

//===========================================================================
//
// functions in rai namespace
//

void rai::normalizeData(arr& X) {
  CHECK_EQ(X.nd, 2, "data has to be a 2D batch");
  uint n, N=X.d0, k, K=X.d1;
  arr mean, var, x, sd;
  arr ones(N); ones=1.;

  innerProduct(mean, ones, X);
  mean/=(double)N;
  for(n=0; n<N; n++) X[n]()-=mean;

  innerProduct(var, ~X, X);
  var/=(double)N;
  sd.resize(K);
  for(k=0; k<K; k++) sd(k)=sqrt(var(k, k));
  for(n=0; n<N; n++) for(k=0; k<K; k++) if(sd(k)) X(n, k)/=sd(k);
}

void rai::makeSpline(arr& X, arr& P, uint intersteps) {
  CHECK_EQ(P.nd, 2, "makeSpline: set of points is 2D array");
  XSpline S;
  S.referTo(P);
  S.type(false, 1.); //is default

  X.resize(1+(P.d0-1)*intersteps, P.d1);
  double tau;
  for(uint t=0; t<X.d0; t++) {
    tau=((double)t)/intersteps;
    S.eval(tau, X[t]());
  }
}

void rai::makeSpline(arr& X, arr& dX, arr& P, uint intersteps) {
  XSpline S;
  S.referTo(P);
  S.type(false, 1.); //is default

  X.resize(1+(P.d0-1)*intersteps, P.d1);
  dX.resizeAs(X);
  double tau;
  for(uint t=0; t<X.d0; t++) {
    tau=((double)t)/intersteps;
    S.eval(tau, X[t](), dX[t]());
  }
}

void rai::randomSpline(arr& X, uint dim, uint points, uint intersteps, double lo, double hi, uint cycles) {
  arr P(points, dim);
  rndUniform(P, lo, hi, false);
  if(cycles>1) P.replicate(cycles);
  P.reshape(points*cycles, dim);
  P[0]() = 0.;
  makeSpline(X, P, intersteps);
}

void rai::randomSpline(arr& X, arr& dX, uint dim, uint points, uint intersteps, double lo, double hi, uint cycles) {
  arr P(points, dim);
  rndUniform(P, lo, hi, false);
  if(cycles>1) P.replicate(cycles);
  P.reshape(points*cycles, dim);
  P[0]() = 0.;
  makeSpline(X, dX, P, intersteps);
}

bool rai::checkGradient(void (*f)(arr&, arr*, const arr&, void*),
                        void* data,
                        const arr& x, double tolerance) {
  arr y, J, dx, dy, JJ;
  f(y, &J, x, data);

  JJ.resize(y.N, x.N);
  double eps=CHECK_EPS;
  uint i, k;
  for(i=0; i<x.N; i++) {
    dx=x;
    dx.elem(i) += eps;
    f(dy, nullptr, dx, data);
    dy = (dy-y)/eps;
    for(k=0; k<y.N; k++) JJ(k, i)=dy.elem(k);
  }
  JJ.reshapeAs(J);
  double md=maxDiff(J, JJ, &i);
  if(md>tolerance) {
    RAI_MSG("checkGradient -- FAILURE -- \nmax diff=" <<md <<" (stored in files z.J and z.JJ)");
    J >>FILE("z.J");
    JJ >>FILE("z.JJ");
    cout <<"\nmeasured grad=" <<JJ <<"\ncomputed grad=" <<J <<endl;
    //HALT("");
    return false;
  } else {
    cout <<"checkGradient -- SUCCESS (max diff error=" <<md <<")" <<endl;
  }
  return true;
}

bool rai::checkGradient(double(*f)(arr*, const arr&, void*),
                        void* data,
                        const arr& x, double tolerance) {
  arr J, dx, JJ;
  double y, dy;
  y=f(&J, x, data);

  JJ.resize(x.N);
  double eps=CHECK_EPS;
  uint i;
  for(i=0; i<x.N; i++) {
    dx=x;
    dx.elem(i) += eps;
    dy = f(nullptr, dx, data);
    dy = (dy-y)/eps;
    JJ(i)=dy;
  }
  JJ.reshapeAs(J);
  double md=maxDiff(J, JJ, 0);
  if(md>tolerance) {
    RAI_MSG("checkGradient -- FAILURE -- \nmax diff=" <<md <<" (stored in files z.J and z.JJ)");
    J >>FILE("z.J");
    JJ >>FILE("z.JJ");
    cout <<"\nmeasured grad=" <<JJ <<"\ncomputed grad=" <<J <<endl;
    //HALT("");
    return false;
  } else {
    cout <<"checkGradient -- SUCCESS (max diff error=" <<md <<")" <<endl;
  }
  return true;
}

void rai::convolution(arr& y, const arr& x, double(*h)(double), double scale) {
  CHECK(x.nd==1 || x.nd==2, "");
  uint T=x.d0, i, j;
  y.resizeAs(x); y.setZero();
  for(i=0; i<T; i++) for(j=0; j<T; j++) {
      if(x.nd==1)
        y(i) += h(RAI_2PI*((double)i-j)/scale)*(double)2./scale * x(j);
      else
        y[i]() += h(RAI_2PI*((double)i-j)/scale)*(double)2./scale * x[j];
    }
}

void rai::bandpassFilter(arr& y, const arr& x, double loWavelength, double hiWavelength) {
  arr y_hi, y_lo;
  convolution(y_lo, x, rai::cosc, hiWavelength);
  convolution(y_hi, x, rai::cosc, loWavelength);
  y=y_hi-y_lo;
}

void rai::bandpassEnergy(arr& y, const arr& x, double loWavelength, double hiWavelength) {
  arr y_his, y_hic, y_los, y_loc, ys, yc;
  convolution(y_los, x, rai::sinc, hiWavelength);
  convolution(y_loc, x, rai::cosc, hiWavelength);
  convolution(y_his, x, rai::sinc, loWavelength);
  convolution(y_hic, x, rai::cosc, loWavelength);
  ys=y_his-y_los;
  yc=y_hic-y_loc;
  uint i;
  y.resizeAs(x);
  for(i=0; i<x.N; i++) {
    y.elem(i) = sqrt(ys.elem(i)*ys.elem(i)+yc.elem(i)*yc.elem(i));
  }
}

/** the Hamming distance between two matrices \c fix and \c fox when
the nodes of \c fox are permuted according to \c p. If they don't
match in size and \c sub=1, then only the upper left submatrices
are compared; if \c sub=0, the missing entries of the smaller
matrix are counted as wrong symbols */
double rai::matdistance(intA& fix, intA& fox, uintA& p, bool sub) {
  CHECK_EQ(fox.d0, p.N, "matrix and its permutation don't agree in size");
  uint i, j, Nmin, Nmax, n=0;
  if(fix.d0<=fox.d0) { Nmin=fix.d0; Nmax=fox.d0; } else { Nmin=fox.d0; Nmax=fix.d0; }
  for(i=0; i<Nmin; i++) for(j=0; j<Nmin; j++) if(fix(i, j)!=fox(p(i), p(j))) n++;
  if(!sub) n+=(Nmax-Nmin)*(Nmax+Nmin);
  //std::cout <<fix <<fox <<n;
  return ((double)n)/Nmax/Nmax;
}

/** same as \c matdistance(A, B, identical-permutation, sub) */
double rai::matdistance(intA& A, intA& B, bool sub) {
  uintA p;
  p.setStraightPerm(B.d0);
  return matdistance(A, B, p, sub);
}

/* Uses simulated annealing to find the permutation \c p such that
\c matdistance(fix, fox, p, sub) becomes minimal */
double rai::matannealing(intA& fix, intA& fox, uintA& p, bool sub, double annealingRepetitions, double annealingCooling) {
  CHECK(fix.nd==2 && fox.nd==2, "");

  uint N=fox.d0;
  p.setStraightPerm(N);
  uintA pStore=p;
  if(fox.N<=1 && fix.N<1) return matdistance(fix, fox, p, sub);

  uint i, j, k;
  unsigned long t;
  double newdist, dist, bestdist=1, temp;

  for(k=0; k<annealingRepetitions; k++) {
    t=N*N/10;
    p.setRandomPerm();
    temp=0.1;
    dist=matdistance(fix, fox, p, sub);
    if(dist<bestdist) { if(dist==0) return 0; bestdist=dist; pStore=p; }
    while(temp>0.001) {
      i=rnd(N); j=rnd(N-1); if(j>=i) j++;
      p.permute(i, j);
      newdist=matdistance(fix, fox, p, sub);
      if(::exp(-(newdist-dist)/temp)>rnd.uni()) {
        dist=newdist;
        if(dist<bestdist) {
          if(dist==0) return 0;
          bestdist=dist;
          pStore=p;
        }
      } else {
        p.permute(i, j);
      }
      if(!(t--)) { t=N*N/10; temp*=annealingCooling; }
    }
  }
  p=pStore;
  return bestdist;
}

//===========================================================================
//
// MonSolver
//

MonSolver::MonSolver() {
  min=-.1; max=.1;
  phase=0;
}
void MonSolver::init(double& par, double wide) {
  phase=0;
  double w=.5*wide*(max-min);
  max+=w;
  min-=w;
  par=min;
}
void MonSolver::solve(double& par, const double& err) {
  if(phase==0) {
    CHECK_EQ(par, min, "not phase 0!");
    if(err>0.) {
      min-=max-min;
      par=min;
      return;
    } else {
      phase=1;
      par=max;
      return;
    }
  }
  if(phase==1) {
    CHECK_EQ(par, max, "not phase 1!");
    if(err<0.) {
      max+=max-min;
      par=max;
      return;
    } else {
      phase=2;
      par=.5*(max+min);
      return;
    }
  }
  if(phase==2) {
    if(err>0.) max=par; else min=par;
    par=.5*(max+min);
  }
}

//===========================================================================
//
// Linear Statistics
//

LinearStatistics::LinearStatistics() { lambda=0.; accum=0.; computed=false; }

/** NOTE: when data is added to the statistics, the means and
variances are not yet normalized and centered (computed indicates that) */
void LinearStatistics::compute() {
  if(computed) return;
  CHECK(accum, "no data accumulated in linear statistics");
  MeanX=meanX/accum;
  MeanY=meanY/accum;
  VarX =(varX  - (meanX^meanX)/accum)/accum;
  CovXY=(covXY - (meanX^meanY)/accum)/accum;
  computed=true;
}

void LinearStatistics::computeZeroMean() {
  if(computed) return;
  CHECK(accum, "no data accumulated in linear statistics");
  MeanX.resizeAs(meanX); MeanX.setZero();
  MeanY.resizeAs(meanY); MeanY.setZero();
  VarX =varX  /(accum*accum);
  CovXY=covXY /(accum*accum);
  computed=true;
}

/** add a single new datum (x and y are 1D arrays)
or a batch of data (x and y are 2D arrays) to the statistics in a weighed fashion */
void LinearStatistics::learn(const arr& X, const arr& Y, double weight) {
  CHECK((X.nd==1 && Y.nd==1) || (X.nd==2 && Y.nd==2), "data must either be batch or online data");
  if(X.nd==1 && Y.nd==1) {
    if(!weight) return;
    computed=false;
    if(!accum) { //first data received
      accum=weight;
      meanX=weight*X;
      meanY=weight*Y;
      varX =weight*(X^X);
      covXY=weight*(X^Y);
      return;
    }
    CHECK(samedim(X, meanX) && samedim(Y, meanY), "PLS: data dimensions differ");
    uint i, j;
    if(lambda>0.) {
      accum=((double)1.-lambda*weight)*accum + weight;
      meanX=((double)1.-lambda*weight)*meanX + weight*X;
      meanY=((double)1.-lambda*weight)*meanY + weight*Y;
      FOR2D(varX, i, j) varX(i, j) =((double)1.-lambda*weight)*varX(i, j)  + weight*X(i)*X(j);
      covXY=((double)1.-lambda*weight)*covXY + weight*(X^Y);
    } else {
      accum+=weight;
      FOR1D(meanX, i) meanX(i)  +=weight*X(i);
      FOR1D(meanY, i) meanY(i)  +=weight*Y(i);
      FOR2D(varX, i, j)  varX(i, j)+=weight*X(i)*X(j);
      FOR2D(covXY, i, j) covXY(i, j)+=weight*X(i)*Y(j);
    }
  }
  if(X.nd==2 && Y.nd==2) {
    CHECK_EQ(X.d0, Y.d0, "need same number of in/out samples");
    arr ones(X.d0); ones=1.;
    computed=false;

    if(!accum) {
      accum=X.d0;
      meanX=ones*X;
      meanY=ones*Y;
      covXY=~X*Y;
      varX =~X*X;
    } else {
      accum+=X.d0;
      meanX+=ones*X;
      meanY+=ones*Y;
      covXY+=~X*Y;
      varX +=~X*X;
    }
  }
}

/// in case y is a scalar
void LinearStatistics::learn(const arr& x, double y, double weight) {
  learn(x, arr(&y, 1), weight);
}

/// in case there is no y
void LinearStatistics::learn(const arr& x) {
  if(x.nd==1) learn(x, arr(0u), 1.); else learn(x, arr(x.d0, 0u), 1.);
}

/// weighted learning
void LinearStatistics::learn(const arr& x, double weight) {
  if(x.nd==1) learn(x, arr(0u), weight); else learn(x, arr(x.d0, 0u), weight);
}

/// forget all data
void LinearStatistics::clear() {
  forget(1);
}

/// forget the current statistics partially
void LinearStatistics::forget(double lambda) {
  computed=false;
  accum=((double)1.-lambda)*accum;
  meanX=((double)1.-lambda)*meanX;
  meanY=((double)1.-lambda)*meanY;
  varX =((double)1.-lambda)*varX;
  covXY=((double)1.-lambda)*covXY;
}

uint LinearStatistics::inDim() {
  return meanX.N;
}

double LinearStatistics::variance() {
  compute();
  return trace(VarX);
}

void LinearStatistics::correlationX(arr& corr, bool clearDiag) {
  compute();
  corr=VarX;
  arr sd(corr.d0);
  uint i, j;
  for(i=0; i<corr.d0; i++) sd(i)=::sqrt(corr(i, i));
  for(i=0; i<corr.d0; i++) for(j=0; j<corr.d1; j++)
          if(sd(i)*sd(j)) corr(i, j)/=sd(i)*sd(j); else corr(i, j)=0.;
  if(clearDiag) for(i=0; i<corr.d0; i++) corr(i, i)=0.;
}

void LinearStatistics::correlationXY(arr& corr, bool clearDiag) {
  compute();
  corr=CovXY;
  arr sd(corr.d0);
  uint i, j;
  for(i=0; i<corr.d0; i++) sd(i)=::sqrt(corr(i, i));
  for(i=0; i<corr.d0; i++) for(j=0; j<corr.d1; j++)
          if(sd(i)*sd(j)) corr(i, j)/=sd(i)*sd(j); else corr(i, j)=0.;
  if(clearDiag) for(i=0; i<corr.d0; i++) corr(i, i)=0.;
}

/// returns the mahalanobis metric relative to the VarX matrix
void LinearStatistics::mahalanobisMetric(arr& metric) {
  compute();
  metric.resizeAs(VarX);
  arr var;
  var=VarX;
  for(uint i=0; i<var.d0; i++) var(i, i)+=1./(1.+accum);
  ::inverse(metric, var);
}

void LinearStatistics::regressor(arr& A) {
  compute();
  A = CovXY * ::inverse(VarX);
}

void LinearStatistics::regressor(arr& A, arr& a) {
  compute();
  A = CovXY * ::inverse(VarX);
  a = MeanY - A*MeanX;
}

void LinearStatistics::predict(const arr& x, arr& y) {
  compute();
  arr A;
  A = ~CovXY * ::inverse(VarX);
  y = MeanY + A*(x-MeanX);
}

/// prototype for operator<<
void LinearStatistics::write(std::ostream& os) const {
  os
      <<"<LinearStatistics>"
      <<"\ndim X=" <<MeanX.N
      <<", dim Y=" <<MeanY.N
      <<", accum=" <<accum
      <<"\nmean X=" <<MeanX.ioraw()
      <<"\nmean Y=" <<MeanY
      <<"\nvariance X=" <<VarX
      <<"\ncovariance XY=" <<CovXY
      <<"\n</LinearStatistics>"
      <<std::endl;
}

//===========================================================================
//
// Tuple index
//

/// initializes to ``k out of n''
void TupleIndex::init(uint k, uint n) {
  CHECK_LE(k, n, "TupleIndex: can't choose " <<k <<" out of " <<n);
  uint i, j;
  tri.resize(n+1, k+1); tri=0;
  for(i=0; i<=n; i++) for(j=0; j<=i && j<=k; j++) {
      if(i==0 || j==0 || j==i) {
        tri(i, j)=1;
      } else {
        tri(i, j)=tri(i-1, j-1) + tri(i-1, j);
      }
    }

  uint N=tri(n, k);
  resize(N, k);
  for(j=0; j<k; j++) operator()(0, j)=j;
  for(i=1; i<N; i++) {
    for(j=0; j<k; j++) operator()(i, j)=operator()(i-1, j);
    for(j--; operator()(i, j)==n+j-k; j--) {};
    operator()(i, j)++;
    for(j++; j<k; j++) operator()(i, j)=operator()(i, j-1)+1;
  }
}

uint TupleIndex::index(uintA i) {
  uint a=0, n=tri.d0-1, k=i.N, u;
  if(k) a+=tri(n, k)-tri(n-i(0), k);
  for(u=1; u<i.N; u++) {
    CHECK(i(u)>i(u-1) && i(u) < n, "wrong symmetic index!");
    a+=tri(n-i(u-1)-1, k-u)-tri(n-i(u), k-u);
  }
  //std::cout <<a <<std::endl;
  return a;
}

void TupleIndex::checkValid() {
  uint i;
  for(i=0; i<d0; i++) CHECK_EQ(i, index(operator[](i)), "wrong index association");
}

//===========================================================================
//
// Kalman filter
//

arr _blockMatrix(const arr& A, const arr& B, const arr& C, const arr& D) {
  CHECK(A.nd==2 && B.nd==2 && C.nd==2 && D.nd==2, "");
  CHECK(A.d0==B.d0 && A.d1==C.d1 && B.d1==D.d1 && C.d0==D.d0, "");
  uint i, j, a=A.d0, b=A.d1;
  arr X(A.d0+C.d0, A.d1+B.d1);
  for(i=0; i<A.d0; i++) for(j=0; j<A.d1; j++) X(i, j)=A(i, j);
  for(i=0; i<B.d0; i++) for(j=0; j<B.d1; j++) X(i, j+b)=B(i, j);
  for(i=0; i<C.d0; i++) for(j=0; j<C.d1; j++) X(i+a, j)=C(i, j);
  for(i=0; i<D.d0; i++) for(j=0; j<D.d1; j++) X(i+a, j+b)=D(i, j);
  return X;
}
void Kalman::setTransitions(uint d, double varT, double varO) {
  a.resize(d); a.setZero();
  A.setDiag(1., d);
  Q.setDiag(varT, d);
  C.setDiag(1., d);
  R.setDiag(varO, d);
}

//notation follows Kevin Murphey's PhD thesis sec 3.6.1 (also in his DBN tutorial)
void Kalman::filter(arr& Y, arr& X, arr& V, arr* Rt) {
  CHECK_EQ(Y.nd, 2, "");
  uint d=Y.d1, n=Y.d0, t;
  CHECK(!Rt || (Rt->nd==3 && Rt->d0==n && Rt->d1==d && Rt->d2==d), "");

  arr e, S, K, L, Id;
  X.resize(n, d);
  V.resize(n, d, d);
  Id.setId(d);

  //initialization:
  X[0] = inverse(C) * Y[0];
  if(Rt) R=(*Rt)[0];
  V[0] = ~C * R * C + Q;

  for(t=1; t<n; t++) {
    X[t] = A * X[t-1]; //mean: forward predicted
    V[t] = A * V[t-1] * ~A + Q; //variance: forward predicted
    e    = Y[t] - C * X[t]; //observation error w.r.t. prediction
    if(Rt) R=(*Rt)[t];
    S    = C * V[t] * ~C + R; //variance of observation error
    K    = V[t] * ~C * inverse(S); //Kalman gain
    //L  = ::log \NN(e ; 0, S ) //observation log-likelihood
    X[t] = X[t] + K * e;
    V[t] = (Id - K*C) * V[t]; //= V[t] - K * S * ~K;
  }
}

//notation follows Kevin Murphey's PhD thesis sec 3.6.1 (also in his DBN tutorial)
void Kalman::smooth(arr& Y, arr& X, arr& V, arr* Vxx, arr* Rt) {
  CHECK_EQ(Y.nd, 2, "");
  uint d=Y.d1, n=Y.d0, t;
  CHECK(!Rt || (Rt->nd==3 && Rt->d0==n && Rt->d1==d && Rt->d2==d), "");

  arr e, S, K, L, Id;
  X.resize(n, d);
  V.resize(n, d, d);
  Id.setId(d);

  //initialization:
  X[0] = inverse(C) * Y[0];
  if(Rt) R=(*Rt)[0];
  V[0] = ~C * R * C + Q;

  for(t=1; t<n; t++) {
    X[t] = A * X[t-1] + a; //mean: forward predicted
    V[t] = A * V[t-1] * ~A + Q; //variance: forward predicted
    e    = Y[t] - C * X[t]; //observation error w.r.t. prediction
    if(Rt) R=(*Rt)[t];
    S    = C * V[t] * ~C + R; //variance of observation error
    K    = V[t] * ~C * inverse(S); //Kalman gain
    //L  = ::log \NN(e ; 0, S ) //observation log-likelihood
    X[t] = X[t] + K * e;
    V[t] = (Id - K*C) * V[t]; //= V[t] - K * S * ~K;
  }

  arr xp, Vp, J;
  if(Vxx) Vxx->resize(n-1, d, d);
  for(t=n-1; t--;) {
    xp   = A * X[t] + a; //mean: forward predicted
    Vp   = A * V[t] * ~A + Q; //variance: forward predicted
    J    = V[t] * ~A * inverse(Vp); //smoother gain matrix
    X[t] = X[t] + J * (X[t+1] - xp);
    V[t] = V[t] + J * (V[t+1] - Vp) * ~J;
    if(Vxx)(*Vxx)[t] = J * V[t+1]; //is cross covariance between x(t) and x(t+1)
  }
}

void Kalman::EMupdate(arr& Y, arr* Rt) {
  uint n=Y.d0, t; // d=Y.d1;
  arr X, V, Vxx;

  smooth(Y, X, V, &Vxx, Rt);

#if 0
  arr xx(d, d), vv(d, d), corr(d, d), sd(d); xx.setZero(); vv.setZero();
  for(t=0; t<n-1; t++) {
    xx += Vxx[t];// * inverse(V[t]);
    vv += V[t];
  }
  //xx/=n-1.;
  //vv/=n-1.;

  std::cout <<"EM-xx=" <<xx <<vv <<xx* inverse(vv);
#endif

  LinearStatistics S;
  for(t=0; t<n-1; t++) S.learn(X[t], X[t+1]);
  //S.regressor(A);
  S.regressor(A, a);

  //S.forget();
  //for(t=0;t<n-1;t++) S.learn(X[t], Y[t]);
  //S.regressor(C);

  std::cout <<"EM-update: A, a=" <<A <<a <<std::endl; //S.CovXY <<S.VarX <<std::endl;
}

void Kalman::fb(arr& y, arr& f, arr& F, arr& g, arr& G, arr& p, arr& P, arr* Rt) {
  CHECK_EQ(y.nd, 2, "");
  uint d=y.d1, n=y.d0, t, T=y.d0;
  CHECK(!Rt || (Rt->nd==3 && Rt->d0==n && Rt->d1==d && Rt->d2==d), "");

  f.resize(n, d);
  f.resize(n, d, d);

  // Some constant matrices :
  arr& CovH=Q, &CovV=R, &B=C; //rename my matricies
  arr Hinv, HinvA, AHinvA, BVinv, BVinvB, Kinv, Linv, Lconst, J;
  Hinv=inverse(CovH); HinvA=Hinv*A; AHinvA=~A*HinvA;
  BVinv=~B*inverse(CovV); BVinvB=BVinv*B;
  if(Rt) { BVinv=~B*inverse((*Rt)[0]); BVinvB=BVinv*B; }

  // Forward Pass
  F.resize(n, d, d);
  f.resize(n, d);
  F[0] = BVinvB;     //inv(CovP) + BVinvB;
  f[0] = BVinv*y[0]; //inv(CovP)*meanP + BVinv*y[0];
  for(t=1; t<T; t++) {
    if(Rt) { BVinv=~B*inverse((*Rt)[t]); BVinvB=BVinv*B; }
    Kinv = inverse(AHinvA+F[t-1]);
    F[t] = Hinv - HinvA*Kinv*~HinvA + BVinvB;
    f[t] = HinvA*Kinv*f[t-1] + BVinv*y[t];
  }

  // Backward Pass
  G.resize(n, d, d);
  g.resize(n, d);
  Lconst = Hinv+BVinvB;
  G[T-1]=0.;
  g[T-1]=0.;
  for(t=T-1; t--;) {
    if(Rt) { BVinv=~B*inverse((*Rt)[t+1]); BVinvB=BVinv*B; Lconst = Hinv+BVinvB; }
    Linv = inverse(Lconst + G[t+1]);
    G[t] = AHinvA - ~HinvA*Linv*HinvA;
    g[t] = ~HinvA*Linv*(BVinv*y[t+1]+g[t+1]);
  }

  // single marginals:
  P.resize(n, d, d);
  p.resize(n, d);
  for(t=0; t<T; t++) {
    P[t] = inverse(F[t]+G[t]);
    p[t] = P[t] * (f[t]+g[t]);
  }

  // pair marginals: the means are as above
  arr PP(n-1, 2*d, 2*d), hh(n, d, d), h1h(n-1, d, d), hh1(n-1, d, d);
  for(t=0; t<T-1; t++) {
    if(Rt) { BVinv=~B*inverse((*Rt)[t+1]); BVinvB=BVinv*B; }
    J = _blockMatrix(F[t]+AHinvA, -~HinvA,
                     -HinvA, G[t+1]+Hinv+BVinvB);
    //jtmp(1:H, 1) = f{t}+BVinv*v{t};
    //jtmp(H+1:2*H, 1) = g{t};
    PP[t] = inverse(J);
    h1h[t]= PP[t].sub(d, 2*d-1, 0, d-1) + p[t+1]*~p[t]; //lower-left submatrix
    hh1[t]= PP[t].sub(0, d-1, d, 2*d-1) + p[t]*~p[t+1]; //upper-right submatrix
  }
  for(t=0; t<T; t++) {
    hh[t] = P[t] + p[t]*~p[t];
  }

  // transform fwd & bwd passes to moment representation
  for(t=0; t<T; t++) {
    F[t] = inverse(F[t]);  f[t] = F[t]*f[t];
    G[t] = inverse(G[t]);  g[t] = G[t]*g[t];
  }

  // EM-update:
  arr up(d, d), dn(d, d);
  std::cout <<"EM-update: before:\nA=" <<A <<" B=" <<B <<" covH=" <<CovH <<" covV=" <<CovV <<std::endl;

  for(t=0, up=0., dn=0.; t<T-1; t++) { up += h1h[t]; dn += hh[t]; }
  A = up * inverse(dn);

  for(t=0, up=0., dn=0.; t<T; t++) { up += y[t]*~p[t]; dn += hh[t]; }
  //B = up * inv(dn);

  for(t=0, up=0., dn=0.; t<T; t++) { up += y[t]*~y[t] - B*hh[t]*~B; }
  //CovV = up/(double)T;

  for(t=0, up=0., dn=0.; t<T-1; t++) { up += hh[t+1] - (double)2.*A*hh1[t] + A*hh[t]*~A; }
  CovH = up/(T-(double)1.);

  std::cout <<"after:\nA=" <<A <<" B=" <<B <<" covH=" <<CovH <<" covV=" <<CovV <<std::endl;

  // log-likelihood:
  double LL=0., l;
  arr z, Z;
  for(t=0; t<T; t++) {
    z = A*p[t];           //mean output
    Z = CovV + A*CovH*~A; //output variance
    z = y[t]-z;
    l=-.5*scalarProduct(Z, z, z) - .5*d*::log(RAI_2PI) - .5*::log(determinant(Z));
    LL += l;
  }
  LL/=T;
  std::cout <<"log-likelihood=" <<LL <<std::endl;
}

//===========================================================================
//
// X-Splines
//

double _intpow(const double x, const int n) {
  double val = x;
  for(int i=1; i<n; i++)
    val *= x;
  return val;
}

// g(u) as defined in the above mentioned article
double _g(double u, double q, double p) {
  return
    q *                            u
    + 2 * q *                 _intpow(u, 2)
    + (10 - 12 * q - p) *     _intpow(u, 3)
    + (2 * p + 14 * q - 15) * _intpow(u, 4)
    + (6 - 5*q - p) *         _intpow(u, 5);
}
double _dgdu(double u, double q, double p) {
  return
    q *                                  1.
    + 2 * q *                        u    *2.
    + (10 - 12 * q - p) *     _intpow(u, 2) *3.
    + (2 * p + 14 * q - 15) * _intpow(u, 3) *4.
    + (6 - 5*q - p) *         _intpow(u, 4) *5.;
}
// h(u)
double _h(double u, double q) {
  return
    q *            u
    + 2 * q * _intpow(u, 2)
    - 2 * q * _intpow(u, 4)
    - q *     _intpow(u, 5);
}

double _dhdu(double u, double q) {
  return
    q *                  1.
    + 2 * q *        u    *2.
    - 2 * q * _intpow(u, 3) *4.
    - q *     _intpow(u, 4) *5.;
}

XSpline::XSpline() { DELTA = 1.; };
XSpline::~XSpline() {};
void XSpline::setWeights(double w) { W.resize(V.d0); W=w; W(0)=W(W.N-1)=0.; }
/// suggested: (true, 1.) or (false, [.5, 1.])
void XSpline::type(bool hit, double smooth) { if(hit) setWeights(-smooth); else setWeights(smooth); }
void XSpline::referTo(arr& P) { V.referTo(P); }
arr XSpline::eval(double t) { arr x; eval(t, x); return x; }
void XSpline::eval(double t, arr& x, arr& v) { eval(t, x, &v); }

//---------------------------------------------------------------------------
// code taken from jab@gk.dtu.dk:
//
// xspline.cc
//
// this file contains a class XSpline that implements the general XSplines
// presented by Blanc and Schlick in the article
//
// ``A Spline Model Designed for the End-User.''
//
// Comments and bug-reports to jab@gk.dtu.dk
//---------------------------------------------------------------------------
void XSpline::eval(double t, arr& x, arr* v) {
  CHECK(t>=0 && t<=V.d0-1, "out of range");

  //standard interpolation type (weighting of vertices)
  if(!W.N) type(true, 1.);

  // query t is between two vertices V_{k+1} and V_{k+2}
  // the four vertices V_{k, .., k+3} will be used to calculate the output
  double kf = floor(t) - 1.0;
  int k = int(kf);

  // the weights associated to the four vertices are
  double W1, W2;
  W1=W(k+1);
  W2=k+2<(int)W.N?W(k+2):0.;

  // Find the big T values
  //
  // T0p is the point where fk decreases to 0
  // T1p is the point where fk+1 decreases to 0
  // T2m is the point where fk+2 increases from 0
  // T3m is the point where fk+3 increases from 0
  //
  // The parameter sk of a control point k affects
  // the functions that have max vals in k-1 and k+1
  // Therefore, sv[k+1] affects T0p (The point where
  // fk decreases to 0) and T2m. Similarly, sv[k+2]
  // affects T1p and T3m
  double T0p = kf+1 + (W1>0?W1:0) * DELTA;
  double T1p = kf+2 + (W2>0?W2:0) * DELTA;
  double T2m = kf+1 - (W1>0?W1:0) * DELTA;
  double T3m = kf+2 - (W2>0?W2:0) * DELTA;

  // Find the pk values
  // The pm values are derived directly from the Ts above
  double tmp_factor = 2/(DELTA*DELTA);
  double pm1 = _intpow(kf   - T0p, 2) * tmp_factor;
  double pm0 = _intpow(kf+1 - T1p, 2) * tmp_factor;
  double pp1 = _intpow(kf+2 - T2m, 2) * tmp_factor;
  double pp2 = _intpow(kf+3 - T3m, 2) * tmp_factor;

  // The sv indices are determined like for the Ts
  double qp0 = W1<0?-W1/2.0:0;
  double qp1 = W2<0?-W2/2.0:0;
  double qp2 = W1<0?-W1/2.0:0;
  double qp3 = W2<0?-W2/2.0:0;

  // The code below resembles the part of (17) in the article,
  // where A0 ... A3 are calculated, but there are differences
  //
  // 1.
  //
  // We use g instead of f. This is ok, because
  //    q = 0 => g(u) = f(u)
  //
  // 2.
  // we don't check whether we are inside the legal range
  // of fk+1 and fk+2. This is not necessary, since we are in the
  // range [ tk+1, tk+2 ] and both gk+1 and gk+2 must be defined
  // in this range.
  //
  // 3.
  //
  // if we are in the ranges where fk and fk+3 would have been
  // nil in the case of basic splines, we use h instead of g
  // except if q
  //
  double A0, A1, A2, A3;
  if(t<=T0p) A0 = _g((t-T0p)/(kf-T0p), qp0, pm1);
  else       A0 = qp0>0 ? _h((t-T0p)/(kf-T0p), qp0) : 0;
  A1 = _g((t-T1p)/(kf+1-T1p), qp1, pm0);
  A2 = _g((t-T2m)/(kf+2-T2m), qp2, pp1);
  if(t>=T3m) A3 = _g((t-T3m)/(kf+3-T3m), qp3, pp2);
  else       A3 = qp3>0 ? _h((t-T3m)/(kf+3-T3m), qp3) : 0;

  CHECK(k>=0 || A0==0., "non-zero weight for out-of-range!");
  CHECK(k+3<(int)V.d0 || A3==0., "non-zero weight for out-of-range!");

  double SUM = A0+A1+A2+A3;
  A0/=SUM;
  A1/=SUM;
  A2/=SUM;
  A3/=SUM;

  x.resize(V.d1); x.setZero();
  if(A0) x+=A0*V[k];
  if(A1) x+=A1*V[k+1];
  if(A2) x+=A2*V[k+2];
  if(A3) x+=A3*V[k+3];

  if(v) {
    double dA0, dA1, dA2, dA3;
    if(t<=T0p) dA0 = _dgdu((t-T0p)/(kf-T0p), qp0, pm1)/(kf-T0p);
    else       dA0 = qp0>0 ? _dhdu((t-T0p)/(kf-T0p), qp0)/(kf-T0p) : 0;
    dA1 = _dgdu((t-T1p)/(kf+1-T1p), qp1, pm0)/(kf+1-T1p);
    dA2 = _dgdu((t-T2m)/(kf+2-T2m), qp2, pp1)/(kf+2-T2m);
    if(t>=T3m) dA3 = _dgdu((t-T3m)/(kf+3-T3m), qp3, pp2)/(kf+3-T3m);
    else       dA3 = qp3>0 ? _dhdu((t-T3m)/(kf+3-T3m), qp3)/(kf+3-T3m) : 0;

    CHECK(k>=0 || dA0==0., "non-zero weight for out-of-range!");
    CHECK(k+3<(int)V.d0 || dA3==0., "non-zero weight for out-of-range!");

    double dSUM = dA0+dA1+dA2+dA3;
    dA0 = dA0/SUM - A0/SUM*dSUM;
    dA1 = dA1/SUM - A1/SUM*dSUM;
    dA2 = dA2/SUM - A2/SUM*dSUM;
    dA3 = dA3/SUM - A3/SUM*dSUM;

    v->resize(V.d1); v->setZero();
    if(dA0) *v+=dA0*V[k];
    if(dA1) *v+=dA1*V[k+1];
    if(dA2) *v+=dA2*V[k+2];
    if(dA3) *v+=dA3*V[k+3];
  }
}

//===========================================================================
//
// Partial Least Squares (PLS, SIMPLS, de Jong)
//

/// add a new datum to the statistics -- calls LinearStatistics::learn (doesn't execute PLS yet!)
void PartialLeastSquares::learn(const arr& x, const arr& y, double weight) {
  S.learn(x, y, weight);
}

/// same for 1D output
void PartialLeastSquares::learn(const arr& x, double y, double weight) {
  S.learn(x, y, weight);
}

void PartialLeastSquares::clear() {
  S.clear();
}

/// executes PLS, computing the linear mapping from the projections
void PartialLeastSquares::SIMPLS() {
  uint I=S.meanX.N, O=S.meanY.N, K=I, i;
  if(maxProj) K=maxProj;
  arr A, M, C;

  S.compute();
  double scale=trace(S.VarX);
  if(scale<1e-10) {
    //RAI_MSG("Warning: PLS: too small variance (only 1 data point?) -> use constant regression");
    B.resize(O, I); W.resize(K, I); Q.resize(K, O); B=0.; W=0.; Q=0.;
    return;
  }
  M=S.VarX/scale;
  A=S.CovXY/scale;
  C.resize(I, I); C=0.; for(i=0; i<I; i++) C(i, i)=1.;

  arr AA, eigU, eigw, eigV;
  double c;
  arr w, q, p, P, v;
  W.resize(K, I); P.resize(K, I); Q.resize(K, O); W=0.; Q=0.;
  resErr.resize(K); resErr=0.;
  for(uint k=0; k<K; k++) {
    //step 0: check non-zero residuals:
    //std::cout <<sumOfSqr(A) <<std::endl;
    if(sumOfSqr(A)<1e-10) break;
    //step 1
    AA=~A*A;
    svd(AA, eigU, eigw, eigV, true);
    q=eigV[0];
    //resErr(k)=eigw(0);
    //step 2
    w=A*q;
    c=scalarProduct(w, M*w);
    if(fabs(c)<1e-10) break;
    if(!(c>0.)) HALT("PLS: variance matrix M is not positive definite!");
    w/=(double)sqrt(c);
    W[k]=w;
    //step 3
    p=M*w;
    P[k]=p;
    //step 4
    q=~A*w;
    Q[k]=q;
    //step 5
    v=C*p; v/=length(v);
    //step 6
    C-=v^v;
    M-=p^p;
    //step 7
    A=C*A;
  }
  //std::cout <<sumOfSqr(A) <<std::endl;
  B=~Q*W;
}

/// the learned mapping
void PartialLeastSquares::map(const arr& x, arr& y) {
  if(!S.computed) SIMPLS();
  if(x.nd==2) {
    y.resize(x.d0, outDim());
    for(uint i=0; i<x.d0; i++) map(x[i], y[i]());
    return;
  }
  arr xx;
  xx=x-S.MeanX;
  y=B*xx;
  y+=S.MeanY;
  //y=B*(x-MeanX)+MeanY;
}

/// in case y is a scalar
void PartialLeastSquares::map(const arr& x, double& y) { arr _y(&y, 1); map(x, _y); }

/// return the output scalar
double PartialLeastSquares::map(const arr& x) { double o; map(x, o); return o; }

/// return the k-th projection learned by PLS
arr PartialLeastSquares::projection(uint k) {
  CHECK(k<W.d0, "PLS: don't have " <<k <<"th projection");
  return W[k];
}

uint PartialLeastSquares::inDim() { return S.meanX.N; }
uint PartialLeastSquares::outDim() { return S.meanY.N; }

void PartialLeastSquares::write(std::ostream& os) const { os <<S; }

//===========================================================================
//
// the minimize routine
//

typedef long int longinteger;

//----- a workspace with static functions suited to pass to the subroutines
namespace minimizeStatic {
arr* startx;     //to look up the dimensionality
arr xref;
uint n;
uint fc=0, dfc=0; //evaluation counters
double y;

double(*f)(arr* grad, const arr& x, void* data);

//conjugate gradient minimizer and wrappers
double CG_f(double x[], void* data) {
  //printf("[CGf]");
  fc++;
  xref.referTo(x, n); xref.reshapeAs(*startx);
  y=f(nullptr, xref, data);
  //printf("minimization (#f=%3i #df=%3i): current f-value = %g  \n", fc, dfc, y);
  return y;
}
void CG_df(double x[], double dx[], void* data) {
  //printf("[CGd]");
  dfc++;
  xref.referTo(x, n); xref.reshapeAs(*startx);
  f(&arr(dx, n)(), xref, data);
}

//LM optimizer and wrappers
void LM_f(double* p, double* hx, longinteger m, longinteger n, void* adata) {
  //printf("%li %li", n, m);
  fc++;
  xref.referTo(p, n); xref.reshapeAs(*startx);
  y = f(nullptr, xref, adata);
  //printf("minimization (#f=%3i #df=%3i): current f-value = %g  \n", fc, dfc, y);
  int i;
  for(i=0; i<n; i++) hx[i]=y;
}
void LM_df(double* p, double* J, longinteger m, longinteger n, void* adata) {
  //printf("%li %li", n, m);
  dfc++;
  xref.referTo(p, n); xref.reshapeAs(*startx);
  f(&arr(J, n)(), xref, adata);
  int i, j;
  for(i=1; i<n; i++) for(j=0; j<n; j++) J[i*n+j]=J[j];
}

//Rprop and wrappers
double RP_f(const arr& x, void* data) {
  //printf("[RPf]");
  fc++;
  y=f(nullptr, x, data);
  //printf("minimization (#f=%3i #df=%3i): current f-value = %g  \n", fc, dfc, y);
  return y;
}
void RP_df(arr& dx, const arr& x, void* data) {
  //printf("[RPd]");
  dfc++;
  f(&dx, x, data);
}
}

//----- fwd declarations of external routines
#ifdef RAI_algos_extern
extern "C" {
  longinteger dlevmar_der(
    void (*func)(double* p, double* hx, longinteger m, longinteger n, void* adata),
    void (*jacf)(double* p, double* j, longinteger m, longinteger n, void* adata),
    double* p, double* x, longinteger m, longinteger n, longinteger itmax, double* opts,
    double* info, double* work, double* covar, void* adata);
}

void frprmn(double p[], int n, double ftol, int* iter, int maxIterations, double* fret,
            double(*func)(double [], void*), void (*dfunc)(double [], double [], void*),
            void* data);
#endif

/*int rpropMinimize(double (*f)(const arr&, void*),
                  void (*df)(arr&, const arr&, void*),
                  void *data,
                  arr& _x,
                  double *fmin_return,
                  uint maxIterations,
                  double stoppingTolerance);*/

//--- the minimize routine itself
int rai::minimize(double(*f)(arr*, const arr&, void*),
                  void* data,
                  arr& x,
                  double* fmin_return,
                  int method,
                  uint maxIterations,
                  double stoppingTolerance,
                  bool testGrad) {

  minimizeStatic::f=f;
  minimizeStatic::n=x.N;
  minimizeStatic::fc=0;
  minimizeStatic::dfc=0;
  minimizeStatic::startx=&x;

  int i;
  double fmin, *fminp;
  if(!fmin_return) fminp=&fmin; else fminp=fmin_return;

  arr LM_target(x.N); LM_target.setZero();

  if(testGrad) checkGradient(f, data, x, stoppingTolerance);

  switch(method) {
    case 2: //Rprop
      NIY;
      //i=rpropMinimize(minimizeStatic::RP_f, minimizeStatic::RP_df, data,
      //  x, fminp, maxIterations, stoppingTolerance);
      break;
#ifdef RAI_algos_extern
    case 0: //conjugate gradient
      ::frprmn(x.p, x.N, stoppingTolerance, &i, maxIterations, fminp,
               minimizeStatic::CG_f,
               minimizeStatic::CG_df,
               data);
      /*void frprmn(double p[], int n, double ftol, int *iter, double *fret,
      double (*func)(double [], void*), void (*dfunc)(double [], double [], void*),
      void *data);*/
      break;
    case 1: //Levenberg-Marquardt
      NIY;
      /*i=dlevmar_der(minimizeStatic::LM_f,
        minimizeStatic::LM_df,
        x.p, LM_target.p, x.N, x.N, 1000,
        nullptr, nullptr, nullptr, nullptr, data);*/
      break;
    case 3: //Rprop + conj Grad
      int j;
      NIY;
      //i=rpropMinimize(minimizeStatic::RP_f, minimizeStatic::RP_df, data, x, fminp, maxIterations, 10*stoppingTolerance);
      frprmn(x.p, x.N, stoppingTolerance, &j, maxIterations, fminp, minimizeStatic::CG_f, minimizeStatic::CG_df, data);
      i+=j;
      break;
#endif
    default:  HALT("don't know method " <<method);
  }
  printf("--- minimization using method %i:\
         \n  #iterations=%i\
         \n  f-counts=%i\
         \n  df-counts=%i\
         \n  f(x_min)=%g\n", method, i, minimizeStatic::fc, minimizeStatic::dfc, *fminp);

  if(testGrad) checkGradient(f, data, x, stoppingTolerance);

  return i;
}

//================================================================================
//
// helper functions for imporing foreign code
//

