/*  ------------------------------------------------------------------
    Copyright (c) 2011-2020 Marc Toussaint
    email: toussaint@tu-berlin.de

    This code is distributed under the MIT License.
    Please see <root-path>/LICENSE for details.
    --------------------------------------------------------------  */

#include "MLcourse.h"
#include "../Core/util.h"

void CrossValidation::crossValidateSingleLambda(const arr& X, const arr& y, double lambda, uint k_fold, bool permute, arr* beta_k_fold, arr* beta_total, double* scoreMean, double* scoreSDV, double* scoreTrain) {
  arr Xtrain, Xtest, ytrain, ytest;
  uint n=X.d0;

  //permute data?
  arr X_perm, y_perm;
  if(permute) {
    uintA perm;
    perm.setRandomPerm(X.d0);
    X_perm=X;  X_perm.permuteRows(perm);
    y_perm=y;  if(y.nd==2) y_perm.permuteRows(perm); else y_perm.permute(perm);
  }
  //initialize
  double cost, costM=0., costD=0.;
  arr beta;
  if(beta_k_fold) beta_k_fold->clear();

  //determine blocks
  CHECK_GE(n, k_fold, "we need at least as much data as k for k-fold CV");
  uintA blockStart(k_fold+1);
  for(uint k=0; k<=k_fold; k++) blockStart(k) = (k*n)/k_fold;

  //go
  for(uint k=0; k<k_fold; k++) {
    if(!permute) {
      Xtrain = X;  ytrain = y;
    } else {
      Xtrain = X_perm;  ytrain = y_perm;
    }
    Xtrain.delRows(blockStart(k), blockStart(k+1)-blockStart(k));
    if(ytrain.nd==2)
      ytrain.delRows(blockStart(k), blockStart(k+1)-blockStart(k));
    else
      ytrain.remove(blockStart(k), blockStart(k+1)-blockStart(k));
    Xtest.referToRange(X, blockStart(k), blockStart(k+1)-1);
    ytest.referToRange(y, blockStart(k), blockStart(k+1)-1);

    if(verbose) cout <<k <<": train:";
    train(Xtrain, ytrain, lambda, beta);
    if(beta_k_fold) beta_k_fold->append(beta);

    cost = test(Xtest, ytest, beta);
    costM += cost;
    costD += cost*cost;
    if(verbose) cout <<" test: " <<cost <<endl;
  }
  if(beta_k_fold) beta_k_fold->reshape(k_fold, beta.N);

  costM /= k_fold;
  costD /= k_fold;
  costD -= costM*costM;
  costD = sqrt(costD)/sqrt((double)k_fold); //sdv of the mean estimator

  //on full training data:
  if(verbose) cout <<"full: train:";
  train(X, y, lambda, beta);
  double costT = test(X, y, beta);
  if(beta_total) *beta_total = beta;
  if(verbose) cout <<" test: " <<costT <<endl;

  if(scoreMean)  *scoreMean =costM; else scoreMeans =ARR(costM);
  if(scoreSDV)   *scoreSDV  =costD; else scoreSDVs  =ARR(costD);
  if(scoreTrain) *scoreTrain=costT; else scoreTrains=ARR(costT);
  if(verbose) cout <<"CV: lambda=" <<lambda <<" \tmean-on-rest=" <<costM <<" \tsdv=" <<costD <<" \ttrain-on-full=" <<costT <<endl;
  if(verbose) cout <<"cross validation results:";
  if(verbose) if(lambda!=-1) cout <<"\n  lambda = " <<lambda;
  if(verbose) cout <<"\n  test-error  = " <<costM <<" (+- " <<costD <<", lower: " <<costM-costD <<")"<<"\n  train-error = " <<costT <<endl;
}

void CrossValidation::crossValidateMultipleLambdas(const arr& X, const arr& y, const arr& _lambdas, uint k_fold, bool permute) {
  lambdas=_lambdas;
  scoreMeans.resizeAs(lambdas);
  scoreSDVs.resizeAs(lambdas);
  scoreTrains.resizeAs(lambdas);
  for(uint i=0; i<lambdas.N; i++) {
    crossValidateSingleLambda(X, y, lambdas(i), k_fold, permute, nullptr, nullptr, &scoreMeans(i), &scoreSDVs(i), &scoreTrains(i));
  }
}

void CrossValidation::plot() {
  FILE("z.cv") <<catCol(lambdas, scoreMeans, scoreSDVs, scoreTrains);
  gnuplot("set log x; set xlabel 'lambda'; set ylabel 'mean squared error'; plot 'z.cv' us 1:2:3 w errorlines title 'cv error','z.cv' us 1:4 w l title 'training error'", "z.pdf", true);

}

void linearFeatures(arr& Z, const arr& X) {
  Z = catCol(ones(X.d0, 1), X);
}

void quadraticFeatures(arr& Z, const arr& X) {
  uint n=X.d0, d=X.d1;
  Z.resize(n, 1 + d + d*(d+1)/2);
  uint i, j, k, l;
  for(i=0; i<n; i++) {
    arr x=X[i];
    arr z=Z[i];
    l=0;
    z(l++)=1.;
    for(j=0; j<d; j++) z(l++) = x(j);
    for(j=0; j<d; j++) for(k=0; k<=j; k++) z(l++) = x(j)*x(k);
  }
}

void cubicFeatures(arr& Z, const arr& X) {
  uint n=X.d0, d=X.d1;
  Z.resize(n, 1 + d + d*(d+1)/2 + d*(d+1)*(d+2)/6);
  uint i, j, k, l, m;
  for(i=0; i<n; i++) {
    arr x=X[i];
    arr z=Z[i];
    l=0;
    z(l++)=1.;
    for(j=0; j<d; j++) z(l++) = x(j);
    for(j=0; j<d; j++) for(k=0; k<=j; k++) z(l++) = x(j)*x(k);
    for(j=0; j<d; j++) for(k=0; k<=j; k++) for(m=0; m<=k; m++) z(l++) = x(j)*x(k)*x(m);
  }
}

void piecewiseConstantFeatures(arr& Z, const arr& X) {
  uint n=X.d0, d=X.d1;
  if(d!=1) HALT("only for 1D data");
  Z.resize(n, 6);
  Z.setZero();
  for(uint i=0; i<n; i++) {
    double x=X(i, 0);
    arr z=Z[i];
    if(x<-2.5) x=-2.5;
    if(x> 2.5) x= 2.5;
    z(floor(x+3.))=1.;
  }
}

void piecewiseLinearFeatures(arr& Z, const arr& X) {
  uint n=X.d0, d=X.d1;
  if(d!=1) HALT("only for 1D data");
  Z.resize(n, 7);
  Z.setZero();
  for(uint i=0; i<n; i++) {
    double x=X(i, 0);
    arr z=Z[i];
    z(0) = 1.; //constant
    z(1) = x; //linear
    for(int j=-2; j<=2; j++) z(j+4) = x<j?0:x-j;
  }
}

void rbfFeatures(arr& Z, const arr& X, const arr& Xtrain, arr& Jacobian) {
  uint rbfBias = rai::getParameter<uint>("rbfBias", 1);
  double rbfWidth = rai::sqr(rai::getParameter<double>("rbfWidth", .2));
  Z.resize(X.d0, Xtrain.d0+rbfBias);
  if(!!Jacobian) Jacobian.resize(X.d0, Xtrain.d0+rbfBias, X.d1);
  for(uint i=0; i<Z.d0; i++) {
    if(rbfBias) Z(i, 0) = 1.; //bias feature also for rbfs?
    for(uint j=0; j<Xtrain.d0; j++) {
      Z(i, j+rbfBias) = ::exp(-sqrDistance(X[i], Xtrain[j])/rbfWidth);
      if(!!Jacobian) {
        Jacobian(i, j+rbfBias, {}) = (-2.*Z(i, j+rbfBias)/rbfWidth) * (X[i] - Xtrain[j]);
      }
    }
  }
}

arr makeFeatures(const arr& X, FeatureType featureType, const arr& rbfCenters, arr& Jacobian) {
  if(X.nd==1) return makeFeatures(~X, featureType, rbfCenters, Jacobian);
  if(featureType==readFromCfgFileFT) featureType = (FeatureType)rai::getParameter<uint>("modelFeatureType", 1);
  arr Z;
  switch(featureType) {
    case constFT:     Z = consts<double>(1., X.d0, 1);  break;
    case linearFT:    linearFeatures(Z, X);  break;
    case quadraticFT: quadraticFeatures(Z, X);  break;
    case cubicFT:     cubicFeatures(Z, X);  break;
    case rbfFT:       if(!!rbfCenters) rbfFeatures(Z, X, rbfCenters, Jacobian); else rbfFeatures(Z, X, X, Jacobian);  break;
    case piecewiseConstantFT:  piecewiseConstantFeatures(Z, X);  break;
    case piecewiseLinearFT:    piecewiseLinearFeatures(Z, X);  break;
    default: HALT("");
  }
  return Z;
}

arr artificialData(arr& X, arr& y, ArtificialDataType dataType) {
  uint n = rai::getParameter<uint>("n", 100);
  uint d = rai::getParameter<uint>("d", 1);
  double sigma = rai::getParameter<double>("sigma", 1.); // observation noise

  arr beta_true;

  if(dataType==readFromCfgFileDT) dataType = (ArtificialDataType)rai::getParameter<uint>("dataType", 1);
  switch(dataType) {
    case linearRedundantData:
    case linearData: {
      X = randn(n, d);
      arr Z = makeFeatures(X, (FeatureType)rai::getParameter<uint>("dataFeatureType", 1));
      arr beta;
      beta = randn(Z.d1, 1).reshape(Z.d1);
      if(dataType==linearRedundantData) {
        double pr = rai::getParameter<double>("d_p_redundant", .5);
        for(uint j=1; j<beta.N; j++) if(rnd.uni()<pr) beta(j)=0.;
      }
      y = Z*beta;
      y = y + sigma*randn(size(y));
      beta_true = beta;
      break;
    }
    case sinusData: {
      X.setGrid(1, -3, 3, n-1);
      y.resize(X.d0);
      for(uint i=0; i<X.d0; i++) y(i) = sin(X(i, 0));
      y += sigma*randn(size(y));
      break;
    }
    case linearOutlier: {
      double rate = rai::getParameter<double>("outlierRate", .1);
      X = randn(n, d);
      arr Z = makeFeatures(X, (FeatureType)rai::getParameter<uint>("dataFeatureType", 1));
      arr beta;
      beta = randn(Z.d1, 1).reshape(Z.d1);
      y = Z*beta;
      for(uint i=0; i<y.N; i++)  if(rnd.uni()<rate) {
          y(i) += rai::getParameter<double>("outlierSigma", 10.)*rnd.gauss();
        } else {
          y(i) += sigma*rnd.gauss();
        }
      beta_true = beta;
      break;
    }
    default: HALT("");
  }
  cout <<"correct beta=" <<beta_true <<endl;
  return beta_true;
}

void artificialData_Hasties2Class(arr& X, arr& y) {
  uint n = rai::getParameter<uint>("n", 100);
  uint d = rai::getParameter<uint>("d", 2);

  arr means0(10, d), means1(10, d), x(d), bias0(d), bias1(d);

  bias0.setZero(); bias0(0) = 1.;
  bias1.setZero(); if(d>1) bias1(1) = 1.;

  rndGauss(means0);  means0 += ones(10, 1)*~bias0;
  rndGauss(means1);  means1 += ones(10, 1)*~bias1;

  X.clear();
  y.clear();
  for(uint i=0; i<n; i++) {
    rndGauss(x, .2);  x += means0[rnd(10)];
    X.append(~x);
    y.append(0);

    rndGauss(x, .2);  x += means1[rnd(10)];
    X.append(~x);
    y.append(1);
  }
}

void artificialData_HastiesMultiClass(arr& X, arr& y) {
  uint n = rai::getParameter<uint>("n", 100);
  uint d = rai::getParameter<uint>("d", 2);
  uint M = rai::getParameter<uint>("M", 3);

  arr means(M, 10, d), x(d);

  rndGauss(means);
  for(uint c=0; c<M; c++)  means[c]() += ones(10, 1)*~consts((double)c, d);

  X.resize(M*n, d);
  y.resize(M*n, M);
  y.setZero();
  for(uint i=0; i<n; i++) {
    for(uint c=0; c<M; c++) {
      arr x=X[i*M+c];  rndGauss(x, .2);  x += means[c][rnd(10)];
      y(i*M+c, c)=1.;
    }
  }
}

void artificialData_GaussianMixture(arr& X, arr& y) {
  uint n = rai::getParameter<uint>("n", 100);
  uint M = rai::getParameter<uint>("M", 3);
  double sig = rai::getParameter<double>("sigma", .2);

  arr means(M, 2), V(M, 2, 2), x(2);

  rndGauss(means);
  rndGauss(V);
  //means.setZero();
  //for(uint c=0;c<M;c++)  means[c]() += ARR(c, c);

  X.resize(M*n, 2);
  y.resize(M*n, M);
  y.setZero();
  for(uint i=0; i<n; i++) {
    for(uint c=0; c<M; c++) {
      arr x=X[i*M+c];  rndGauss(x, sig);  x = V[c]*x;  x += means[c];
      y(i*M+c, c)=1.;
    }
  }
}

void load_data(arr& X, const char* filename, bool whiten) {
  ifstream is;
  rai::open(is, filename);
  rai::Array<rai::String> strs;
  if(!rai::contains("0123456789.-+", rai::peerNextChar(is))) {
    //read line of strings
    rai::String str;
    for(;;) {
      str.read(is, " \"\t\r", " \"\t\r\n", false);
      if(!str.N) break;
      strs.append(str);
    }
    cout <<"header: " <<strs <<endl;
  }
  X.clear();
  X.read(is);
  cout <<"data stats:"
       <<"\n  data entries    n=" <<X.d0
       <<"\n  entry dimension d=" <<X.d1
       <<"\n  stats: [# 'name' mean sdv]" <<endl;
  arr mean = sum(X, 0);  mean /= (double)X.d0;
  arr var = ~X*X;       var /= (double)X.d0;
  var -= mean^mean;
  for(uint j=0; j<X.d1; j++) {
    cout <<j <<' ';
    if(strs.N) cout <<strs(j) <<' ';
    cout <<mean(j) <<' ' <<sqrt(var(j, j)) <<endl;
  }

  //-- whiten the data
  if(whiten) {
    for(uint i=0; i<X.d0; i++) for(uint j=0; j<X.d1; j++) {
        X(i, j) /= sqrt(var(j, j));
      }
  }
}

