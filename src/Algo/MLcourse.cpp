/*  ------------------------------------------------------------------
    Copyright (c) 2011-2024 Marc Toussaint
    email: toussaint@tu-berlin.de

    This code is distributed under the MIT License.
    Please see <root-path>/LICENSE for details.
    --------------------------------------------------------------  */

#include "MLcourse.h"
#include "../Core/util.h"

#include <math.h>

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

  if(scoreMean)  *scoreMean =costM; else scoreMeans =arr{costM};
  if(scoreSDV)   *scoreSDV  =costD; else scoreSDVs  =arr{costD};
  if(scoreTrain) *scoreTrain=costT; else scoreTrains=arr{costT};
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
  gnuplot("set log x; set xlabel 'lambda'; set ylabel 'mean squared error'; plot 'z.cv' us 1:2:3 w errorlines title 'cv error','z.cv' us 1:4 w l title 'training error'", true, false, "z.pdf");

}

arr artificialData(arr& X, arr& y, str dataType) {
  uint n = rai::getParameter<double>("n", 100);
  uint d = rai::getParameter<double>("d", 1);
  double sigma = rai::getParameter<double>("ML/dataSigma", 1.); // observation noise

  arr beta_true;

  if(dataType=="readFromCfgFile") dataType = rai::getParameter<str>("ML/dataType", "linear");

  if(dataType=="linear") {
    X = randn(n, d);
    arr Z = makeFeatures(X, "readFromCfgFile");
    arr beta;
    beta = randn(Z.d1, 1).reshape(Z.d1);
    //      if(dataType==linearRedundantData) {
    //        double pr = rai::getParameter<double>("d_p_redundant", .5);
    //        for(uint j=1; j<beta.N; j++) if(rnd.uni()<pr) beta(j)=0.;
    //      }
    y = Z*beta;
    y = y + sigma*randn(y.dim());
    beta_true = beta;
  } else if(dataType=="sinus") {
    X = rai::grid<double>(1, -3, 3, n-1);
    y.resize(X.d0);
    for(uint i=0; i<X.d0; i++) y(i) = sin(X(i, 0));
    y += sigma*randn(y.dim());
  } else if(dataType=="linearOutlier") {
    double rate = rai::getParameter<double>("ML/dataOutlierRate", .1);
    X = randn(n, d);
    arr Z = makeFeatures(X, "readFromCfgFile");
    arr beta;
    beta = randn(Z.d1, 1).reshape(Z.d1);
    y = Z*beta;
    for(uint i=0; i<y.N; i++)  if(rnd.uni()<rate) {
      y(i) += rai::getParameter<double>("ML/dataOutlierSigma", 10.)*rnd.gauss();
    } else {
      y(i) += sigma*rnd.gauss();
    }
    beta_true = beta;
  } else HALT("");
  LOG(0) <<"ground truth beta=" <<beta_true;
  return beta_true;
}

void artificialData_Hasties2Class(arr& X, arr& y) {
  uint n = rai::getParameter<double>("n", 100);
  uint d = rai::getParameter<double>("d", 2);

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
  uint n = rai::getParameter<double>("n", 100);
  uint d = 2; //rai::getParameter<double>("d", 2);
  uint M = rai::getParameter<double>("M", 3);

  arr means(M, 10, d), x(d);

  rndGauss(means);
  for(uint c=0; c<M; c++)  means[c] += ones(10, 1)*~consts((double)c, d);

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
  uint n = rai::getParameter<double>("n", 100);
  uint M = rai::getParameter<double>("M", 3);
  double sig = rai::getParameter<double>("sigma", .2);

  arr means(M, 2), V(M, 2, 2), x(2);

  rndGauss(means);
  rndGauss(V);
  //means.setZero();
  //for(uint c=0;c<M;c++)  means[c]() += arr{c, c};

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

