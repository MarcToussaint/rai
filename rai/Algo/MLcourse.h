/*  ------------------------------------------------------------------
    Copyright (c) 2017 Marc Toussaint
    email: marc.toussaint@informatik.uni-stuttgart.de

    This code is distributed under the MIT License.
    Please see <root-path>/LICENSE for details.
    --------------------------------------------------------------  */

#ifndef RAI_linearRegression_h
#define RAI_linearRegression_h

#include <Core/array.h>

//===========================================================================
//
// basic regression and classification methods
//

arr ridgeRegression(const arr& X, const arr& y, double lambda=-1., arr& bayesSigma=NoArr, const arr& weighted=NoArr, arr& zScores=NoArr);
arr evaluateBayesianRidgeRegressionSigma(const arr& X, const arr& bayesSigma);

arr logisticRegression2Class(const arr& X, const arr& y, double lambda=-1., arr& bayesSigma=NoArr);
arr logisticRegressionMultiClass(const arr& X, const arr& y, double lambda=-1.);

struct RidgeRegression {
  arr beta; ///< (X^T X + lambda I)^-1 X^T y
  arr XtX_I; ///< (X^T X + lambda I)
  double sigmaSqr; ///< mean squared error on training data; estimate of noise
  arr betaSigmaMatrix; ///< variance (matrix) of estimated beta
  
  RidgeRegression(const arr& X, const arr& y, double lambda=-1, const arr& weighted=NoArr, int verbose=1);
  arr evaluate(const arr& X, arr& bayesSigma2=NoArr);
  
  arr getBetaSigmaMatrix();
  arr getBetaZscores();
  arr getMultiOutputSquaredErrors(const arr& X, const arr& y);
};

struct DefaultKernelFunction : KernelFunction {
  enum KernelType { readFromCfg=0, Gauss=1 } type;
  arr hyperParam1,hyperParam2;
  DefaultKernelFunction(KernelType _type=readFromCfg):type(_type) {}
  virtual double k(const arr& x1, const arr& x2, arr& gx1, arr& Hx1);
};
extern DefaultKernelFunction defaultKernelFunction;

struct KernelRidgeRegression {
  arr X; ///< stored data (to compute kappa for queries)
  arr kernelMatrix_lambda; ///< X X^T + lambda I
  arr invKernelMatrix_lambda; ///< (X X^T + lambda I)^-1
  arr alpha; ///< (X X^T + lambda I)^-1 y
  double sigmaSqr; ///< mean squared error on training data; estimate of noise
  double mu; ///< fixed global bias (default=0)
  KernelFunction& kernel;
  KernelRidgeRegression(const arr& X, const arr& y, KernelFunction& kernel=defaultKernelFunction, double lambda=-1, double mu=0.);
  arr evaluate(const arr& X, arr& bayesSigma2=NoArr); ///< returns f(x) and \s^2(x) for a set of points X
  
  double evaluate(const arr& x, arr& df_x, arr& H, double plusSigma, bool onlySigma); ///< returns f(x) + coeff*\sigma(x) and its gradient and Hessian
  ScalarFunction getF(double plusSigma);
};

struct KernelLogisticRegression {
  arr X; ///< stored data (to compute kappa for queries)
  arr kernelMatrix_lambda; ///< X X^T + 2 lambda W^-1
  arr invKernelMatrix_lambda;
  arr alpha; ///< (X X^T + 2 lambda W^-1)^-1 (f - (p-y)/w)
  double lambda;
  double mu; ///< fixed global bias (default=0)
  KernelFunction& kernel;
  KernelLogisticRegression(const arr& X, const arr& y, KernelFunction& kernel=defaultKernelFunction, double lambda=-1, double mu=0.);
  arr evaluate(const arr& X, arr &p_bayes=NoArr, arr& p_hi=NoArr, arr& p_lo=NoArr);
  arr evaluateF(const arr& X, arr& bayesSigma2=NoArr);
};

struct KernelCRF {
};

//===========================================================================
//
// cross validation
//

struct CrossValidation {
  arr scoreMeans, scoreSDVs, scoreTrains, lambdas;
  bool verbose = true;
  
  virtual void  train(const arr& X, const arr& y, double lambda, arr& beta) = 0;
  virtual double test(const arr& X, const arr& y, const arr& beta) = 0;
  
  //beta_k_fold will contain k parameter sets for the partitions
  void crossValidateSingleLambda(const arr& X, const arr& y, double lambda, uint k_fold, bool permute, arr* beta_k_fold=NULL, arr *beta_total=NULL, double *scoreMean=NULL, double *scoreSDV=NULL, double *scoreTrain=NULL);
  void crossValidateMultipleLambdas(const arr& X, const arr& y, const arr& lambdas, uint k_fold, bool permute);
  void plot();
};

//===========================================================================
//
// constructing features from data
//

enum FeatureType { readFromCfgFileFT=0, linearFT=1, quadraticFT, cubicFT, rbfFT=4, piecewiseConstantFT=5, piecewiseLinearFT=6, constFT=7 };
arr makeFeatures(const arr& X, FeatureType featureType=readFromCfgFileFT, const arr& rbfCenters=NoArr);

//===========================================================================
//
// artificial test data & data load routines
//

extern arr beta_true;
enum ArtificialDataType { readFromCfgFileDT=0, linearData, sinusData, linearOutlier, linearRedundantData };

void artificialData(arr& X, arr& y, ArtificialDataType dataType=readFromCfgFileDT);
void artificialData_1D2Class(arr& X, arr& y);
void artificialData_Hasties2Class(arr& X, arr& y);
void artificialData_HastiesMultiClass(arr& X, arr& y);
void artificialData_GaussianMixture(arr& X, arr& y);
void load_data(arr& X, const char* filename, bool whiten);

//===========================================================================
//
// helper
//

double NormalSdv(const double& a, const double& b, double sdv);

#endif
