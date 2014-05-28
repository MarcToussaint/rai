/*  ---------------------------------------------------------------------
    Copyright 2014 Marc Toussaint
    email: marc.toussaint@informatik.uni-stuttgart.de
    
    This program is free software: you can redistribute it and/or modify
    it under the terms of the GNU General Public License as published by
    the Free Software Foundation, either version 3 of the License, or
    (at your option) any later version.
    
    This program is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
    GNU General Public License for more details.
    
    You should have received a COPYING file of the GNU General Public License
    along with this program. If not, see <http://www.gnu.org/licenses/>
    -----------------------------------------------------------------  */



#ifndef MT_linearRegression_h
#define MT_linearRegression_h

#include <Core/array.h>

//===========================================================================
//
// basic regression and classification methods
//

arr ridgeRegression(const arr& X, const arr& y, double lambda=-1., arr& bayesSigma=NoArr, const arr& weighted=NoArr, arr& zScores=NoArr);
arr evaluateBayesianRidgeRegressionSigma(const arr& X, const arr& bayesSigma);

arr logisticRegression2Class(const arr& X, const arr& y, double lambda=-1., arr& bayesSigma=NoArr);
arr logisticRegressionMultiClass(const arr& X, const arr& y, double lambda=-1.);

struct DefaultKernelFunction:KernelFunction{
  enum KernelType{ readFromCfg=0, Gauss=1 } type;
  arr hyperParam1,hyperParam2;
  DefaultKernelFunction(KernelType _type=readFromCfg):type(_type){}
  virtual double k(const arr& x1, const arr& x2, arr& g1, arr& g2);
};
extern DefaultKernelFunction defaultKernelFunction;

struct KernelRidgeRegression{
  arr X; ///< stored data (to compute kappa for queries)
  arr kernelMatrix_lambda; ///< X X^T + lambda I
  arr invKernelMatrix_lambda;
  arr alpha; ///< (X X^T + lambda I)^-1 y
  double sigma; ///< mean squared error on training data; estimate of noise
  KernelFunction& kernel;
  KernelRidgeRegression(const arr& X, const arr& y, KernelFunction& kernel=defaultKernelFunction, double lambda=-1);
  arr evaluate(const arr& X, arr& bayesSigma2=NoArr);
};

struct KernelLogisticRegression{
  arr X; ///< stored data (to compute kappa for queries)
  arr kernelMatrix_lambda; ///< X X^T + 2 lambda W^-1
  arr invKernelMatrix_lambda;
  arr alpha; ///< (X X^T + 2 lambda W^-1)^-1 (f - (p-y)/w)
  double lambda;
  KernelFunction& kernel;
  KernelLogisticRegression(const arr& X, const arr& y, KernelFunction& kernel=defaultKernelFunction, double lambda=-1);
  arr evaluate(const arr& X, arr &p_bayes=NoArr, arr& p_hi=NoArr, arr& p_lo=NoArr);
};

struct KernelCRF{
};

//===========================================================================
//
// cross validation
//

struct CrossValidation {
  arr scoreMeans, scoreSDVs, scoreTrains, lambdas;
  
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

enum FeatureType { readFromCfgFileFT=0, linearFT=1, quadraticFT, cubicFT, rbfFT=4, piecewiseConstantFT=5, piecewiseLinearFT=6 };
arr makeFeatures(const arr& X, FeatureType featureType=readFromCfgFileFT, const arr& rbfCenters=NoArr);


//===========================================================================
//
// artificial test data & data load routines
//

extern arr beta_true;
enum ArtificialDataType { readFromCfgFileDT=0, linearData, sinusData, linearOutlier, linearRedundantData };

void artificialData(arr& X, arr& y, ArtificialDataType dataType=readFromCfgFileDT);
void artificialData_1D2Class(arr& X, arr& y);
void artificialData_Hasties2Class(arr& X, arr& y, uint dim=2);
void artificialData_HastiesMultiClass(arr& X, arr& y);
void artificialData_GaussianMixture(arr& X, arr& y);
void load_data(arr& X, const char* filename, bool whiten);


//===========================================================================
//
// helper
//

double NormalSdv(const double& a, const double& b, double sdv);

#endif
