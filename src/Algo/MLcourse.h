/*  ------------------------------------------------------------------
    Copyright (c) 2011-2020 Marc Toussaint
    email: toussaint@tu-berlin.de

    This code is distributed under the MIT License.
    Please see <root-path>/LICENSE for details.
    --------------------------------------------------------------  */

#pragma once

#include "../Core/array.h"
#include "../Optim/RidgeRegression.h"

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
  void crossValidateSingleLambda(const arr& X, const arr& y, double lambda, uint k_fold, bool permute, arr* beta_k_fold=nullptr, arr* beta_total=nullptr, double* scoreMean=nullptr, double* scoreSDV=nullptr, double* scoreTrain=nullptr);
  void crossValidateMultipleLambdas(const arr& X, const arr& y, const arr& lambdas, uint k_fold, bool permute);
  void plot();
};

//===========================================================================
//
// constructing features from data
//

enum FeatureType { readFromCfgFileFT=0, linearFT=1, quadraticFT, cubicFT, rbfFT=4, piecewiseConstantFT=5, piecewiseLinearFT=6, constFT=7 };
arr makeFeatures(const arr& X, FeatureType featureType=readFromCfgFileFT, const arr& rbfCenters=NoArr, arr& Jacobian=NoArr);

//===========================================================================
//
// artificial test data & data load routines
//

extern arr beta_true;
enum ArtificialDataType { readFromCfgFileDT=0, linearData, sinusData, linearOutlier, linearRedundantData };

arr artificialData(arr& X, arr& y, ArtificialDataType dataType=readFromCfgFileDT);
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
