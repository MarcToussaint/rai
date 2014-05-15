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

void linearRegression(arr& beta, const arr& X, const arr& y, const arr* weighted=NULL);
void ridgeRegression(arr& beta, const arr& X, const arr& y, double lambda=-1., const arr* weighted=NULL, arr* zScores=NULL);
void logisticRegression2Class(arr& beta, const arr& X, const arr& y, double lambda=-1.);
void logisticRegressionMultiClass(arr& beta, const arr& X, const arr& y, double lambda=-1.);


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
void makeFeatures(arr& Phi, const arr& X, FeatureType featureType=readFromCfgFileFT, const arr& rbfCenters=NoArr);


//===========================================================================
//
// artificial test data & data load routines
//

extern arr beta_true;
enum ArtificialDataType { readFromCfgFileDT=0, linearData, sinusData, linearOutlier, linearRedundantData };

void artificialData(arr& X, arr& y, ArtificialDataType dataType=readFromCfgFileDT);
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
