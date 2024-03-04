/*  ------------------------------------------------------------------
    Copyright (c) 2011-2024 Marc Toussaint
    email: toussaint@tu-berlin.de

    This code is distributed under the MIT License.
    Please see <root-path>/LICENSE for details.
    --------------------------------------------------------------  */

#pragma once

#include "../Core/array.h"

//===========================================================================
//
// Approximate Nearest Neighbor Search (kd-tree)
//

struct ANN {
  unique_ptr<struct sANN> self;

  arr X;       //the data set for which a ANN tree is build
  uint bufferSize; //a tree is only rebuild if there are more than 'buffer' new points appended [default: 20]

  ANN();
  ANN(const ANN& ann);
  ~ANN();

  void clear();              //clears the tree and X
  void setX(const arr& _X);  //set X
  void append(const arr& x); //append to X
  void calculate();          //compute a tree for all of X

  void getkNN(arr& sqrDists, uintA& idx, const arr& x, uint k, double eps=.0, bool verbose=false); //core method

  uint getNN(const arr& x, double eps=.0, bool verbose=false);
  void getkNN(uintA& idx, const arr& x, uint k, double eps=.0, bool verbose=false);
  void getkNN(arr& X, const arr& x, uint k, double eps=.0, bool verbose=false);
};
