/*  ------------------------------------------------------------------
    Copyright (c) 2017 Marc Toussaint
    email: marc.toussaint@informatik.uni-stuttgart.de

    This code is distributed under the MIT License.
    Please see <root-path>/LICENSE for details.
    --------------------------------------------------------------  */

/// @file
/// @ingroup group_Core
/// @addtogroup group_Core
/// @{

#ifndef RAI_ann_h
#define RAI_ann_h

#include <Core/array.h>

//===========================================================================
//
// Approximate Nearest Neighbor Search (kd-tree)
//

struct ANN {
  struct sANN *s;
  
  arr X;       //the data set for which a ANN tree is build
  uint bufferSize; //a tree is only rebuild if there are more than 'buffer' new points appended [default: 20]
  
  ANN();
  ANN(const ANN& ann);
  ~ANN();

  void clear();              //clears the tree and X
  void setX(const arr& _X);  //set X
  void append(const arr& x); //append to X
  void calculate();          //compute a tree for all of X
  
  uint getNN(const arr& x, double eps=.0, bool verbose=false);
  void getkNN(intA& idx, const arr& x, uint k, double eps=.0, bool verbose=false);
  void getkNN(arr& sqrDists, intA& idx, const arr& x, uint k, double eps=.0, bool verbose=false);
  void getkNN(arr& X, const arr& x, uint k, double eps=.0, bool verbose=false);
};

#endif

/// @} //end group
