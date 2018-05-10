/*  ------------------------------------------------------------------
    Copyright (c) 2017 Marc Toussaint
    email: marc.toussaint@informatik.uni-stuttgart.de

    This code is distributed under the MIT License.
    Please see <root-path>/LICENSE for details.
    --------------------------------------------------------------  */

/// Vien Ngo added CMA-ES algorithm
/// @file
/// @ingroup group_Core
/// @addtogroup group_Core
/// @{

#ifndef RAI_cma_h
#define RAI_cma_h

#include <Core/array.h>

//===========================================================================
//
// The standard CMA-ES algorithm
//

struct CMA {
  arr m; //the mean
  arr C; //the covariance matrix
  double sigma; //the step-size
  uint dim;
  
  CMA();//
  ~CMA();
  
  
  void run();
};

#endif

/// @} //end group
