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

/// Vien Ngo added CMA-ES algorithm
/// @file
/// @ingroup group_Core
/// @addtogroup group_Core
/// @{

#ifndef MLR_cma_h
#define MLR_cma_h

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
