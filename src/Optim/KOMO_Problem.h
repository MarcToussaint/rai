/*  ------------------------------------------------------------------
    Copyright 2016 Marc Toussaint
    email: marc.toussaint@informatik.uni-stuttgart.de
    
    This program is free software: you can redistribute it and/or modify
    it under the terms of the GNU General Public License as published by
    the Free Software Foundation, either version 3 of the License, or (at
    your option) any later version. This program is distributed without
    any warranty. See the GNU General Public License for more details.
    You should have received a COPYING file of the full GNU General Public
    License along with this program. If not, see
    <http://www.gnu.org/licenses/>
    --------------------------------------------------------------  */


#pragma once

#include <Core/array.h>
#include "optimization.h"
#include "Graph_Problem.h"

struct KOMO_Problem {
  virtual uint get_k() = 0;
  virtual void getStructure(uintA& variableDimensions, uintA& featureTimes, ObjectiveTypeA& featureTypes)=0;
  virtual void phi(arr& phi, arrA& J, arrA& H, ObjectiveTypeA& tt, const arr& x) = 0;

  bool checkStructure(const arr& x);                 ///< check if Jacobians and Hessians have right dimensions (=clique size)
  void report(const arr& phi=NoArr);
};


//-- converters
struct Conv_KOMO_ConstrainedProblem : ConstrainedProblem{
  KOMO_Problem& KOMO;
  uintA variableDimensions, varDimIntegral;
  uintA featureTimes;
  ObjectiveTypeA featureTypes;
  arrA J_KOMO, H_KOMO;

  Conv_KOMO_ConstrainedProblem(KOMO_Problem& P);

  void phi(arr& phi, arr& J, arr& H, ObjectiveTypeA& tt, const arr& x);
};


struct KOMO_GraphProblem : GraphProblem{
  KOMO_Problem& KOMO;
  KOMO_GraphProblem(KOMO_Problem& P) : KOMO(P){}
  virtual void getStructure(uintA& variableDimensions, uintAA& featureVariables, ObjectiveTypeA& featureTypes);
  virtual void phi(arr& phi, arrA& J, arrA& H, const arr& x);
};
