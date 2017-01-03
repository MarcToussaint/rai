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


#include "Graph_Problem.h"

Conv_Graph_ConstrainedProblem::Conv_Graph_ConstrainedProblem(GraphProblem& _G) : G(_G){
  G.getStructure(variableDimensions, featureVariables, featureTypes);
  varDimIntegral = integral(variableDimensions);
}

void Conv_Graph_ConstrainedProblem::phi(arr& phi, arr& J, arr& H, ObjectiveTypeA& tt, const arr& x) {
  G.phi(phi, J_G, H_G, x);

  if(&tt) tt = featureTypes;

  //-- construct a row-shifed J from the array of featureJs
  if(&J){
    J.resize(phi.N, x.N).setZero();

    for(uint i=0; i<phi.N; i++) { //loop over features
      arr& Ji = J_G(i);
      uint c=0;
      for(uint& j:featureVariables(i)){ //loop over variables of this features
        uint xjN = variableDimensions(j);
#if 0
        memmove(&J(i,(j?varDimIntegral(j-1):0)), Ji.p+c, xjN+Ji.sizeT);
        c+=xjN;
#else
        for(uint xi=0;xi<xjN;xi++){ //loop over variable dimension
          J(i, (j?varDimIntegral(j-1):0) + xi) = Ji.elem(c);
          c++;
        }
#endif
      }
      CHECK_EQ(c, Ji.N, "you didn't count through all indexes");
    }
  }
}
