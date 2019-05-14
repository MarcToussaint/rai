/*  ------------------------------------------------------------------
    Copyright (c) 2017 Marc Toussaint
    email: marc.toussaint@informatik.uni-stuttgart.de

    This code is distributed under the MIT License.
    Please see <root-path>/LICENSE for details.
    --------------------------------------------------------------  */

#include "Graph_Problem.h"

bool GraphProblem::checkStructure(const arr& x) {
  arr y;
  arrA J, H;
  ObjectiveTypeA featureTypes;
  uintA variableDimensions;
  intAA featureVariables;
  getStructure(variableDimensions, featureVariables, featureTypes);

  phi(y, J, H, x);

  uint m=y.N;

  CHECK_EQ(sum(variableDimensions), x.N, "variable dimensions don't match");
  CHECK_EQ(featureVariables.N, m, "");
  CHECK_EQ(J.N, m, "");
  CHECK_EQ(featureTypes.N, m, "");

  for(uint i=0; i<m; i++) {
    uint d=0;
    intA& vars=featureVariables(i);
    for(int& j:vars) if(j>=0) d += variableDimensions(j);
    CHECK_EQ(J(i).N, d, i<<"th Jacobian has wrong dim");
  }
  return true;
}

Conv_Graph_ConstrainedProblem::Conv_Graph_ConstrainedProblem(GraphProblem& _G) : G(_G) {
  G.getStructure(variableDimensions, featureVariables, featureTypes);
  varDimIntegral = integral(variableDimensions);
}

#if 0

//dense
void Conv_Graph_ConstrainedProblem::phi(arr& phi, arr& J, arr& H, ObjectiveTypeA& tt, const arr& x, arr& lambda) {
  G.phi(phi, J_G, H_G, x, lambda);
  
  if(!!tt) tt = featureTypes;
  
  //-- construct a dense J from the array of feature Js
  if(!!J) {
    J.resize(phi.N, x.N).setZero();
    
    for(uint i=0; i<phi.N; i++) { //loop over features
      arr& Ji = J_G(i);
      uint c=0;
      for(uint& j:featureVariables(i)) { //loop over variables of this features
        uint xjN = variableDimensions(j);
#if 0
        memmove(&J(i,(j?varDimIntegral(j-1):0)), Ji.p+c, xjN+Ji.sizeT);
        c+=xjN;
#else
        for(uint xi=0; xi<xjN; xi++) { //loop over variable dimension
          J(i, (j?varDimIntegral(j-1):0) + xi) += Ji.elem(c);
          c++;
        }
#endif
      }
      CHECK_EQ(c, Ji.N, "you didn't count through all indexes");
    }
  }
}

#else

//sparse
void Conv_Graph_ConstrainedProblem::phi(arr& phi, arr& J, arr& H, ObjectiveTypeA& tt, const arr& x, arr& lambda) {
  G.phi(phi, J_G, H_G, x);

  if(!!tt) tt = featureTypes;

  //-- construct a sparse J from the array of feature Js
  if(!!J) {
    //count non-zeros!
    uint k=0;
    for(uint i=0;i<J_G.N;i++){
      arr& Ji = J_G(i);
      for(uint j=0;j<Ji.N;j++) if(Ji(j)) k++;
    }

    //any sparse matrix is k-times-3
    J.sparse().resize(phi.N, varDimIntegral.last(), k);
    J.setZero();

    k=0;
    for(uint i=0; i<J_G.N; i++) { //loop over features
      arr& Ji = J_G(i);
      uint c=0;
      for(int& j:featureVariables(i)) if(j>=0){ //loop over variables of this features
        uint xjN = variableDimensions(j);
        for(uint xi=0; xi<xjN; xi++) { //loop over variable dimension
          double J_value = Ji.elem(c);
          if(J_value){
            uint jj = (j?varDimIntegral(j-1):0) + xi;
            J.sparse().entry(i, jj, k) = J_value;
            k++;
          }
          c++;
        }
      }
      CHECK_EQ(c, Ji.N, "you didn't count through all indexes");
    }
    CHECK_EQ(k, J.N, ""); //one entry for each non-zero
  }
}

#endif
