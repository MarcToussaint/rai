#include "Graph_Problem.h"

Graph_ConstrainedProblem::Graph_ConstrainedProblem(GraphProblem& _G) : G(_G){
  G.getStructure(variableDimensions, featureVariables, featureTypes);
  varDimIntegral = integral(variableDimensions);

  ConstrainedProblem::operator=( [this](arr& phi, arr& J, arr& H, TermTypeA& tt, const arr& x) -> void{
    f(phi, J, H, tt, x);
  } );
}

void Graph_ConstrainedProblem::f(arr& phi, arr& J, arr& H, TermTypeA& tt, const arr& x) {
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
