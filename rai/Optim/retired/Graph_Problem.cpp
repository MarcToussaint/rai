/*  ------------------------------------------------------------------
    Copyright (c) 2011-2020 Marc Toussaint
    email: toussaint@tu-berlin.de

    This code is distributed under the MIT License.
    Please see <root-path>/LICENSE for details.
    --------------------------------------------------------------  */

#include "Graph_Problem.h"
#include "../Core/graph.h"

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

Conv_Graph_MathematicalProgram::Conv_Graph_MathematicalProgram(GraphProblem& _G,  ostream* _log) : G(_G), logFile(_log) {
  G.getStructure(variableDimensions, featureVariables, featureTypes);
  varDimIntegral = integral(variableDimensions);

  if(logFile) {
    StringA varNames, phiNames;
    G.getSemantics(varNames, phiNames);

    rai::arrayElemsep=", ";
    rai::arrayBrackets="[]";

    rai::Graph data = {
      {"graphStructureQuery", true},
      {"numVariables", variableDimensions.N},
      {"variableNames", varNames},
      {"variableDimensions", variableDimensions},
      {"numFeatures", featureVariables.N},
      {"featureNames", phiNames},
      {"featureVariables", featureVariables},
      {"featureTypes", featureTypes},
    };

    data.write(*logFile, ",\n", "{\n\n}");
    (*logFile) <<',' <<endl;
  }
}

#if 0

//dense
void Conv_Graph_MathematicalProgram::phi(arr& phi, arr& J, arr& H, ObjectiveTypeA& tt, const arr& x, arr& lambda) {
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
        memmove(&J(i, (j?varDimIntegral(j-1):0)), Ji.p+c, xjN+Ji.sizeT);
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
uint Conv_Graph_MathematicalProgram::getDimension() {
  return varDimIntegral.elem(-1);
}

void Conv_Graph_MathematicalProgram::getFeatureTypes(ObjectiveTypeA& ft) {
  if(!!ft) ft = featureTypes;
}

void Conv_Graph_MathematicalProgram::evaluate(arr& phi, arr& J, const arr& x) {
  G.phi(phi, J_G, H_G, x);

  //-- construct a sparse J from the array of feature Js
  if(!!J) {
    //count non-zeros!
    uint k=0;
    for(uint i=0; i<J_G.N; i++) {
      arr& Ji = J_G(i);
      if(!isSparseVector(Ji)) {
        for(uint j=0; j<Ji.N; j++) if(Ji(j)) k++;
      } else {
        k += Ji.sparseVec().Z.N;
      }
    }

    //any sparse matrix is k-times-3
    J.sparse().resize(phi.N, varDimIntegral.last(), k);
    J.setZero();

    k=0;
    for(uint i=0; i<J_G.N; i++) { //loop over features
      arr& Ji = J_G(i);
      if(!isSparseVector(Ji)) {
        CHECK(!isSpecial(Ji), "");
        uint c=0;
        for(int& j:featureVariables(i)) if(j>=0) { //loop over variables of this features
            uint xj_dim = variableDimensions(j);
            for(uint xi=0; xi<xj_dim; xi++) { //loop over variable dimension
              double J_value = Ji.elem(c);
              if(J_value) {
                uint jj = (j?varDimIntegral(j-1):0) + xi;
                J.sparse().entry(i, jj, k) = J_value;
                k++;
              }
              c++;
            }
          }
        CHECK_EQ(c, Ji.N, "you didn't count through all indexes");
      } else { //sparse vector
        for(uint l=0; l<Ji.N; l++) {
          double J_value = Ji.elem(l);
          uint   xi      = Ji.sparseVec().elems(l); //column index
          for(int& j:featureVariables(i)) if(j>=0) {
              uint xj_dim = variableDimensions(j);
              if(xi<xj_dim) { //xj is the variable, and xi is the index within the variable
                uint jj = (j?varDimIntegral(j-1):0) + xi;
                J.sparse().entry(i, jj, k) = J_value;
                k++;
              }
              xi -= xj_dim; //xj is not yet the variable, skip over xj, and shift xi
            }
        }
      }
    }
    CHECK_EQ(k, J.N, ""); //one entry for each non-zero
  }

  if(logFile) {
    arr err = summarizeErrors(phi, featureTypes);

    rai::Graph data = {
      {"graphQuery", queryCount},
      {"errors", err},
      {"x", x},
      {"phi", phi},
      {"J", J}
    };

    data.write(*logFile, ",\n", "{\n\n}");
    (*logFile) <<',' <<endl;
  }

  queryCount++;
}

void Conv_Graph_MathematicalProgram::reportProblem(std::ostream& os) {
  uint nG=0, nH=0;
    for(ObjectiveType t:featureTypes) if(t==OT_ineq) nG++; else if(t==OT_eq) nH++;
  os <<"\n# GraphProblem";
  os <<"\n# num_vars: " <<variableDimensions.N <<" num_feat: " <<featureTypes.N <<" num_ineq: " <<nG <<" num_eq: " <<nH;
  StringA varNames, phiNames;
  G.getSemantics(varNames, phiNames);
  os <<"\n# vars: ";
  for(uint i=0; i<varNames.N; i++) os <<"\n#   " <<varNames.elem(i) <<"  (" <<variableDimensions.elem(i) <<")";
  os <<"\n# features: ";
  for(uint i=0; i<phiNames.N; i++) os <<"\n#   " <<phiNames.elem(i) <<"  (" <<featureTypes.elem(i) <<")";
  os <<endl;
}

#endif

void ModGraphProblem::getStructure(uintA& variableDimensions, intAA& featureVariables, ObjectiveTypeA& featureTypes) {
  G.getStructure(variableDimensions, featureVariables, featureTypes);
  for(uint i=0; i<featureVariables.N; i++) {
    if(featureTypes(i)==OT_ineq || featureTypes(i)==OT_eq) {
      subselectFeatures.append(i);
    }
  }
  featureVariables = featureVariables.sub(subselectFeatures);
  featureTypes = featureTypes.sub(subselectFeatures);
}

void ModGraphProblem::getSemantics(StringA& varNames, StringA& phiNames) {
  G.getSemantics(varNames, phiNames);
  phiNames = phiNames.sub(subselectFeatures);
}

void ModGraphProblem::phi(arr& phi, arrA& J, arrA& H, const arr& x) {
  G.phi(phi, J, H, x);
  phi = phi.sub(subselectFeatures);
  J = J.sub(subselectFeatures);
}

