/*  ------------------------------------------------------------------
    Copyright (c) 2011-2020 Marc Toussaint
    email: toussaint@tu-berlin.de

    This code is distributed under the MIT License.
    Please see <root-path>/LICENSE for details.
    --------------------------------------------------------------  */

#include "MathematicalProgram.h"

//===========================================================================

arr summarizeErrors(const arr& phi, const ObjectiveTypeA& tt) {
  arr err = zeros(3);
  for(uint i=0; i<phi.N; i++) {
    if(tt(i)==OT_f) err(0) += phi(i);
    if(tt(i)==OT_sos) err(0) += rai::sqr(phi(i));
    if(tt(i)==OT_ineq && phi(i)>0.) err(1) += phi(i);
    if(tt(i)==OT_eq) err(2) += fabs(phi(i));
  }
  return err;
}

//===========================================================================

arr MathematicalProgram::getInitializationSample(const arr& previousOptima) {
  arr blo, bup;
  uint n = getDimension();
  getBounds(blo, bup);
  if(!blo.N) {
    return 2.*rand(n)-1.;
  }

  CHECK_EQ(n, blo.N, "");
  CHECK_EQ(n, bup.N, "");
  return blo + rand(n) % (bup - blo);
}

//===========================================================================

void MathematicalProgram_Factored::evaluate(arr& phi, arr& J, const arr& x) {
  uintA variableDimensions; //the size of each variable block
  uintA featureDimensions;  //the size of each feature block
  intAA featureVariables;
  getFactorization(variableDimensions, //the size of each variable block
                   featureDimensions,  //the size of each feature block
                   featureVariables    //which variables the j-th feature block depends on
                  );
  uintA varDimIntegral = integral(variableDimensions).prepend(0);

  uint n=0;
  for(uint i=0; i<variableDimensions.N; i++) {
    uint d = variableDimensions(i);
    arr xi = x({n, n+d-1});
    setSingleVariable(i, xi);
    n += d;
  }
  CHECK_EQ(n, x.N, "");

  phi.resize(sum(featureDimensions)).setZero();
  if(!!J) J.resize(phi.N, x.N).setZero();

  n=0;
  arr phi_i, J_i;
  for(uint i=0; i<featureDimensions.N; i++) {
    uint d = featureDimensions(i);
    evaluateSingleFeature(i, phi_i, J_i, NoArr);
    CHECK_EQ(phi_i.N, d, "");
    CHECK_EQ(J_i.d0, d, "");
    phi({n, n+d-1}) = phi_i;
    if(!!J) {
      uint Jii=0;
      for(uint j=0; j<featureVariables(i).N; j++) {
        int varId = featureVariables(i)(j);
        if(varId>=0) {
          uint varDim = variableDimensions(varId);
          J.setMatrixBlock(J_i.sub(0, -1, Jii, Jii+varDim-1), n, varDimIntegral(varId));
          Jii += varDim;
        }
      }
      CHECK_EQ(Jii, J_i.d1, "");
    }
    n += d;
  }
  CHECK_EQ(n, phi.N, "");
}

//===========================================================================

Conv_FactoredNLP_BandedNLP::Conv_FactoredNLP_BandedNLP(MathematicalProgram_Factored& P, uint _maxBandSize, bool _sparseNotBanded)
  : P(P), maxBandSize(_maxBandSize), sparseNotBanded(_sparseNotBanded) {
  P.getFactorization(variableDimensions, //the size of each variable block
                     featureDimensions,  //the size of each feature block
                     featureVariables    //which variables the j-th feature block depends on
                    );
  varDimIntegral = integral(variableDimensions).prepend(0);
  featDimIntegral = integral(featureDimensions).prepend(0);
}

//===========================================================================

void Conv_FactoredNLP_BandedNLP::evaluate(arr& phi, arr& J, const arr& x) {
  CHECK_EQ(x.N, varDimIntegral.last(), "");

  //set all variables
#if 0
  uint n=0;
  for(uint i=0; i<variableDimensions.N; i++) {
    uint d = variableDimensions(i);
    CHECK_EQ(n, varDimIntegral(i), "");
    arr xi = x({n, n+d-1});
    P.setSingleVariable(i, xi);
    n += d;
  }
#else
  P.setAllVariables(x);
#endif

  //evaluate all features individually
  phi.resize(featDimIntegral.last()).setZero();
  arr phi_i;
  J_i.resize(featureDimensions.N);
  for(uint i=0; i<featureDimensions.N; i++) {
    uint d = featureDimensions(i);
    if(d) {
      P.evaluateSingleFeature(i, phi_i, J_i(i), NoArr);
      CHECK_EQ(phi_i.N, d, "");
      if(!!J) CHECK_EQ(J_i.elem(i).d0, d, "");
      phi.setVectorBlock(phi_i, featDimIntegral(i));
    }
  }

  if(!J) return;

  if(sparseNotBanded) {
    //count non-zeros!
    uint k=0;
    for(uint i=0; i<J_i.N; i++) {
      arr& Ji = J_i(i);
      if(!isSparseVector(Ji)) {
        for(uint j=0; j<Ji.N; j++) if(Ji.elem(j)) k++;
      } else {
        k += Ji.sparseVec().Z.N;
      }
    }

    //any sparse matrix is k-times-3
    J.sparse().resize(phi.N, x.N, k);
    J.setZero();

    k=0;
    for(uint i=0; i<J_i.N; i++) { //loop over features
      arr& Ji = J_i(i);
      uint f_dim = featureDimensions(i);
      intA& vars = featureVariables(i);
      if(!isSparseVector(Ji)) {
        CHECK(!isSpecial(Ji), "");
        uint c=0;
        for(uint fi=0; fi<f_dim; fi++) { //loop over feature dimension
          for(int& j:vars) if(j>=0) { //loop over variables of this features
              uint x_dim = variableDimensions.elem(j);
              for(uint xi=0; xi<x_dim; xi++) { //loop over variable dimension
                double J_value = Ji.elem(c);
                if(J_value) {
                  J.sparse().entry(featDimIntegral.elem(i)+fi, varDimIntegral.elem(j)+xi, k) = J_value;
                  k++;
                }
                c++;
              }
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

  } else {

    if(!!J) {
      rai::RowShifted& Jaux = J.rowShifted();
      Jaux.resize(phi.N, x.N, maxBandSize); //(k+1)*dim_xmax
      J.setZero();

      for(uint i=0; i<featureDimensions.N; i++) {
        uint n = featDimIntegral(i);
        uint d = featureDimensions(i);
        if(d) {
          J.setMatrixBlock(J_i(i), n, 0);
          //      memmove(&J(i, 0), J_i.p, J_i.sizeT*J_i.N);
//          intA& vars = featureVariables(i);
//          int t=vars.first();
          //      uint xdim=variableDimensions(t);
          //      for(int s=1;s<vars.N;s++){ xdim+=variableDimensions(t+s); CHECK_EQ(vars(s), t+s, ""); }
          //      CHECK_EQ(xdim, J_i.d1, "");

          if(!isRowShifted(J_i(i))){
            HALT("perhaps the below needs to be enabled!");
            //          for(uint j=n; j<n+d; j++) {
            //            if(t<=0) Jaux.rowShift(j) += 0;
            //            else Jaux.rowShift(j) += varDimIntegral(t);
            //          }
          }
        }
      }
      Jaux.reshift();
      Jaux.computeColPatches(true);
    }
  }
  P.report();
}

//===========================================================================

void MathematicalProgram_Traced::evaluate(arr& phi, arr& J, const arr& x) {
  P.evaluate(phi, J, x);
  if(trace_x){ xTrace.append(x); xTrace.reshape(-1, x.N); }
  if(trace_costs){ if(!featureTypes.N) P.getFeatureTypes(featureTypes); costTrace.append(summarizeErrors(phi, featureTypes)); costTrace.reshape(-1,3);  }
  if(trace_phi && !!phi) { phiTrace.append(phi);  phiTrace.reshape(-1, phi.N); }
  if(trace_J && !!J) { JTrace.append(J);  JTrace.reshape(-1, phi.N, x.N); }
}
