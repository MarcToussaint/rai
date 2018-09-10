/*  ------------------------------------------------------------------
    Copyright (c) 2017 Marc Toussaint
    email: marc.toussaint@informatik.uni-stuttgart.de

    This code is distributed under the MIT License.
    Please see <root-path>/LICENSE for details.
    --------------------------------------------------------------  */

#include "KOMO_Problem.h"
#include "Graph_Problem.h"

bool KOMO_Problem::checkStructure(const arr& x) {
  arr y;
  arrA J, H;
  ObjectiveTypeA tt, featureTypes;
//  uint T=get_T();
  uintA variableDimensions, featureTimes, phiTimes;
  getStructure(variableDimensions, featureTimes, featureTypes);
  uintA varDimIntegral = integral(variableDimensions);
  
  phi(y, J, H, phiTimes, tt, x, NoArr);
  
#ifndef RAI_NOCHECK
  uint m=y.N;
  uint k=get_k();

  CHECK_EQ(tt, featureTypes,"");
  CHECK_EQ(sum(variableDimensions), x.N, "variable dimensions don't match");
  CHECK_EQ(featureTimes.N, m, "");
  CHECK_EQ(J.N, m, "");
  CHECK_EQ(tt.N, m, "");
  
  for(uint i=0; i<m; i++) {
    uint t=featureTimes(i);
    uint d=varDimIntegral(t) - (t>k? varDimIntegral(t-k-1) : 0);
    CHECK_EQ(J(i).N, d, i<<"th Jacobian has wrong dim");
  }
#endif
  return true;
}

void KOMO_Problem::report(const arr& phi) {
  ObjectiveTypeA featureTypes;
  uint k=get_k();
  uintA variableDimensions, featureTimes;
  getStructure(variableDimensions, featureTimes, featureTypes);
  
  cout <<"KOMO Problem report:  k=" <<k <<"  Features:" <<endl;
  for(uint i=0; i<featureTimes.N; i++) {
    cout <<i <<" t=" <<featureTimes(i) <<" vardim=" <<variableDimensions(featureTimes(i)) <<" type=" <<featureTypes(i);
    if(!!phi) cout <<" phi=" <<phi(i) <<" phi^2=" <<rai::sqr(phi(i));
    cout <<endl;
  }
}

void KOMO_GraphProblem::getStructure(uintA& variableDimensions, uintAA& featureVariables, ObjectiveTypeA& featureTypes) {
  uintA featureTimes;
  KOMO.getStructure(variableDimensions, featureTimes, featureTypes);
  
  uint k=KOMO.get_k();
  uint m=featureTypes.N;
  CHECK_EQ(featureTimes.N, m, "");
  
  //-- tuples (x_{t-k:t})
  featureVariables.resize(m);
  for(uint i=0; i<m; i++) {
    int first = featureTimes(i) - k;
    if(first<0) first=0;
    featureVariables(i).clear();
    for(uint j=first; j<=featureTimes(i); j++) featureVariables(i).append(j);
  }
}

void KOMO_GraphProblem::phi(arr& phi, arrA& J, arrA& H, const arr& x, arr& lambda) {
  ObjectiveTypeA featureTypes; //TODO: redundant -> remove
  KOMO.phi(phi, J, H, NoUintA, featureTypes, x, lambda);
}

Conv_KOMO_ConstrainedProblem::Conv_KOMO_ConstrainedProblem(KOMO_Problem& P) : KOMO(P) {
  KOMO.getStructure(variableDimensions, featureTimes, featureTypes);
  varDimIntegral = integral(variableDimensions);
}

void Conv_KOMO_ConstrainedProblem::phi(arr& phi, arr& J, arr& H, ObjectiveTypeA& featureTypes, const arr& x, arr& lambda) {
  KOMO.phi(phi, (!!J?J_KOMO:NoArrA), (!!H?H_KOMO:NoArrA), featureTimes, featureTypes, x, lambda);
  
  //-- construct a row-shifed J from the array of featureJs
  if(!!J) {
    uint k=KOMO.get_k();
    uint dim_xmax = max(variableDimensions);
    RowShifted *Jaux = makeRowShifted(J, phi.N, (k+1)*dim_xmax, x.N);
    J.setZero();
    
    //loop over features
    for(uint i=0; i<phi.N; i++) {
      arr& Ji = J_KOMO(i);
      CHECK_LE(Ji.N, J.d1,"");
      //        J({i, 0, J_KOMO(i}).N-1) = J_KOMO(i);
      memmove(&J(i,0), Ji.p, Ji.sizeT*Ji.N);
      uint t=featureTimes(i);
      if(t<=k) Jaux->rowShift(i) = 0;
      else Jaux->rowShift(i) =  varDimIntegral(t-k-1);
    }
    
    Jaux->reshift();
    Jaux->computeColPatches(true);
  }
  
  if(!!H) {
    bool hasFterm = false;
    if(!!featureTypes) hasFterm = (featureTypes.findValue(OT_f) != -1);
    if(hasFterm) {
      CHECK(H_KOMO.N, "this problem has f-terms -- I need a Hessian!");
      NIY
    } else {
      H.clear();
    }
  }
}
