#include "KOMO_Problem.h"
#include "Graph_Problem.h"

bool KOMO_Problem::checkStructure(const arr& x){
  arr y;
  arrA J, H;
  TermTypeA tt, featureTypes;
//  uint T=get_T();
  uint k=get_k();
  uintA variableDimensions, featureTimes;
  getStructure(variableDimensions, featureTimes, featureTypes);
  uintA varDimIntegral = integral(variableDimensions);

  phi(y, J, H, tt, x);

  CHECK_EQ(tt, featureTypes,"");
  CHECK_EQ(sum(variableDimensions), x.N, "variable dimensions don't match");
  uint m=y.N;
  CHECK_EQ(featureTimes.N, m, "");
  CHECK_EQ(J.N, m, "");
  CHECK_EQ(tt.N, m, "");

  for(uint i=0;i<m;i++){
    uint t=featureTimes(i);
    uint d=varDimIntegral(t) - (t>k? varDimIntegral(t-k-1) : 0);
    CHECK_EQ(J(i).N, d, i<<"th Jacobian has wrong dim");
  }
  return true;
}


void KOMO_GraphProblem::getStructure(uintA& variableDimensions, uintAA& featureVariables, TermTypeA& featureTypes){
  uintA featureTimes;
  KOMO.getStructure(variableDimensions, featureTimes, featureTypes);

  uint k=KOMO.get_k();
  uint m=featureTypes.N;
  CHECK_EQ(featureTimes.N, m, "");

  //-- tuples (x_{t-k:t})
  featureVariables.resize(m);
  for(uint i=0;i<m;i++){
    int first = featureTimes(i) - k;
    if(first<0) first=0;
    featureVariables(i).clear();
    for(uint j=first;j<=featureTimes(i);j++) featureVariables(i).append(j);
  }
}

void KOMO_GraphProblem::phi(arr& phi, arrA& J, arrA& H, const arr& x){
  TermTypeA featureTypes; //TODO: redundant -> remove
  KOMO.phi(phi, J, H, featureTypes, x);
}


KOMO_ConstrainedProblem::KOMO_ConstrainedProblem(KOMO_Problem& P) : KOMO(P){
  KOMO.getStructure(variableDimensions, featureTimes, featureTypes);
  varDimIntegral = integral(variableDimensions);

  ConstrainedProblem::operator=( [this](arr& phi, arr& J, arr& H, TermTypeA& tt, const arr& x) -> void{
    return f(phi, J, H, tt, x);
  } );
}

void KOMO_ConstrainedProblem::f(arr& phi, arr& J, arr& H, TermTypeA& tt, const arr& x){
  KOMO.phi(phi, J_KOMO, H_KOMO, tt, x);

  //-- construct a row-shifed J from the array of featureJs
  if(&J){
    uint k=KOMO.get_k();
    uint dim_xmax = max(variableDimensions);
    RowShiftedPackedMatrix *Jaux = auxRowShifted(J, phi.N, (k+1)*dim_xmax, x.N);
    J.setZero();

    //loop over features
    for(uint i=0; i<phi.N; i++) {
      arr& Ji = J_KOMO(i);
      CHECK(Ji.N<=J.d1,"");
      //        J.refRange(i, 0, J_KOMO(i).N-1) = J_KOMO(i);
      memmove(&J(i,0), Ji.p, Ji.sizeT*Ji.N);
      uint t=featureTimes(i);
      if(t<=k) Jaux->rowShift(i) = 0;
      else Jaux->rowShift(i) =  varDimIntegral(t-k-1);
    }

    Jaux->computeColPatches(true);
  }
}
