#include "KOMO_Problem.h"

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

