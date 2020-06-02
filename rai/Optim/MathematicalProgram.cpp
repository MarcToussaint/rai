#include "MathematicalProgram.h"

arr MathematicalProgram::getInitializationSample(const arrL& previousOptima){
  arr blo, bup;
  uint n = getDimension();
  getBounds(blo, bup);
  if(!blo.N){
    return 2.*rand(n)-1.;
  }

  CHECK_EQ(n, blo.N, "");
  CHECK_EQ(n, bup.N, "");
  return blo + rand(n) % (bup - blo);
}

void MathematicalProgram_Structured::getStructure(uintA& variableDimensions, uintA& featureDimensions, intAA& featureVariables){
  variableDimensions = { getDimension() };
  ObjectiveTypeA featureTypes;
  getFeatureTypes(featureTypes);
  featureDimensions = { featureTypes.N };
  featureVariables = { intA({0}) };
}

void MathematicalProgram_Structured::evaluate(arr& phi, arr& J, arr& H, const arr& x)
{
  uintA variableDimensions; //the size of each variable block
  uintA featureDimensions;  //the size of each feature block
  intAA featureVariables;
  getStructure(variableDimensions, //the size of each variable block
               featureDimensions,  //the size of each feature block
               featureVariables    //which variables the j-th feature block depends on
               );
  uint n=0;
  for(uint i=0;i<variableDimensions.N;i++){
    uint d = variableDimensions(i);
    arr xi = x({n,n+d-1});
    setSingleVariable(i, xi);
    n += d;
  }
  CHECK_EQ(n, x.N, "");

  phi.resize(sum(featureDimensions));
  J.resize(phi.N, x.N);

  n=0;
  arr phi_i, J_i, H_i;
  for(uint i=0;i<featureDimensions.N;i++){
    uint d = featureDimensions(i);
    evaluateSingleFeature(i, phi_i, J_i, H_i);
    if(H_i.N){ NIY }
    CHECK_EQ(phi_i.N, d, "");
    CHECK_EQ(J_i.d0, d, "");
    phi({n,n+d-1}) = phi_i;
    if(!!J){
      NIY; //fill the Jacobian, shifting indices...
    }
    n += d;
  }
  CHECK_EQ(n, phi.N, "");
}

//void MathematicalProgram_Structured::setSingleVariable(uint var_id, const arr& x){
//  x_buffer = x;
//}

//void MathematicalProgram::evaluateSingleFeature(uint feat_id, arr& phi, arr& J, arr& H){
//  evaluate(phi, J, H, x_buffer);
//}
