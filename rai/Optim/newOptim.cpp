#include "newOptim.h"


void MathematicalProgram::getBounds(arr& bounds_lo, arr& bounds_up){
  bounds_lo.clear();
  bounds_up.clear();
}

void MathematicalProgram::getStructure(uintA& variableDimensions, uintA& featureDimensions, intAA& featureVariables){
  CHECK_EQ(isStructured(), false, "");
  variableDimensions = { getDimension() };
  ObjectiveTypeA featureTypes;
  getFeatureTypes(featureTypes);
  featureDimensions = { featureTypes.N };
  featureVariables = { intA({0}) };
}

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

void MathematicalProgram::setSingleVariable(uint var_id, const arr& x){
  CHECK_EQ(isStructured(), false, "");
  x_buffer = x;
}

void MathematicalProgram::evaluateSingleFeature(uint var_id, arr& phi, arr& J, arr& H){
  CHECK_EQ(isStructured(), false, "");
  evaluate(phi, J, H, x_buffer);
}
