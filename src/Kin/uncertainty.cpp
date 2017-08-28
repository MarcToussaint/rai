#include "uncertainty.h"

mlr::Uncertainty::Uncertainty(mlr::Joint *j, mlr::Uncertainty *copyUncertainty) : joint(j), sigma(.1){
  CHECK(!j->uncertainty, "the Joint already has an Uncertainty");
  j->uncertainty = this;

  if(copyUncertainty){
    sigma = copyUncertainty->sigma;
  }
}

void mlr::Uncertainty::read(const Graph &ats){
  ats.get(sigma, "sigma");
  CHECK_EQ(sigma.N, joint->qDim(), "");
}
