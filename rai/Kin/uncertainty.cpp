/*  ------------------------------------------------------------------
    Copyright (c) 2011-2020 Marc Toussaint
    email: toussaint@tu-berlin.de

    This code is distributed under the MIT License.
    Please see <root-path>/LICENSE for details.
    --------------------------------------------------------------  */

#include "uncertainty.h"

rai::Uncertainty::Uncertainty(rai::Joint* j, rai::Uncertainty* copyUncertainty) : joint(j), sigma(.1) {
  CHECK(!j->uncertainty, "the Joint already has an Uncertainty");
  j->uncertainty = this;

  if(copyUncertainty) {
    sigma = copyUncertainty->sigma;
  }
}

void rai::Uncertainty::read(const Graph& ats) {
  ats.get(sigma, "sigma");
  CHECK_EQ(sigma.N, joint->qDim(), "");
}
