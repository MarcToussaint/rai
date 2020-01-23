/*  ------------------------------------------------------------------
    Copyright (c) 2019 Marc Toussaint
    email: marc.toussaint@informatik.uni-stuttgart.de

    This code is distributed under the MIT License.
    Please see <root-path>/LICENSE for details.
    --------------------------------------------------------------  */

#pragma once

#include "frame.h"

namespace rai {

struct Uncertainty {
  Joint* joint;
  arr sigma;

  Uncertainty(Joint* j, Uncertainty* copyUncertainty=nullptr);

  void read(const Graph& ats);
};

}//namespace rai
