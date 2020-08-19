/*  ------------------------------------------------------------------
    Copyright (c) 2011-2020 Marc Toussaint
    email: toussaint@tu-berlin.de

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
