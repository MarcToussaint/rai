/*  ------------------------------------------------------------------
    Copyright (c) 2017 Marc Toussaint
    email: marc.toussaint@informatik.uni-stuttgart.de

    This code is distributed under the MIT License.
    Please see <root-path>/LICENSE for details.
    --------------------------------------------------------------  */

#pragma once

#include <Kin/frame.h>

namespace rai {

struct Uncertainty {
  Joint *joint;
  arr sigma;
  
  Uncertainty(Joint *j, Uncertainty *copyUncertainty=NULL);
  
  void read(const Graph &ats);
};

}//namespace rai
