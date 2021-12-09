/*  ------------------------------------------------------------------
    Copyright (c) 2011-2020 Marc Toussaint
    email: toussaint@tu-berlin.de

    This code is distributed under the MIT License.
    Please see <root-path>/LICENSE for details.
    --------------------------------------------------------------  */

#include "optimization.h"

namespace rai {

OptOptions& globalOptOptions() {
  static OptOptions opt;
  return opt;
}

template<> const char* Enum<ConstrainedMethodType>::names []= {
  "noMethod", "squaredPenalty", "augmentedLag", "logBarrier", "anyTimeAula", "squaredPenaltyFixed", nullptr
};

}
