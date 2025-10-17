/*  ------------------------------------------------------------------
    Copyright (c) 2011-2024 Marc Toussaint
    email: toussaint@tu-berlin.de

    This code is distributed under the MIT License.
    Please see <root-path>/LICENSE for details.
    --------------------------------------------------------------  */

#include "options.h"

namespace rai {

OptOptions& globalOptOptions() {
  static OptOptions opt;
  return opt;
}

template<> const char* Enum<OptMethod>::names []= {
    "noMethod", "gradientDescent", "rprop", "LBFGS", "newton",
    "augmentedLag", "logBarrier", "slackGN_logBarrier",  "squaredPenalty", "singleSquaredPenalty",
    "slackGN",
    "NLopt", "Ipopt", "slackGN_Ipopt", "Ceres",
    "LSZO", "greedy", "NelderMead",
    nullptr
};

}
