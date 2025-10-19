/*  ------------------------------------------------------------------
    Copyright (c) 2011-2024 Marc Toussaint
    email: toussaint@tu-berlin.de

    This code is distributed under the MIT License.
    Please see <root-path>/LICENSE for details.
    --------------------------------------------------------------  */

#include "options.h"

namespace rai {

std::shared_ptr<OptOptions> globalOptOptions() {
  static std::shared_ptr<OptOptions> opt;
  if(!opt) opt=std::make_shared<OptOptions>();
  return opt;
}

template<> const char* Enum<OptMethod>::names []= {
    "noMethod", "GradientDescent", "Rprop", "LBFGS", "Newton",
    "AugmentedLag", "LogBarrier", "slackGN_logBarrier",  "squaredPenalty", "singleSquaredPenalty",
    "slackGN",
    "NLopt", "Ipopt", "slackGN_Ipopt", "Ceres",
    "LSZO", "greedy", "NelderMead", "CMA",
    nullptr
};

}
