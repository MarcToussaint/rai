/*  ------------------------------------------------------------------
    Copyright (c) 2019 Marc Toussaint
    email: marc.toussaint@informatik.uni-stuttgart.de

    This code is distributed under the MIT License.
    Please see <root-path>/LICENSE for details.
    --------------------------------------------------------------  */

#include "op.h"

template<> const char* rai::Enum<ActStatus>::names []= {
  /*"AS_init", */"AS_running", "AS_done", "AS_converged", "AS_stalled", "AS_true", "AS_false", "AS_kill", nullptr
};
