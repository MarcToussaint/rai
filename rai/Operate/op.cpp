/*  ------------------------------------------------------------------
    Copyright (c) 2011-2020 Marc Toussaint
    email: toussaint@tu-berlin.de

    This code is distributed under the MIT License.
    Please see <root-path>/LICENSE for details.
    --------------------------------------------------------------  */

#include "op.h"

template<> const char* rai::Enum<ActStatus>::names []= {
  /*"AS_init", */"AS_running", "AS_done", "AS_converged", "AS_stalled", "AS_true", "AS_false", "AS_kill", nullptr
};
