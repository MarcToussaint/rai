#include "op.h"

template<> const char* rai::Enum<ActStatus>::names []={
  /*"AS_init", */"AS_running", "AS_done", "AS_converged", "AS_stalled", "AS_true", "AS_false", "AS_kill", nullptr
};
