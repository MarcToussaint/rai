#pragma once

#include <Core/thread.h>

namespace rai{
    struct Configuration;
    struct Frame;
}

namespace ry{
struct RyFrame {
  ptr<Var_data<rai::Configuration>> config; //only to ensure the containing configuration is not destroyed
  rai::Frame* frame=0;
};
}
