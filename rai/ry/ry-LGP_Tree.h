#pragma once

#include <memory>

struct LGP_Tree_Thread;

namespace ry {

struct RyLGP_Tree { std::shared_ptr<LGP_Tree_Thread> lgp; };

};
