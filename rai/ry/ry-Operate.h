#pragma once

#include <memory>

struct RobotOperation;

namespace ry {

struct RyOperate { std::shared_ptr<RobotOperation> R; };

};
