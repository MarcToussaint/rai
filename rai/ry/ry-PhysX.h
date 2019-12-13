#pragma once

#include <memory>

struct PhysXInterface;

namespace ry {

struct RyPhysX { std::shared_ptr<PhysXInterface> physx; };

};
