#pragma once

#include <memory>

struct BulletInterface;

namespace ry {

struct RyBullet { std::shared_ptr<BulletInterface> bullet; };

}
