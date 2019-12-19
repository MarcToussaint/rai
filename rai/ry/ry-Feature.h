#pragma once

#include <memory>

struct Feature;

namespace ry{
  struct RyFeature { std::shared_ptr<Feature> feature; };
}
