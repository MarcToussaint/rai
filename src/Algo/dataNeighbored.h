#pragma once

#include <Core/array.h>

struct DataNeighbored{
  arr X; ///< data (features for each point)
  boolA ok; ///< whether the data point is ok (not corrupted)
  MT::Array<uintA> N; ///< neighborhood
  uintA idx2pixel; ///< after removing points, this maps from X-index to original data index
  arr weights;

  DataNeighbored(const arr& pts);

  uint n() const;
  uint d() const;

  void setGridNeighborhood(uint height, uint width);
  uintA getKneighborhood(uint i, uint k);

  void removeNonOk();

  void initFringe(uintA& fringe, uintA& pts, boolA& included, uint i);
  void expandFringe(uintA& fringe, uintA& pts, boolA& included);
};
