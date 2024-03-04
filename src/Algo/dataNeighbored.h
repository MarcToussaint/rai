/*  ------------------------------------------------------------------
    Copyright (c) 2011-2024 Marc Toussaint
    email: toussaint@tu-berlin.de

    This code is distributed under the MIT License.
    Please see <root-path>/LICENSE for details.
    --------------------------------------------------------------  */

#pragma once

#include "../Core/array.h"

struct DataNeighbored {
  arr X; ///< data (features for each point)
  boolA valid; ///< whether the data point is ok (not corrupted)
  rai::Array<uintA> N; ///< neighborhood
  uintA idx2pixel; ///< after removing points, this maps from X-index to original data index
  arr isModelledWeights;
  arr costs;  ///< the 'cost' per data point, i.e., weigthing in the cost function

  DataNeighbored() {}
  DataNeighbored(const arr& pts) { setData(pts); }

  uint n() const;
  uint d() const;

  void setData(const arr& pts);
  void setCosts(const arr& _costs);
  void setGridNeighborhood(uint height, uint width, bool excludeNonValids=true);
  uintA getKneighborhood(uint i, uint k);

  void removeNonValid();

  void initFringe(uintA& fringe, uintA& pts, boolA& included, uint i);
  void expandFringe(uintA& fringe, uintA& pts, boolA& included);
};
