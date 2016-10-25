/*  ------------------------------------------------------------------
    Copyright 2016 Marc Toussaint
    email: marc.toussaint@informatik.uni-stuttgart.de
    
    This program is free software: you can redistribute it and/or modify
    it under the terms of the GNU General Public License as published by
    the Free Software Foundation, either version 3 of the License, or (at
    your option) any later version. This program is distributed without
    any warranty. See the GNU General Public License for more details.
    You should have received a COPYING file of the full GNU General Public
    License along with this program. If not, see
    <http://www.gnu.org/licenses/>
    --------------------------------------------------------------  */


#pragma once

#include <Core/array.h>

struct DataNeighbored{
  arr X; ///< data (features for each point)
  boolA valid; ///< whether the data point is ok (not corrupted)
  mlr::Array<uintA> N; ///< neighborhood
  uintA idx2pixel; ///< after removing points, this maps from X-index to original data index
  arr isModelledWeights;
  arr costs;  ///< the 'cost' per data point, i.e., weigthing in the cost function

  DataNeighbored(){}
  DataNeighbored(const arr& pts){ setData(pts); }

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
