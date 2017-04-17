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
#include <Algo/eigenValues.h>
#include <Geo/mesh.h>
#include <Algo/dataNeighbored.h>

struct MinEigModel : GLDrawer{
  DataNeighbored& data;
  arr weights;
  double margin;

  //statistics
  arr bias_xx;
  double stat_n;
  arr stat_x, stat_xx;

  //eigen value/vector
  arr mean;
  ExtremeEigenValues eig;

  //used points
  uintA pts;      ///< set of points that this model models (has non-zero weights)
  uintA fringe;   ///< the fringe of model points (subset of pts)
  boolA included; ///< binary indicator encoding of pts (is equivalent to pts)

  //cvx hull
  mlr::Mesh convexHull;
  double density;

  //label
  int label;


  MinEigModel(DataNeighbored& data, double margin) : data(data), weights(zeros(data.n())), margin(margin), label(0) {}
  virtual ~MinEigModel(){}

  void setPoints(const uintA& points); ///< set the model points (weights initialized to one)
  void setWeightsToOne();    ///< set all weights in pts to 1
  void setWeightsToZero();   ///< set all weights (ans stats) to zero
  void calc(bool update);    ///< compute the model (calc the eigenvalue/vector) incremental/update or exact
  void expand(uint steps=1); ///< add all neighbors of the fringe
  void reweightWithError(uintA& pts);
  arr getInliers();
  void computeConvexHull();
  void computeConvexHull2();
  double coveredData(bool novelDataOnly=true);
  void calcDensity();
  void colorPixelsWithWeights(arr& cols);
  void glDraw(OpenGL&);
  void report(ostream& os=std::cout, bool mini=false);

private:
  void addStatistics(const uintA& points, bool minus=false);
};
