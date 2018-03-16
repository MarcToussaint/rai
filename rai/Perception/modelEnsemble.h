/*  ------------------------------------------------------------------
    Copyright (c) 2017 Marc Toussaint
    email: marc.toussaint@informatik.uni-stuttgart.de
    
    This code is distributed under the MIT License.
    Please see <root-path>/LICENSE for details.
    --------------------------------------------------------------  */

#pragma once

#include <Algo/dataNeighbored.h>
#include "minEigModel.h"
#include <Gui/opengl.h>

struct ModelEnsemble :GLDrawer{
  mlr::Array<MinEigModel*> models;

  arr vert;

  ModelEnsemble();
  ~ModelEnsemble();

  bool addNewRegionGrowingModel(DataNeighbored& data);
  void reestimateVert();
  void reoptimizeModels(DataNeighbored& data);

  void glDraw(OpenGL &);

  void report(ostream& os=cout, bool mini=true);
};
