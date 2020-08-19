/*  ------------------------------------------------------------------
    Copyright (c) 2011-2020 Marc Toussaint
    email: toussaint@tu-berlin.de

    This code is distributed under the MIT License.
    Please see <root-path>/LICENSE for details.
    --------------------------------------------------------------  */

#pragma once

#include "minEigModel.h"
#include "../Algo/dataNeighbored.h"
#include "../Gui/opengl.h"

struct ModelEnsemble :GLDrawer {
  rai::Array<MinEigModel*> models;

  arr vert;

  ModelEnsemble();
  ~ModelEnsemble();

  bool addNewRegionGrowingModel(DataNeighbored& data);
  void reestimateVert();
  void reoptimizeModels(DataNeighbored& data);

  void glDraw(OpenGL&);

  void report(ostream& os=cout, bool mini=true);
};
