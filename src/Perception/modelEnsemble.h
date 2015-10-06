#pragma once

#include <Algo/dataNeighbored.h>
#include <Algo/minEigModel.h>
#include <Gui/opengl.h>

struct ModelEnsemble :OpenGL::GLDrawer{
  mlr::Array<MinEigModel*> models;

  arr vert;

  ModelEnsemble();

  bool addNewRegionGrowingModel(DataNeighbored& D);
  void reestimateVert();
  void reoptimizeModels(DataNeighbored& D);

  void glDraw(OpenGL &);

  void report(ostream& os=cout);
};
