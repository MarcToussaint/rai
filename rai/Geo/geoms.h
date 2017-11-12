#pragma once

#include <Core/array.h>
#include "mesh.h"

namespace mlr{

enum ShapeType { ST_none=-1, ST_box=0, ST_sphere, ST_capsule, ST_mesh, ST_cylinder, ST_marker, ST_retired_SSBox, ST_pointCloud, ST_ssCvx, ST_ssBox };

struct GeomStore;

struct Geom{
  GeomStore& store;

  uint ID;
  Enum<ShapeType> type;
  arr size;
  Mesh mesh, sscCore;

  Geom(GeomStore& _store);

  ~Geom();

  void read(const Graph &ats);
  void createMeshes();
  void glDraw(OpenGL&);
};

struct GeomStore{
  mlr::Array<Geom*> geoms;

  ~GeomStore();

  Geom& get(uint id);
};

} //namespace mlr

extern Singleton<mlr::GeomStore> _GeomStore;
