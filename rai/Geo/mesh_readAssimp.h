/*  ------------------------------------------------------------------
    Copyright (c) 2017 Marc Toussaint
    email: marc.toussaint@informatik.uni-stuttgart.de

    This code is distributed under the MIT License.
    Please see <root-path>/LICENSE for details.
    --------------------------------------------------------------  */

#pragma once

#include <Geo/mesh.h>

struct AssimpLoader {
  std::vector<rai::Mesh> meshes;
  std::string directory;

  AssimpLoader(std::string const &path);

  rai::Mesh getSingleMesh();

private:
  void loadNode(struct aiNode *node, const struct aiScene *scene, arr T);
  rai::Mesh loadMesh(struct aiMesh *mesh, const struct aiScene *scene);
};

