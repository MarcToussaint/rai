/*  ------------------------------------------------------------------
    Copyright (c) 2011-2020 Marc Toussaint
    email: toussaint@tu-berlin.de

    This code is distributed under the MIT License.
    Please see <root-path>/LICENSE for details.
    --------------------------------------------------------------  */

#pragma once

#include "mesh.h"

struct AssimpLoader {
  std::vector<rai::Mesh> meshes;
  std::string directory;
  int verbose=0;

  AssimpLoader(std::string const& path, bool flipYZ=true);
  AssimpLoader(const struct aiScene* scene);

  rai::Mesh getSingleMesh();

 private:
  void loadNode(const struct aiNode* node, const struct aiScene* scene, arr T);
  rai::Mesh loadMesh(const struct aiMesh* mesh, const struct aiScene* scene);
};
