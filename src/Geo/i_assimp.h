/*  ------------------------------------------------------------------
    Copyright (c) 2011-2024 Marc Toussaint
    email: toussaint@tu-berlin.de

    This code is distributed under the MIT License.
    Please see <root-path>/LICENSE for details.
    --------------------------------------------------------------  */

#pragma once

#include "mesh.h"

struct AssimpLoader {
  rai::Array<MeshA> meshes;
  rai::Array<rai::Transformation> poses;
  StringA names;
  StringA parents;
  arr masses;
  std::string directory;
  int verbose=0;

  AssimpLoader(const std::string& path, bool flipYZ=true, bool relativeMeshPoses=false, int _verbose=0);
  AssimpLoader(const struct aiScene* scene);

  rai::Mesh getSingleMesh();

 private:
  void loadNode(const struct aiNode* node, const struct aiScene* scene, arr T, bool relativeMeshPoses);
  rai::Mesh loadMesh(const struct aiMesh* mesh, const struct aiScene* scene);
};

void buildAiMesh(const rai::Mesh& M, struct aiMesh* pMesh);
void writeAssimp(const rai::Mesh& M, const char* filename, const char* format);
