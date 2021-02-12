/*  ------------------------------------------------------------------
    Copyright (c) 2011-2020 Marc Toussaint
    email: toussaint@tu-berlin.de

    This code is distributed under the MIT License.
    Please see <root-path>/LICENSE for details.
    --------------------------------------------------------------  */

#include "mesh_readAssimp.h"

#define STB_IMAGE_IMPLEMENTATION
#include "stb_image.h"

#include <assimp/scene.h>
#include <assimp/Importer.hpp>
#include <assimp/Exporter.hpp>
#include <assimp/postprocess.h>

bool loadTextures = true;

AssimpLoader::AssimpLoader(const std::string& path, bool flipYZ, bool relativeMeshPoses) {
  verbose = 0; //rai::getParameter<double>("Assimp/verbose", 0);

  Assimp::Importer importer;
  const aiScene* scene = importer.ReadFile(path, aiProcess_Triangulate | aiProcess_FlipUVs | aiProcess_CalcTangentSpace);
  if(!scene || scene->mFlags & AI_SCENE_FLAGS_INCOMPLETE || !scene->mRootNode) {
    cout <<"current dir: " <<getcwd_string() <<endl;
    HALT("ERROR::ASSIMP:: " << importer.GetErrorString());
  }

  directory = path.substr(0, path.find_last_of('/'));

  if(verbose>0){
    LOG(0) <<"loading " <<path <<" from directory " <<directory;
  }

  arr T = eye(4);
  if(flipYZ) {
    T(1, 1) = T(2, 2) = 0.;
    T(1, 2) = -1;
    T(2, 1) = +1.;
  }
  loadNode(scene->mRootNode, scene, T, relativeMeshPoses);
}

AssimpLoader::AssimpLoader(const aiScene* scene) {
  arr T = eye(4);
  T(1, 1) = T(2, 2) = 0.;
  T(1, 2) = -1;
  T(2, 1) = +1.;
  loadNode(scene->mRootNode, scene, T, false);
}

rai::Mesh AssimpLoader::getSingleMesh() {
  CHECK(meshes.N, "nothing loaded");
  rai::Mesh M;
  for(auto& _meshes: meshes) for(auto& mesh:_meshes){
    M.addMesh(mesh);
  }
  if(!M.tex.N) M.Tt.clear();
  return M;
}

uint depth=0;
void AssimpLoader::loadNode(const aiNode* node, const aiScene* scene, arr T, bool relativeMeshPoses) {
  arr t(4, 4);
  for(uint i=0; i<4; i++) for(uint j=0; j<4; j++) t(i, j) = node->mTransformation[i][j];

  T = T*t;

  arr R = T.sub(0, 2, 0, 2);
  arr p = T.sub(0, 2, 3, 3).reshape(3);
  arr Rt = ~R;
  arr scales(3);
  for(uint i=0;i<3;i++) scales(i) = 1./length(Rt[i]);
  rai::Transformation X;
  X.pos.set(p);
  X.rot.setMatrix((R%scales).p);

  if(verbose>0){
    LOG(0) <<" loading node '" <<node->mName.C_Str() <<"' of parent '" <<(node->mParent?node->mParent->mName.C_Str():"<nil>");

    cout <<"Transform: T=\n" <<T <<"\n<" <<X <<'>' <<endl;
    cout <<"Trans scaling: " <<scales <<"ortho: ";
    cout <<scalarProduct(Rt[0],Rt[1]) <<' '<<scalarProduct(Rt[0],Rt[2]) <<' '<<scalarProduct(Rt[1],Rt[2]) <<endl;
  }

  names.append(node->mName.C_Str());
  poses.append(X);
  if(node->mParent){
    parents.append(node->mParent->mName.C_Str());
  }else{
    parents.append();
  }
  meshes.append();


  for(unsigned int i = 0; i < node->mNumMeshes; i++)  {
    aiMesh* mesh = scene->mMeshes[node->mMeshes[i]];
    meshes.last().append(loadMesh(mesh, scene));
    rai::Mesh& M = meshes.last().last();
    M.V = M.V * ~R;
    for(uint i=0; i<M.V.d0; i++) M.V[i] += p;
    if(relativeMeshPoses){
      M.transform(-poses.last());
    }
  }

  for(unsigned int i = 0; i < node->mNumChildren; i++) {
    depth++;
    loadNode(node->mChildren[i], scene, T, relativeMeshPoses);
    depth--;
  }
}

rai::Mesh AssimpLoader::loadMesh(const aiMesh* mesh, const aiScene* scene) {
  if(verbose>0){
    LOG(0) <<"loading mesh: #V=" <<mesh->mNumVertices;
  }
  rai::Mesh M;
  M.V.resize(mesh->mNumVertices, 3);
  if(mesh->mNormals) M.Vn.resize(mesh->mNumVertices, 3);
  if(loadTextures && mesh->mTextureCoords[0]) M.tex.resize(mesh->mNumVertices, 2);

  for(unsigned int i = 0; i < mesh->mNumVertices; i++) {
    M.V[i] = ARR(mesh->mVertices[i].x, mesh->mVertices[i].y, mesh->mVertices[i].z);
    if(mesh->mNormals) M.Vn[i] = ARR(mesh->mNormals[i].x, mesh->mNormals[i].y, mesh->mNormals[i].z);
    if(loadTextures && mesh->mTextureCoords[0]) M.tex[i] = ARR(mesh->mTextureCoords[0][i].x, mesh->mTextureCoords[0][i].y);
  }

  M.T.resize(mesh->mNumFaces, 3);
  M.T.setZero();
  for(unsigned int i = 0; i < mesh->mNumFaces; i++) {
    aiFace& face = mesh->mFaces[i];
    //      CHECK_EQ(face.mNumIndices, 3, "");
    if(face.mNumIndices==3)
      M.T[i] = TUP(face.mIndices[0], face.mIndices[1], face.mIndices[2]);
  }
  //cout <<"mean of loaded mesh=" <<M.getCenter() <<endl;

  if(loadTextures && mesh->mTextureCoords[0]) M.Tt = M.T;

  aiMaterial* material = scene->mMaterials[mesh->mMaterialIndex];

  for(uint i=0; i<material->mNumProperties; i++) {
    aiMaterialProperty* m = material->mProperties[i];
    //        cout <<  m->mKey.C_Str() <<endl;
    if(!strcmp(m->mKey.C_Str(), "$clr.diffuse")) {
      float* col = (float*)m->mData;
      if(m->mDataLength>=4*sizeof(float) && col[3]) { //not completely transparent
        M.C = ARR(col[0], col[1], col[2], col[3]);
      }
    }
  }

  uint nTex = material->GetTextureCount(aiTextureType_DIFFUSE);
  if(verbose>0){
    cout <<"material: #textures=" <<nTex <<endl;
  }
  if(loadTextures && nTex) {
    CHECK_EQ(nTex, 1, "");
    aiString str;
    material->GetTexture(aiTextureType_DIFFUSE, 0, &str);

    if(verbose>0){
      cout <<"texture=" <<str.C_Str() <<endl;
    }

//    std::string filename = this->directory + '/' + std::string(str.C_Str());
    std::string filename = std::string(str.C_Str());

    int width, height, nrComponents;
    unsigned char* data = stbi_load(filename.c_str(), &width, &height, &nrComponents, 0);
    if(data) {
      M.texImg.resize(height, width, nrComponents);
      memmove(M.texImg.p, data, M.texImg.N);
      M.C = {1., 1., 1.};
    } else {
      LOG(-1) << "Texture failed to load at path: " <<filename;
    }
    stbi_image_free(data);

    CHECK_EQ(M.Tt.d0, M.T.d0, "");
    CHECK_EQ(M.tex.d0, M.V.d0, "");
    CHECK_EQ(M.texImg.nd, 3, "");
  }

  return M;
}

void buildAiMesh(const rai::Mesh& M, aiMesh* pMesh) {
  pMesh->mVertices = new aiVector3D[ M.V.d0 ];
//  pMesh->mNormals = new aiVector3D[ M.V.d0 ];
  pMesh->mNumVertices = M.V.d0;

//  pMesh->mTextureCoords[ 0 ] = new aiVector3D[ M.V.d0 ];
//  pMesh->mNumUVComponents[ 0 ] = M.V.d0;

  for(uint i=0; i<M.V.d0; i++) {
    pMesh->mVertices[i] = aiVector3D(M.V(i, 0), M.V(i, 1), M.V(i, 2));
//      pMesh->mNormals[ itr - vVertices.begin() ] = aiVector3D( normals[j].x, normals[j].y, normals[j].z );
//      pMesh->mTextureCoords[0][ itr - vVertices.begin() ] = aiVector3D( uvs[j].x, uvs[j].y, 0 );
  }

  pMesh->mFaces = new aiFace[ M.T.d0 ];
  pMesh->mNumFaces = M.T.d0;

  for(uint i=0; i<M.T.d0; i++) {
    aiFace& face = pMesh->mFaces[i];
    face.mIndices = new unsigned int[3];
    face.mNumIndices = 3;
    face.mIndices[0] = M.T(i, 0);
    face.mIndices[1] = M.T(i, 1);
    face.mIndices[2] = M.T(i, 2);
  }
}

void writeAssimp(const rai::Mesh& M, const char* filename, const char* format){
  // create a new scene
  aiScene scene;
  scene.mRootNode = new aiNode("root");
  // create a dummy material
  scene.mMaterials = new aiMaterial* [1];
  scene.mNumMaterials = 1;
  scene.mMaterials[0] = new aiMaterial();
  // create meshes
  scene.mMeshes = new aiMesh *[1];
  scene.mNumMeshes = 1;
  aiMesh* mesh = scene.mMeshes[0] = new aiMesh();
  buildAiMesh(M, mesh);
  mesh->mMaterialIndex = 0;
  // associate with root
  scene.mRootNode->mMeshes = new unsigned[1];
  scene.mRootNode->mNumMeshes = 1;
  scene.mRootNode->mMeshes[0] = 0;
  // export
  Assimp::Exporter exporter;
  exporter.Export(&scene, format, filename);
}
