/*  ------------------------------------------------------------------
    Copyright (c) 2011-2024 Marc Toussaint
    email: toussaint@tu-berlin.de

    This code is distributed under the MIT License.
    Please see <root-path>/LICENSE for details.
    --------------------------------------------------------------  */

#include "mesh.h"
#include "qhull.h"
#include "assimpInterface.h"
#include "stbImage.h"

#include "../Algo/ann.h"
#include "../Algo/marching_cubes.h"
#include "../Optim/newton.h"
#include "../Core/graph.h"
#include "../Core/h5.h"

#include <limits>
#include <algorithm>
#include <math.h>

#ifdef RAI_PLY
#  include "ply/ply.h"
#endif

#ifdef RAI_VHACD
#  define ENABLE_VHACD_IMPLEMENTATION 1
#  include "vhacd/VHACD.h"
#endif


extern arr id2color(uint id);

namespace rai {

//==============================================================================

template<> const char* Enum<ShapeType>::names []= {
  "box", "sphere", "capsule", "mesh", "cylinder", "marker", "pointCloud", "ssCvx", "ssBox", "ssCylinder", "ssBoxElip", "quad", "camera", "sdf", nullptr
};

//==============================================================================
//
// Mesh code
//

SharedTextureImage& Mesh::texImg(const char* file){
  if(!_texImg){
    if(file){
      auto p = params()->find<shared_ptr<SharedTextureImage>>(file);
      if(p) _texImg = *p;
      else{
        _texImg = make_shared<SharedTextureImage>();
        _texImg->img = rai::loadImage(file);
        _texImg->file.setCarray(file, strlen(file)+1);
        params()->add<shared_ptr<SharedTextureImage>>(file, _texImg);
      }
    }else{
      _texImg = make_shared<SharedTextureImage>();
    }
  }
  return *_texImg;
}

void clearAssetMeshesTextures(){
  NodeL M = params()->findNodesOfType(typeid(shared_ptr<rai::Mesh>));
  for(Node *n:M) delete n;
  NodeL T = params()->findNodesOfType(typeid(shared_ptr<SharedTextureImage>));
  for(Node *n:T) delete n;
}

Mesh::Mesh()
  : glX(0)
    /*parsing_pos_start(0),
        parsing_pos_end(std::numeric_limits<long>::max())*/{}

Mesh::~Mesh() {
}

void Mesh::clear() {
  V.clear(); Vn.clear();
  if(C.nd==2) C.clear();
  T.clear(); Tn.clear();
  isArrayFormatted=false;
  graph.clear();
}

Mesh& Mesh::setBox(bool edgesOnly) {
  clear();
  double verts[24] = {
    -.5, -.5, -.5,
      +.5, -.5, -.5,
      +.5, +.5, -.5,
      -.5, +.5, -.5,
      -.5, -.5, +.5,
      +.5, -.5, +.5,
      +.5, +.5, +.5,
      -.5, +.5, +.5
    };
  uint   tris [36] = {
    0, 3, 2, 2, 1, 0, //bottom
    4, 5, 6, 6, 7, 4, //top
    1, 5, 4, 4, 0, 1,
    3, 7, 6, 6, 2, 3,
    2, 6, 5, 5, 1, 2,
    0, 4, 7, 7, 3, 0
  };
  uint edges[24] = {
    0, 1, 1, 2, 2, 3, 3, 0,
    4, 5, 5, 6, 6, 7, 7, 4,
    0, 4, 1, 5, 2, 6, 3, 7
  };

  V.setCarray(verts, 24);
  V.reshape(8, 3);
  if(!edgesOnly) {
    T.setCarray(tris, 36);
    T.reshape(12, 3);
  } else {
    T.setCarray(edges, 24);
    T.reshape(12, 2);
  }
  Vn.clear(); Tn.clear();
  graph.clear();
  //cout <<V <<endl;  for(uint i=0;i<4;i++) cout <<length(V[i]) <<endl;
  return *this;
}

void Mesh::setBox(const arr& lo, const arr& up, bool edgesOnly) {
  setBox(edgesOnly);
  scale(up-lo);
  translate(.5*(lo+up));
}

Mesh& Mesh::setDot() {
  clear();
  V.resize(1, 3).setZero();
  return *this;
}

void Mesh::setLine(double l) {
  clear();
  V.resize(2, 3).setZero();
  V(0, 2) = -.5*l;
  V(1, 2) = +.5*l;
}

void Mesh::setQuad(double x_width, double y_width, const byteA& __texImg, bool flipY, bool texByReference) {
  clear();
  V = {
    -.5*x_width, -.5*y_width, 0,
      +.5*x_width, -.5*y_width, 0,
      +.5*x_width, +.5*y_width, 0,
      -.5*x_width, +.5*y_width, 0
    };
  T = {
    0, 1, 2, 2, 3, 0
  };
  V.reshape(4, 3);
  T.reshape(2, 3);
  if(__texImg.N) {
    if(texByReference) {
      texImg().img.referTo(__texImg);
    } else {
      texImg().img = __texImg;
    }
//    C = {1.,1.,1.}; //bright color
    if(!flipY) {
      texCoords = {0., 1.,  1., 1.,  1., 0.,  0., 0.};
    } else {
      texCoords = {0., 0.,  1., 0.,  1., 1.,  0., 1.};
    }
    texCoords.reshape(V.d0, 2);
  }
}

void Mesh::setTetrahedron() {
  clear();
  double s2=RAI_SQRT2/3., s6=sqrt(6.)/3.;
  double verts[12] = { 0., 0., 1., 2.*s2, 0., -1./3., -s2, s6, -1./3., -s2, -s6, -1./3. };
  uint   tris [12] = { 0, 1, 2, 0, 2, 3, 0, 3, 1, 1, 3, 2 };
  V.setCarray(verts, 12);
  T.setCarray(tris, 12);
  V.reshape(4, 3);
  T.reshape(4, 3);
}

void Mesh::setOctahedron() {
  clear();
  double verts[18] = {
    1, 0, 0,
    -1, 0, 0,
    0, 1, 0,
    0, -1, 0,
    0, 0, 1,
    0, 0, -1
  };
  uint   tris [24] = {
    4, 0, 2,  4, 2, 1,  4, 1, 3,  4, 3, 0,
    5, 2, 0,  5, 1, 2,  5, 3, 1,  5, 0, 3
  };
  V.setCarray(verts, 18);
  T.setCarray(tris, 24);
  V.reshape(6, 3);
  T.reshape(8, 3);
}

void Mesh::setDodecahedron() {
  clear();
  double a = 1/sqrt(3.), b = sqrt((3.-sqrt(5.))/6.), c=sqrt((3.+sqrt(5.))/6.);
  double verts[60] = {
    a, a, a,
    a, a, -a,
    a, -a, a,
    a, -a, -a,
    -a, a, a,
    -a, a, -a,
    -a, -a, a,
    -a, -a, -a,
    b, c, 0,
    -b, c, 0,
    b, -c, 0,
    -b, -c, 0,
    c, 0, b,
    c, 0, -b,
    -c, 0, b,
    -c, 0, -b,
    0, b, c,
    0, -b, c,
    0, b, -c,
    0, -b, -c
  };
  uint tris [108] = {
    0, 8, 9, 0, 9, 4, 0, 4, 16, 0, 12, 13, 0, 13, 1, 0, 1, 8,
    0, 16, 17, 0, 17, 2, 0, 2, 12, 8, 1, 18, 8, 18, 5, 8, 5, 9,
    12, 2, 10, 12, 10, 3, 12, 3, 13, 16, 4, 14, 16, 14, 6, 16, 6, 17,
    9, 5, 15, 9, 15, 14, 9, 14, 4, 6, 11, 10, 6, 10, 2, 6, 2, 17,
    3, 19, 18, 3, 18, 1, 3, 1, 13, 7, 15, 5, 7, 5, 18, 7, 18, 19,
    7, 11, 6, 7, 6, 14, 7, 14, 15, 7, 19, 3, 7, 3, 10, 7, 10, 11
  };
  V.setCarray(verts, 60);
  T.setCarray(tris, 108);
  V.reshape(20, 3);
  T.reshape(36, 3);
}

void Mesh::setIcosahedron() {
  clear();
  double t = .5*(1.+sqrt(5.));
  V = arr{
      t, 1, 0,
      -t, 1, 0,
      t, -1, 0,
      -t, -1, 0,
      1, 0, t,
      1, 0, -t,
  -1, 0, t,
        -1, 0, -t,
        0, t, 1,
        0, -t, 1,
        0, t, -1,
        0, -t, -1 };
  T = uintA{
      0, 8, 4,  2, 9, 11,
      0, 5, 10,  3, 11, 9,
      2, 4, 9,  4, 2, 0,
      2, 11, 5,  5, 0, 2,
      1, 6, 8,  6, 1, 3,
      1, 10, 7,  7, 3, 1,
      3, 9, 6,  8, 6, 4,
      3, 7, 11,  9, 4, 6,
      0, 10, 8,  10, 5, 7,
      1, 8, 10,  11, 7, 5 };
  V.reshape(12,3);
  T.reshape(-1,3);
  for(uint i=0; i<V.d0; i++) V[i] /= length(V[i]);
//  V *= 1./(1.+t*t);
}

void Mesh::setSphere(uint fineness) {
//  setOctahedron();
//  setDodecahedron();
//  setTetrahedron();
  setIcosahedron();
  for(uint k=0; k<fineness; k++) {
    subDivide();
    for(uint i=0; i<V.d0; i++) V[i] /= length(V[i]);
  }
  fuseNearVertices(1e-6);
//  makeConvexHull();
}

void Mesh::setHalfSphere(uint fineness) {
  setOctahedron();
  V.resizeCopy(5, 3);
  T.resizeCopy(4, 3);
  for(uint k=0; k<fineness; k++) {
    subDivide();
    for(uint i=0; i<V.d0; i++) V[i] /= length(V[i]);
  }
  makeConvexHull();
}

void Mesh::setCylinder(double r, double l, uint fineness) {
  clear();
  uint div = 4 * (1 <<fineness);
  V.resize(2*div+2, 3);
  T.resize(4*div, 3);
  uint i, j;
  double phi;
  for(i=0; i<div; i++) {  //vertices
    phi=RAI_2PI*i/div;
    V(i, 0)=r*::cos(phi);
    V(i, 1)=r*::sin(phi);
    V(i, 2)=.5*l;
    V(i+div, 0)=V(i, 0);
    V(i+div, 1)=V(i, 1);
    V(i+div, 2)=-.5*l;
  }
  V(2*div+0, 0)=V(2*div+0, 1)=.0;  V(2*div+0, 2)=+.5*l; //upper center
  V(2*div+1, 0)=V(2*div+1, 1)=.0;  V(2*div+1, 2)=-.5*l; //lower center
  for(i=0; i<div; i++) {  //triangles
    j=(i+1)%div;
    T(4*i, 0)=i;
    T(4*i, 1)=j+div;
    T(4*i, 2)=j;

    T(4*i+2, 0)=i;
    T(4*i+2, 1)=j;
    T(4*i+2, 2)=2*div+0;

    T(4*i+1, 0)=i;
    T(4*i+1, 1)=i+div;
    T(4*i+1, 2)=j+div;

    T(4*i+3, 0)=j+div;
    T(4*i+3, 1)=i+div;
    T(4*i+3, 2)=2*div+1;
  }
}

void Mesh::setCone(double r, double h, uint fineness) {
  clear();
  uint div = 4 * (1 <<fineness);
  V.resize(div+2, 3).setZero();
  T.resize(2*div, 3);
  for(uint i=0; i<div; i++) {  //vertices
    double phi=RAI_2PI*i/div;
    V(i, 0)=r*::cos(phi);
    V(i, 1)=r*::sin(phi);
  }
  V(-2,2) = h;//upper center
  V(-1,2) = 0.;//lower center
  for(uint i=0; i<div; i++) {  //triangles
    uint j=(i+1)%div;
    T(2*i, 0)=i;
    T(2*i, 1)=V.d0-1;
    T(2*i, 2)=j;

    T(2*i+1, 0)=i;
    T(2*i+1, 1)=j;
    T(2*i+1, 2)=V.d0-2;
  }
}

void Mesh::setSSBox(double x_width, double y_width, double z_height, double r, uint fineness) {
  CHECK(r>=0. && x_width>=2.*r && y_width>=2.*r && z_height>=2.*r, "width/height includes radius!");
  arr size = {x_width, y_width, z_height};
  setSphere(fineness);
  //duplicate axis points
  for(uint j=0; j<3; j++) {
    for(uint i=0; i<V.d0; i++) {
      if(!V(i, j)) {
        V.append(V[i]);
        V(i, j) -= 1e-6;
        V(-1, j) += 1e-6;
      }
    }
  }

  scale(r);

  //push apart
  for(uint j=0; j<3; j++) {
    double del = .5*size(j)-r;
    for(uint i=0; i<V.d0; i++) {
      double& v = V(i, j);
      v += sign(v)*del;
    }
  }
  makeConvexHull();
}

void Mesh::setCapsule(double r, double l, uint fineness) {
  uint i;
  setSphere(fineness);
  scale(r);
  for(i=0; i<V.d0; i++) V(i, 2) += .5*sign(V(i, 2))*l;
  makeConvexHull();
}

/** @brief add triangles according to the given grid; grid has to be a 2D
  Array, the elements of which are indices referring to vertices in
  the vertex list (V) */
void Mesh::setGrid(uint X, uint Y) {
  CHECK(X>1 && Y>1, "grid has to be at least 2x2");
  CHECK_EQ(V.d0, X*Y, "don't have X*Y mesh-vertices to create grid faces");
  uint i, j, k=T.d0;
  T.resizeCopy(k+(Y-1)*2*(X-1), 3);
  for(j=0; j<Y-1; j++) {
    for(i=0; i<X-1; i++) {
      T(k, 0)=j*X+i; T(k, 1)=(j+1)*X+i; T(k, 2)=(j+1)*X+(i+1);
      k++;
      T(k, 0)=j*X+i; T(k, 1)=(j+1)*X+(i+1); T(k, 2)=j*X+(i+1);
      k++;
    }
  }
}

Mesh& Mesh::setRandom(uint vertices) {
  clear();
  V.resize(vertices, 3);
  rndUniform(V, -1., 1.);
//  rndGauss(V);
  Quaternion q;
  q.setRandom();
  q.applyOnPointArray(V);
  makeConvexHull();
  return *this;
}

void Mesh::subDivide() {
  uint v=V.d0, t=T.d0;
  V.resizeCopy(v+3*t, 3);
  uintA newT(4*t, 3);
  uint a, b, c, i, k, l;
  for(i=0, k=v, l=0; i<t; i++) {
    a=T(i, 0); b=T(i, 1); c=T(i, 2);
    V[k+0] = (double).5*(V[a] + V[b]);
    V[k+1] = (double).5*(V[b] + V[c]);
    V[k+2] = (double).5*(V[c] + V[a]);
    newT(l, 0)=a;   newT(l, 1)=k+0; newT(l, 2)=k+2; l++;
    newT(l, 0)=k+0; newT(l, 1)=b;   newT(l, 2)=k+1; l++;
    newT(l, 0)=k+0; newT(l, 1)=k+1; newT(l, 2)=k+2; l++;
    newT(l, 0)=k+2; newT(l, 1)=k+1; newT(l, 2)=c;   l++;
    k+=3;
  }
  CHECK_EQ(l, newT.d0, "");
  T = newT;
//  fuseNearVertices();
}

void Mesh::subDivide(uint i) {
  uint v=V.d0, t=T.d0;
  V.resizeCopy(v+3, 3);
  T.resizeCopy(t+3, 3);
  uint a, b, c;
  a=T(i, 0); b=T(i, 1); c=T(i, 2);
  V[v+0] = (double).5*(V[a] + V[b]);
  V[v+1] = (double).5*(V[b] + V[c]);
  V[v+2] = (double).5*(V[c] + V[a]);
  T(i, 0)=a;   T(i, 1)=v+0; T(i, 2)=v+2; //the old ith tri becomes one of the 4 new ones
  T(t, 0)=v+0; T(t, 1)=b;   T(t, 2)=v+1; t++;
  T(t, 0)=v+0; T(t, 1)=v+1; T(t, 2)=v+2; t++;
  T(t, 0)=v+2; T(t, 1)=v+1; T(t, 2)=c;   t++;
}

Mesh& Mesh::scale(double s) { V *= s;  return *this; }

Mesh& Mesh::scale(double sx, double sy, double sz) {
  uint i;
  for(i=0; i<V.d0; i++) {  V(i, 0)*=sx;  V(i, 1)*=sy;  V(i, 2)*=sz;  }
  return *this;
}

Mesh& Mesh::scale(const arr& s) {
  scale(s.elem(0), s.elem(1), s.elem(2));
  return *this;
}

void Mesh::translate(double dx, double dy, double dz) {
  uint i;
  for(i=0; i<V.d0; i++) {  V(i, 0)+=dx;  V(i, 1)+=dy;  V(i, 2)+=dz;  }
}

void Mesh::translate(const arr& d) {
  CHECK_EQ(d.N, 3, "");
  translate(d.elem(0), d.elem(1), d.elem(2));
}

void Mesh::transform(const Transformation& t) {
  t.applyOnPointArray(V);
}

Vector Mesh::center() {
  arr Vmean = mean(V);
  for(uint i=0; i<V.d0; i++) V[i] -= Vmean;
  return Vector(Vmean);
}

void Mesh::box() {
  double x, X, y, Y, z, Z, m;
  x=X=V(0, 0);
  y=Y=V(0, 1);
  z=Z=V(0, 2);
  for(uint i=0; i<V.d0; i++) {
    if(V(i, 0)<x) x=V(i, 0);
    if(V(i, 0)>X) X=V(i, 0);
    if(V(i, 1)<y) y=V(i, 1);
    if(V(i, 1)>Y) Y=V(i, 1);
    if(V(i, 2)<z) z=V(i, 2);
    if(V(i, 2)>Z) Z=V(i, 2);
  }
  translate(-.5*(x+X), -.5*(y+Y), -.5*(z+Z));
  m=X-x;
  if(Y-y>m) m=Y-y;
  if(Z-z>m) m=Z-z;
  scale(1./m);
}

void Mesh::addMesh(const Mesh& mesh2, const Transformation& X) {
  uint n=V.d0, t=T.d0;
  if(!V.N){
    C = mesh2.C;
  }else{
    if(!C.N && !mesh2.C.N){ //no color
      //do nothing
    }else if(C.nd==1 && mesh2.C==C){ //exact same color
      //do nothing
    }else{
      C = reshapeColor(C,V.d0);
      C.append(reshapeColor(mesh2.C, mesh2.V.d0));
      CHECK(C.nd==2 && C.d0==V.d0+mesh2.V.d0, "colors misshaped")
    }
  }
  // if(V.d0==C.d0 && (C.N || mesh2.C.N)) {
  //   if(mesh2.V.d0==mesh2.C.d0) C.append(mesh2.C);
  //   else if(mesh2.C.N==3) C.append(replicate(mesh2.C, mesh2.V.d0));
  //   else if(mesh2.C.N==4) C.append(replicate(mesh2.C({0, 2+1}), mesh2.V.d0));
  //   else if(!mesh2.C.N) C.append(replicate(arr{.8, .8, .8}, mesh2.V.d0));
  // } else {
  //   if(C.nd==2) C.clear();
  // }
  V.append(mesh2.V);
  T.append(mesh2.T);
  for(; t<T.d0; t++) {  T(t, 0)+=n;  T(t, 1)+=n;  T(t, 2)+=n;  }

  if(mesh2.texCoords.N) {
    texCoords.append(mesh2.texCoords);
  }
  if(mesh2._texImg) {
//    CHECK(!texImg.N, "can't append texture images");
    _texImg = mesh2._texImg;
  }
  if(!X.isZero()) {
    X.applyOnPointArray(V({n, -1+1}).noconst());
  }
}

void Mesh::addConvex(const arr& points, const arr& color) {
  Mesh sub;
  sub.V = getHull(points, sub.T);
  if(!!color) sub.C = color;
  cvxParts.append(V.d0);
  addMesh(sub);
}

void Mesh::makeConvexHull() {
  if(V.d0<=1) return;
#if 1
  V = getHull(V, T);
  cvxParts.clear();
  if(C.nd==2) C.clear();
  Vn.clear();
  Tn.clear();
  texCoords.clear();
  _texImg.reset();
  isArrayFormatted=false;
#else
  uintA H = getHullIndices(V, T);
  intA Hinv = consts<int>(-1, V.d0);
  for(uint i=0; i<H.N; i++) Hinv(H(i)) = i;

//  if(C.N==V.N){
//    arr Cnew(H.N, 3);
//    for(uint i=0;i<H.N;i++) Cnew[i] = C[H.elem(i)];
//    C=Cnew;
//  }

  arr Vnew(H.N, 3);
  for(uint i=0; i<H.N; i++) Vnew[i] = V[H.elem(i)];
  V=Vnew;

  for(uint i=0; i<T.d0; i++) {
    T(i, 0) = Hinv(T(i, 0));
    T(i, 1) = Hinv(T(i, 1));
    T(i, 2) = Hinv(T(i, 2));
  }
#endif
}

void Mesh::makeTriangleFan() {
  T.clear();
  for(uint i=1; i+1<V.d0; i++) {
    T.append(uintA{0, i, i+1});
    T.append(uintA{0, i+1, i});
  }
  T.reshape(T.N/3, 3);
}

void Mesh::makeLines() {
  CHECK(!(V.d0&1), "must have even number of lines vertices")
  T.resize(V.d0/2, 2);
  for(uint i=0; i<T.d0; i++) {
    T[i] = {2*i, 2*i+1};
  }
  isArrayFormatted=true;
}

void Mesh::makeLinesArrayFormatted() {
  CHECK_EQ(T.d1, 2, "");
  arr v(2*T.d0, 3);
  for(uint i=0;i<T.d0;i++){
    v[2*i] = V[T(i,0)];  T(i,0)=2*i;
    v[2*i+1] = V[T(i,1)];  T(i,1)=2*i+1;
  }
  V = v;
  isArrayFormatted=true;
}

void Mesh::makeArrayFormatted(double avgNormalsThreshold){
  if(isArrayFormatted){
    LOG(0) <<"is already array formatted";
    return;
  }
  computeTriNormals();
  if(C.N) C = reshapeColor(C, V.d0);
  arr vertices(T.d0*3, 3);
  arr colors  (T.d0*3, 4);
  arr normals (T.d0*3, 3);
  arr _texUV   (T.d0*3, 2);
  for(uint i=0;i<T.d0;i++){
    for(uint j=0;j<3;j++){
      uint v = T(i,j);
      uint l = (3*i+j);
      for(uint k=0;k<3;k++) vertices.p[3*l + k] = V.p[3*v + k]; //vertices(l, k) = V(T(i,j), k);
      if(C.N) for(uint k=0;k<4;k++) colors.p[4*l + k] = C.p[4*v + k];
      for(uint k=0;k<3;k++) normals.p[3*l + k] = Tn.p[3*i + k];
      if(texCoords.N) for(uint k=0;k<2;k++) _texUV.p[2*l + k] = texCoords.p[2*v + k]; //texUV(l,k) = tex(T(i,j), k);
    }
  }

  //-- normal smoothing
  if(avgNormalsThreshold<1.){
    //go through all vertices and check whether to average normals for smoothing
    uintAA sameVertices(V.d0);
    for(uint i=0;i<T.d0;i++){
      for(uint j=0;j<3;j++) sameVertices(T(i,j)).append(3*i+j);
    }

    for(uint i=0;i<sameVertices.N;i++){
      uintA& verts = sameVertices(i);
      uint n = verts.N;
      if(n<2) continue;
      //copy to arr
      arr ns(n, 3);
      for(uint j=0;j<n;j++) ns[j] = rai::convert<double>(normals[verts(j)]);
      //check if we can average all of them (all pairwise scalar products > threshold)
      bool avg = true;
      for(uint j=0;j<n;j++) for(uint k=j+1;k<n;k++){
        if(scalarProduct(ns[j], ns[k])<avgNormalsThreshold){ avg=false; break; }
      }
      if(avg){
        arr na = ::sum(ns, 0);
        na /= length(na);
        for(uint j=0;j<n;j++) normals[verts(j)] = na;
      }else{
        //average them pair-wise
        for(uint j=0;j<n;j++) for(uint k=0;k<n;k++){
          arr nj=ns[j], nk=ns[k];
          if(scalarProduct(nj, nk)>avgNormalsThreshold){
            arr na = nj+nk;
            na /= length(na);
            nj = na;
            nk = na;
          }
        }
        for(uint j=0;j<n;j++) normals[verts(j)] = ns[j];
      }
    }
  }

  V = vertices;
  Vn = normals;
  if(C.N) C = colors;
  T.setStraightPerm(V.d0);
  T.reshape(-1, 3);
  Tn.clear();
  if(texCoords.N) texCoords = _texUV;
  isArrayFormatted=true;
}

Mesh Mesh::decompose() {
  Mesh M;
#ifdef RAI_VHACD
  VHACD::IVHACD::Parameters p;
  VHACD::IVHACD* iface = VHACD::CreateVHACD();
  iface->Compute(V.p, V.d0, T.p, T.d0, p);

  Mesh c;
  VHACD::IVHACD::ConvexHull ch;
  for(uint i=0; i<iface->GetNConvexHulls(); i++) {
    iface->GetConvexHull(i, ch);
    c.clear();
#if 1 //new version
    c.V.referTo((double*)ch.m_points.data(), 3*ch.m_points.size()).reshape(-1, 3);
    c.T.referTo((uint32_t*)ch.m_triangles.data(), 3*ch.m_triangles.size()).reshape(-1, 3);
#else
    c.V.referTo(ch.m_points, 3*ch.m_nPoints).reshape(-1, 3);
    c.T.referTo(ch.m_triangles, 3*ch.m_nTriangles).reshape(-1, 3);
#endif
    c.C = id2color(i);
    M.cvxParts.append(M.V.d0);
    M.addMesh(c);
  }
  iface->Release();
#else
  NICO
#endif
  return M;
}

// uint Mesh::getComponents() {
//   //usually we'd analyze connected components... here assume sorted vertices and triangles!
//   return 0;
// //  uint part=0;
// //  uint partStart=0;
// //  uint end=0;
// //  cvxParts = {0};
// //  for(uint i=0;i<T.d0;i++){
// //    uint* t = &T(i,0);
// //    if(t[0]>end && t[1]>end && t[2]>end){ //NEW COMPONENT!
// //      part++;
// //      cvxParts.append(end);
// //      partStart = end+1;
// //    }
// //    CHECK_GE(t[0], partStart, "");
// //    CHECK_GE(t[1], partStart, "");
// //    CHECK_GE(t[2], partStart, "");
// //    if(t[0]>end) end=t[0];
// //    if(t[1]>end) end=t[1];
// //    if(t[2]>end) end=t[2];
// //  }
// //  LOG(0) <<"parts:" <<cvxParts;
//   return cvxParts.N;
// }

void Mesh::setSSCvx(const arr& core, double r, uint fineness) {
  if(r>0.) {
    Mesh ball;
    ball.setSphere(fineness);
    ball.scale(r);

    arr c=C;
    clear();
#if 1
    V = MinkowskiSum(core, ball.V);
#else
    for(uint i=0; i<core.d0; i++) {
      ball.translate(core(i, 0), core(i, 1), core(i, 2));
      addMesh(ball);
      ball.translate(-core(i, 0), -core(i, 1), -core(i, 2));
    }
#endif
    makeConvexHull();
    C=c;
  } else {
    arr c=C;
    V = core;
    makeConvexHull();
    C=c;
  }
}

/** @brief calculate the normals of all triangles (Tn) and the average
  normals of the vertices (N); average normals are averaged over
  all adjacent triangles that are in the triangle list or member of
  a strip */
void Mesh::computeTriNormals() {
  CHECK(T.N, "can't compute normals for a point cloud");
//  CHECK(!isArrayFormatted, "not implemented");
  Vector a, b, c;
  Tn.resize(T.d0, 3).setZero();
  for(uint i=0; i<T.d0; i++) {
    uint* t=T.p+3*i;
    a.set(V.p+3*t[0]);
    b.set(V.p+3*t[1]);
    c.set(V.p+3*t[2]);

    b-=a; c-=a; a=b^c; if(!a.isZero) a.normalize();
    Tn(i, 0)=a.x;  Tn(i, 1)=a.y;  Tn(i, 2)=a.z;
  }
}

void Mesh::computeFaceColors() {
  CHECK_EQ(C.nd, 2, "");
  CHECK_EQ(C.d0, V.d0, "");
  CHECK_EQ(C.d1, 3, "");
  arr CV = C;
  C.resize(T.d0, 3);
  for(uint i=0; i<T.d0; i++) {
    uint* t=T.p+3*i;
    double* a = CV.p+3*t[0];
    double* b = CV.p+3*t[1];
    double* c = CV.p+3*t[2];
    double* d = C.p+3*i;
    d[0] = (a[0]+b[0]+c[0])/3.;
    d[1] = (a[1]+b[1]+c[1])/3.;
    d[2] = (a[2]+b[2]+c[2])/3.;
  }
}

arr Mesh::computeTriDistances() {
  if(!Tn.N) computeTriNormals();
  arr d(T.d0);
  Vector n, a, b, c;
  for(uint i=0; i<T.d0; i++) {
    uint* t=T.p+3*i;
    a.set(V.p+3*t[0]);
    b.set(V.p+3*t[1]);
    c.set(V.p+3*t[2]);
    n.set(Tn.p+3*i);

    d(i) = a*n;
  }
  return d;
}

/** @brief add triangles according to the given grid; grid has to be a 2D
  Array, the elements of which are indices referring to vertices in
  the vertex list (V) */
/*void Mesh::gridToTriangles(const uintA &grid){
  uint i, j, k=T.d0;
  T.resizeCopy(T.d0+2*(grid.d0-1)*(grid.d1-1), 3);
  for(i=0;i<grid.d0-1;i++) for(j=0;j<grid.d1-1;j++){
    if((i+j)&1){
      T(k, 0)=grid(i+1, j  );
      T(k, 1)=grid(i  , j  );
      T(k, 2)=grid(i  , j+1);
      k++;
      T(k, 0)=grid(i+1, j  );
      T(k, 1)=grid(i  , j+1);
      T(k, 2)=grid(i+1, j+1);
      k++;
    }else{
      T(k, 0)=grid(i+1, j  );
      T(k, 1)=grid(i  , j  );
      T(k, 2)=grid(i+1, j+1);
      k++;
      T(k, 0)=grid(i+1, j+1);
      T(k, 1)=grid(i  , j  );
      T(k, 2)=grid(i  , j+1);
      k++;
    }
  }
}*/

/** @brief add strips according to the given grid (sliced in strips along
  the x-axis (the first index)); grid has to be a 2D Array, the
  elements of which are indices referring to vertices in the vertex
  list (V) */
/*void Mesh::gridToStrips(const uintA& grid){
  CHECK(grid.d0>1 && grid.d1>1, "grid has to be at least 2x2");
  uint i, j, k=strips.N, l;
  strips.resizeCopy(strips.N+grid.d0-1);
  for(i=0;i<grid.d0-1;i++){
    strips(k).resize(2*grid.d1);
    l=0;
    for(j=0;j<grid.d1;j++){
      strips(k)(l)=grid(i+1, j); l++;
      strips(k)(l)=grid(i  , j); l++;
    }
#if 0 //code to make it less symmetric
      //}else{
    strips(k)(l)=grid(i, 0); l++;
    for(j=0;j<grid.d1;j++){
      strips(k)(l)=grid(i  , j); l++;
      strips(k)(l)=grid(i+1, j); l++;
    }
#endif
    k++;
  }
}*/

/** @brief add strips according to the given grid (sliced in strips along
  the x-axis (the first index)); it is assumed that the vertices in
  the list V linearly correspond to points in the XxY grid */
/*void Mesh::gridToStrips(uint X, uint Y){
  CHECK(X>1 && Y>1, "grid has to be at least 2x2");
  uint i, j, k=strips.N, l;
  strips.resizeCopy(strips.N+Y-1);
  for(j=0;j<Y-1;j++){
    strips(k).resize(2*X);
    l=0;
    for(i=0;i<X;i++){
      strips(k)(l)=(j+1)*X+i;
      l++;
      strips(k)(l)=    j*X+i;
      l++;
    }
    k++;
  }
}*/

void deleteZeroTriangles(Mesh& m) {
  uintA newT;
  newT.resizeAs(m.T);
  uint i, j;
  for(i=0, j=0; i<m.T.d0; i++) {
    if(m.T(i, 0)!=m.T(i, 1) && m.T(i, 0)!=m.T(i, 2) && m.T(i, 1)!=m.T(i, 2))
      memmove(&newT(j++, 0), &m.T(i, 0), 3*newT.sizeT);
  }
  newT.resizeCopy(j, 3);
  m.T=newT;
}

void permuteVertices(Mesh& m, uintA& p) {
  CHECK_EQ(p.N, m.V.d0, "");
  uint i;
  arr x(p.N, 3);
  for(i=0; i<p.N; i++) { x(i, 0)=m.V(p(i), 0); x(i, 1)=m.V(p(i), 1); x(i, 2)=m.V(p(i), 2); }
  m.V=x;
  if(m.Vn.N) {
    for(i=0; i<p.N; i++) { x(i, 0)=m.Vn(p(i), 0); x(i, 1)=m.Vn(p(i), 1); x(i, 2)=m.Vn(p(i), 2); }
    m.Vn=x;
  }
  if(m.C.N==m.V.N) {
    for(i=0; i<p.N; i++) { x(i, 0)=m.C(p(i), 0); x(i, 1)=m.C(p(i), 1); x(i, 2)=m.C(p(i), 2); }
    m.C=x;
  }
  uintA y(m.T.d0, m.T.d1);
  uintA p2(p.N); //inverse permutation
  for(i=0; i<p.N; i++) p2(p(i))=i;
  for(i=0; i<m.T.N; i++) y.elem(i)=p2(m.T.elem(i));
  m.T=y;
}

/** @brief delete all void triangles (with vertex indices (0, 0, 0)) and void
  vertices (not used for triangles or strips) */
void Mesh::deleteUnusedVertices() {
  if(!V.N) return;
  uintA p;
  uintA u;
  uint i, Nused;

  deleteZeroTriangles(*this);

  //count vertex usage
  u.resize(V.d0);
  u.setZero();
  for(i=0; i<T.d0; i++) { u(T(i, 0))++; u(T(i, 1))++; u(T(i, 2))++; }
  //for(i=0;i<strips.N;i++) for(j=0;j<strips(i).N;j++) u(strips(i)(j))=true;

  //find proper permutation of vertex list
  p.setStraightPerm(V.d0);
  Nused=p.N;
  for(i=0; i<Nused; i++) if(!u(i)) { Nused--; p.permute(i, Nused); u.permute(i, Nused); i--; }

  permuteVertices(*this, p);
  V.resizeCopy(Nused, 3);
}

arr* COMP_V;
bool COMP(uint i, uint j) {
  bool r=(*COMP_V)[i]<(*COMP_V)[j];
  return r;
}

/** @brief delete all void triangles (with vertex indices (0, 0, 0)) and void
  vertices (not used for triangles or strips) */
void Mesh::fuseNearVertices(double tol) {
  if(!V.N) return;
  uintA p;
  uint i, j;

  if(C.N==V.N) C.clear();

//  cout <<"fusing vertices: #V=" <<V.d0 <<", sorting.." <<std::flush;
  //cout <<V <<endl;
  //sort vertices lexically
  p.setStraightPerm(V.d0);
  COMP_V=&V;
  uint* pstop=p.p+p.N;
  std::sort(p.p, pstop, COMP);
  permuteVertices(*this, p);

//  cout <<"permuting.." <<std::flush;
  //cout <<V <<endl;
  p.setStraightPerm(V.d0);
  for(i=0; i<V.d0; i++) {
    if(p(i)!=i) continue;  //i has already been fused with p(i), and p(i) has already been checked...
    for(j=i+1; j<V.d0; j++) {
      if(V(j, 0)-V(i, 0)>tol) break;
      if(sqr(V(j, 0)-V(i, 0))+sqr(V(j, 1)-V(i, 1))+sqr(V(j, 2)-V(i, 2))<tol*tol) {
        //cout <<"fusing " <<i <<" " <<j <<" " <<V[i] <<" " <<V[j] <<endl;
        p(j)=i;
      }
    }
  }

  uintA y(T.d0, 3);
  for(i=0; i<T.d0; i++) { y(i, 0)=p(T(i, 0)); y(i, 1)=p(T(i, 1)); y(i, 2)=p(T(i, 2)); }
  T=y;

//  cout <<"deleting tris.." <<std::flush;
  deleteZeroTriangles(*this);

//  cout <<"deleting verts.." <<std::flush;
  deleteUnusedVertices();

//  cout <<"#V=" <<V.d0 <<", done" <<endl;

  texCoords.clear();
  _texImg.reset();
}

void Mesh::deleteVertices(uintA& delLabels) {
  CHECK_EQ(delLabels.N, V.d0, "");
  uintA p;
  p.setStraightPerm(V.d0);
  uint N=p.N;
  for(uint i=0; i<N; i++) if(delLabels(i)) { N--; p.permute(i, N); delLabels.permute(i, N); i--; }

  permuteVertices(*this, p);
  if(C.d0==V.d0) C.resizeCopy(N, 3);
  V.resizeCopy(N, 3);

  //remove tris..
  for(uint i=T.d0; i--;) {
    bool del=false;
    for(uint j=0; j<T.d1; j++) if(T(i, j)>=V.d0) { del=true; break; }
    if(del) T.delRows(i);
  }
}

void getVertexNeighorsList(const Mesh& m, intA& Vt, intA& VT) {
  uint i, j;
  Vt.resize(m.V.d0);  Vt.setZero();
  VT.resize(m.V.d0, 100);
  for(i=0; i<m.T.d0; i++) {
    j=m.T(i, 0);  VT(j, Vt(j))=i;  Vt(j)++;
    j=m.T(i, 1);  VT(j, Vt(j))=i;  Vt(j)++;
    j=m.T(i, 2);  VT(j, Vt(j))=i;  Vt(j)++;
  }
}

void getTriNormals(const Mesh& m, arr& Tn) {
  uint i;
  Vector a, b, c;
  Tn.resize(m.T.d0, 3); //tri normals
  for(i=0; i<m.T.d0; i++) {
    a.set(&m.V(m.T(i, 0), 0)); b.set(&m.V(m.T(i, 1), 0)); c.set(&m.V(m.T(i, 2), 0));
    b-=a; c-=a; a=b^c; a.normalize();
    Tn(i, 0)=a.x;  Tn(i, 1)=a.y;  Tn(i, 2)=a.z;
  }
}

/// flips all faces
void Mesh::flipFaces() {
  uint i, a;
  for(i=0; i<T.d0; i++) {
    a=T(i, 0);
    T(i, 0)=T(i, 1);
    T(i, 1)=a;
  }
}

/// check whether this is really a closed mesh, and flip inconsistent faces
void Mesh::clean() {
  uint i, j, idist=0;
  Vector a, b, c, m;
  double mdist=0.;
  arr Tc(T.d0, 3); //tri centers
  arr Tn(T.d0, 3); //tri normals
  uintA Vt(V.d0);
  intA VT(V.d0, 100); //tri-neighbors to a vertex
  Vt.setZero(); VT=-1;

  for(i=0; i<T.d0; i++) {
    a.set(&V(T(i, 0), 0)); b.set(&V(T(i, 1), 0)); c.set(&V(T(i, 2), 0));

    //tri center
    m=(a+b+c)/3.;
    Tc(i, 0)=m.x;  Tc(i, 1)=m.y;  Tc(i, 2)=m.z;

    //farthest tri
    if(m.length()>mdist) { mdist=m.length(); idist=i; }

    //tri normal
    b-=a; c-=a; a=b^c; a.normalize();
    Tn(i, 0)=a.x;  Tn(i, 1)=a.y;  Tn(i, 2)=a.z;

    //vertex neighbor count
    j=T(i, 0);  VT(j, Vt(j))=i;  Vt(j)++;
    j=T(i, 1);  VT(j, Vt(j))=i;  Vt(j)++;
    j=T(i, 2);  VT(j, Vt(j))=i;  Vt(j)++;
  }

  //step through tri list and flip them if necessary
  boolA Tisok(T.d0); Tisok=false;
  uintA Tok; //contains the list of all tris that are ok oriented
  uintA Tnew(T.d0, T.d1);
  Tok.append(idist);
  Tisok(idist)=true;
  int A=0, B=0, D;
  uint r, k, l;
  intA neighbors;
  for(k=0; k<Tok.N; k++) {
    i=Tok(k);
    Tnew(k, 0)=T(i, 0); Tnew(k, 1)=T(i, 1); Tnew(k, 2)=T(i, 2);

    for(r=0; r<3; r++) {
      if(r==0) { A=T(i, 0);  B=T(i, 1);  /*C=T(i, 2);*/ }
      if(r==1) { A=T(i, 1);  B=T(i, 2);  /*C=T(i, 0);*/ }
      if(r==2) { A=T(i, 2);  B=T(i, 0);  /*C=T(i, 1);*/ }

      //check all triangles that share A & B
      neighbors = setSection(VT[A], VT[B]);
      neighbors.removeAllValues(-1);
      if(neighbors.N>2) RAI_MSG("edge shared by more than 2 triangles " <<neighbors);
      neighbors.removeValue(i);
      //if(!neighbors.N) cout <<"mesh.clean warning: edge has only one triangle that shares it" <<endl;

      //orient them correctly
      for(l=0; l<neighbors.N; l++) {
        j=neighbors(l); //j is a neighboring triangle sharing A & B
        D=-1;
        //align the neighboring triangle and let D be its 3rd vertex
        if((int)T(j, 0)==A && (int)T(j, 1)==B) D=T(j, 2);
        if((int)T(j, 0)==A && (int)T(j, 2)==B) D=T(j, 1);
        if((int)T(j, 1)==A && (int)T(j, 2)==B) D=T(j, 0);
        if((int)T(j, 1)==A && (int)T(j, 0)==B) D=T(j, 2);
        if((int)T(j, 2)==A && (int)T(j, 0)==B) D=T(j, 1);
        if((int)T(j, 2)==A && (int)T(j, 1)==B) D=T(j, 0);
        if(D==-1) HALT("dammit");
        //determine orientation
        if(!Tisok(j)) {
          T(j, 0)=B;  T(j, 1)=A;  T(j, 2)=D;
          Tok.append(j);
          Tisok(j)=true;
        } else {
          //check if consistent!
        }
      }

#if 0
      //compute their rotation
      if(neighbors.N>1) {
        double phi, phimax;
        int jmax=-1;
        Vector ni, nj;
        for(l=0; l<neighbors.N; l++) {
          j=neighbors(l); //j is a neighboring triangle sharing A & B

          a.set(&V(T(i, 0), 0)); b.set(&V(T(i, 1), 0)); c.set(&V(T(i, 2), 0));
          b-=a; c-=a; a=b^c; a.normalize();
          ni = a;

          a.set(&V(T(j, 0), 0)); b.set(&V(T(j, 1), 0)); c.set(&V(T(j, 2), 0));
          b-=a; c-=a; a=b^c; a.normalize();
          nj = a;

          Quaternion q;
          q.setDiff(ni, -nj);
          q.getDeg(phi, c);
          a.set(&V(A, 0)); b.set(&V(B, 0));
          if(c*(a-b) < 0.) phi+=180.;

          if(jmax==-1 || phi>phimax) { jmax=j; phimax=phi; }
        }
        if(!Tisok(jmax)) {
          Tok.append(jmax);
          Tisok(jmax)=true;
        }
      } else {
        j = neighbors(0);
        if(!Tisok(j)) {
          Tok.append(j);
          Tisok(j)=true;
        }
      }
#endif
    }
  }
  if(k<T.d0) {
    cout <<"mesh.clean warning: not all triangles connected: " <<k <<"<" <<T.d0 <<endl;
    cout <<"WARNING: cutting of all non-connected triangles!!" <<endl;
    Tnew.resizeCopy(k, 3);
    T=Tnew;
    deleteUnusedVertices();
  }

  texCoords.clear();
  _texImg.reset();
//  computeNormals();
}

void getEdgeNeighborsList(const Mesh& m, uintA& EV, uintA& Et, intA& ET) {
  intA Vt, VT;
  getVertexNeighorsList(m, Vt, VT);

  uint A=0, B=0, t, tt, i, r, k;
  //build edge list
  EV.resize(m.T.d0*3, 2);   EV=0;     //edge vert neighbors
  ET.resize(m.T.d0*3, 10);  ET=-1;    //edge tri neighbors
  Et.resize(m.T.d0*3); Et.setZero(); //#edge tri neighbors
  boolA done(m.T.d0); done=false;
  for(t=0, k=0; t<m.T.d0; t++) {
    for(r=0; r<3; r++) {
      if(r==0) { A=m.T(t, 0);  B=m.T(t, 1);  }
      if(r==1) { A=m.T(t, 1);  B=m.T(t, 2);  }
      if(r==2) { A=m.T(t, 2);  B=m.T(t, 0);  }

      //has AB already been taken care of?
      bool yes=false;
      for(i=0; i<(uint)Vt(A); i++) {
        tt=VT(A, i);
        if(m.T(tt, 0)==B || m.T(tt, 1)==B || m.T(tt, 2)==B) {
          if(done(tt)) yes=true;
        }
      }
      if(yes) continue;

      //if not, then do it
      EV(k, 0)=A;
      EV(k, 1)=B;
      for(i=0; i<(uint)Vt(A); i++) {
        tt=VT(A, i);
        if(m.T(tt, 0)==B || m.T(tt, 1)==B || m.T(tt, 2)==B) {
          ET(k, Et(k))=tt;
          Et(k)++;
        }
      }
      k++;
    }
    done(t)=true;
  }

  EV.resizeCopy(k, 2);
  ET.resizeCopy(k, 10);
  Et.resizeCopy(k);

  cout <<"\n#edges=" <<k
       <<"\nedge=\n" <<EV
       <<"\n@neighs=\n" <<Et
       <<"\nneighs=\n" <<ET <<endl;
}

void getTriNeighborsList(const Mesh& m, uintA& Tt, intA& TT) {
  intA Vt, VT;
  getVertexNeighorsList(m, Vt, VT);

  uint A=0, B=0, t, tt, r, i;
  Tt.resize(m.T.d0, 3);     Tt.setZero();
  TT.resize(m.T.d0, 3, 100); TT=-1;
  for(t=0; t<m.T.d0; t++) {
    for(r=0; r<3; r++) {
      if(r==0) { A=m.T(t, 0);  B=m.T(t, 1);  }
      if(r==1) { A=m.T(t, 1);  B=m.T(t, 2);  }
      if(r==2) { A=m.T(t, 2);  B=m.T(t, 0);  }

      for(i=0; i<(uint)Vt(A); i++) {
        tt=VT(A, i);
        if(tt!=t && (m.T(tt, 0)==B || m.T(tt, 1)==B || m.T(tt, 2)==B)) {
          TT(t, r, Tt(t, r))=tt;
          Tt(t, r)++;
        }
      }
    }
  }

  //cout <<Tt <<TT <<endl;
}

void Mesh::skin(uint start) {
  intA TT;
  uintA Tt;
  getTriNeighborsList(*this, Tt, TT);
  arr Tn;
  getTriNormals(*this, Tn);

  uintA goodTris;
  boolA added(T.d0);
  goodTris.append(start);
  added=false;
  added(start)=true;
  uint t, tt, r, i, k;
  int m;
  double p, mp=0;
  for(k=0; k<goodTris.N; k++) {
    t=goodTris(k);
    for(r=0; r<3; r++) {
      //select from all neighbors the one most parallel
      m=-1;
      for(i=0; i<Tt(t, r); i++) {
        tt=TT(t, r, i);
        p=scalarProduct(Tn[t], Tn[tt]);
        if(m==-1 || p>mp) { m=tt; mp=p; }
      }
      if(m!=-1 && !added(m)) { goodTris.append(m); added(m)=true; }
    }
  }

  uintA Tnew(k, 3);
  for(k=0; k<goodTris.N; k++) {
    t=goodTris(k);
    Tnew(k, 0)=T(t, 0); Tnew(k, 1)=T(t, 1); Tnew(k, 2)=T(t, 2);
  }
  T=Tnew;
  cout <<T <<endl;
}

arr Mesh::getMean() const {
  return mean(V);
}

Vector Mesh::getCenter() const {
  return Vector(getMean());
}

void Mesh::getBox(double& dx, double& dy, double& dz) const {
  dx=dy=dz=0.;
  for(uint i=0; i<V.d0; i++) {
    dx=MAX(dx, fabs(V(i, 0)));
    dy=MAX(dy, fabs(V(i, 1)));
    dz=MAX(dz, fabs(V(i, 2)));
  }
}

arr Mesh::getBounds() const {
  arr a, b;
  a = b = V[0];
  for(uint i=0; i<V.d0; i++) {
    a = elemWiseMin(a, V[i]);
    b = elemWiseMax(b, V[i]);
  }
  return (a, b).reshape(2, 3);
}

double Mesh::getRadius() const {
  double r=0.;
  for(uint i=0; i<V.d0; i++) r=MAX(r, sumOfSqr(V[i]));
  return sqrt(r);
}

double triArea(const arr& a, const arr& b, const arr& c) {
  return .5*length(crossProduct(b-a, c-a));
}

double Mesh::getArea() const {
  CHECK_EQ(T.d1, 3, "");
  double A=0.;
  for(uint i=0; i<T.d0; i++) A += getArea(i);
  return A;
}

double Mesh::getArea(uint i) const {
  CHECK_EQ(T.d1, 3, "");
  Vector a, b, c;
  a.set(V.p+3*T.p[3*i+0]);
  b.set(V.p+3*T.p[3*i+1]);
  c.set(V.p+3*T.p[3*i+2]);
  return .5*((b-a)^(c-a)).length();
}

double Mesh::getVolume() const {
  CHECK_EQ(T.d1, 3, "");
  Vector z = getMean();
  Vector a, b, c;
  double vol=0.;
  for(uint i=0; i<T.d0; i++) {
    a.set(V.p+3*T.p[3*i+0]);
    b.set(V.p+3*T.p[3*i+1]);
    c.set(V.p+3*T.p[3*i+2]);
    vol += (a-z) * ((b-a)^(c-a));
  }
  return vol/6.;
}

uintA Mesh::getVertexDegrees() const {
  uintA deg(V.d0);
  deg.setZero();
  for(uint v:T) deg(v)++;
  return deg;
}

void Mesh::samplePoints(arr& pts, arr& normals, uint n){
  //-- random triangle selections
  arr A(T.d0);
  for(uint i=0; i<T.d0; i++) A(i) = getArea(i);
  A = integral(A);
  A *= double(n)/A(-1);
  double off=rnd.uni();
  uintA sel(n);
  for(uint i=0,t=0; i<n; i++){
    while(A(t)-off<i) t++;
    sel(i)=t;
  }
  //-- random bc coordinates
  arr T = sqrt(rand({n}));
  arr S = rand({n});
  arr B(3,n);
  B[0] = 1.-T;
  B[1] = (1.-S) % T;
  B[2] = S % T;
  B = (~B).reshape(n*3);
  //-- apply bc to selected vertices and normals
  CHECK(isArrayFormatted, "");
  V.reshape(V.d0/3, 3, 3);
  Vn.reshape(Vn.d0/3, 3, 3);
  arr Vsel = B % V.pick(sel).reshape(n*3,3);
  arr Nsel = B % Vn.pick(sel).reshape(n*3,3);
  pts = ::sum(Vsel.reshape(n,3,3), 1u);
  normals = ::sum(Nsel.reshape(n,3,3), 1u);
  V.reshape(V.d0*3, 3);
  Vn.reshape(Vn.d0*3, 3);
}

ANN& Mesh::ensure_ann() {
  if(!ann) ann = make_shared<ANN>();
  if(ann->X.d0 != V.d0) ann->setX(V);
  return *ann;
}

double Mesh::meshMetric(const Mesh& trueMesh, const Mesh& estimatedMesh) {
  //basically a Haussdorf metric, stupidly realized by brute force algorithm
  auto haussdorfDistanceOneSide = [](const arr& V1, const arr& V2)->double {
    double distance = 0.0;
    for(uint i = 0; i < V1.d0; i++) {
      double shortestDistance = std::numeric_limits<double>::infinity();
      for(uint j = 0; j < V2.d0; j++) {
        double d = length(V2[j]-V1[i]);
        if(d < shortestDistance) {
          shortestDistance = d;
        }
      }
      if(shortestDistance > distance) {
        distance = shortestDistance;
      }
    }
    return distance;
  };

  return MAX(haussdorfDistanceOneSide(trueMesh.V, estimatedMesh.V), haussdorfDistanceOneSide(estimatedMesh.V, trueMesh.V));
}

double Mesh::getCircum() const {
  if(!T.N) return 0.;
  CHECK_EQ(T.d1, 2, "");
  double A=0.;
  for(uint i=0; i<T.d0; i++) A += length(V[T(i, 0)] - V[T(i, 1)]);
  return A;
}

double Mesh::getCircum(uint i) const {
  if(!T.N) return 0.;
  CHECK_EQ(T.d1, 3, "");
  double A=0.;
  A += length(V[T(i, 0)] - V[T(i, 1)]);
  A += length(V[T(i, 1)] - V[T(i, 2)]);
  A += length(V[T(i, 2)] - V[T(i, 0)]);
  return A;
}

void Mesh::write(std::ostream& os) const {
  os <<"Mesh: " <<V.d0 <<" vertices, " <<T.d0 <<" triangles" <<endl;
}

Mesh& Mesh::readFile(const char* filename) {
  const char* fileExtension = filename+(strlen(filename)-3);
  read(FILE(filename).getIs(), fileExtension, filename);
  return *this;
}

void Mesh::read(std::istream& is, const char* fileExtension, const char* filename) {
  if(!strcmp(fileExtension, "arr")) { readArr(is); }
  else if(!strcmp(fileExtension, "omp")) { readArr(is); } //decomp
  else if(!strcmp(fileExtension, "esh")) { readArr(is); } //mesh
  else if(!strcmp(fileExtension, "nts")) { readPts(is); } //points
  else if(!strcmp(fileExtension, "pts")) { readPts(is); }
  else if(!strcmp(fileExtension, "msh")) { readJson(is); }
  else if(!strcmp(fileExtension, ".h5")) { readH5(filename, "mesh"); }
  else if(!strcmp(fileExtension, "off")) { readOffFile(is); }
  else if(!strcmp(fileExtension, "ply")) { readPLY(filename); }
  else if(!strcmp(fileExtension, "tri")) { readTriFile(is); }
  //  else if(!strcmp(fileExtension, "stl") || !strcmp(fileExtension, "STL")) { readStlFile(is); }
  else{
    bool flipYZ = false;
    if(!strcmp(fileExtension, "dae")) flipYZ = rai::getParameter("assimp/daeFlipYZ", true);
    *this = AssimpLoader(filename, flipYZ, false, 0).getSingleMesh();
  }
}

void Mesh::writeTriFile(const char* filename) {
  ofstream os;
  open(os, filename);
  os <<"TRI" <<endl <<endl
     <<V.d0 <<endl
     <<T.d0 <<endl <<endl;

  V.write(os, " ", "\n ", "  ");
  os <<endl <<endl;
  T.write(os, " ", "\n ", "  ");
}

void Mesh::readTriFile(std::istream& is) {
  uint i, nV, nT;
  is >>PARSE("TRI") >>nV >>nT;
  V.resize(nV, 3);
  T.resize(nT, 3);
  for(i=0; i<V.N; i++) is >>V.elem(i);
  for(i=0; i<T.N; i++) is >>T.elem(i);
}

void Mesh::writeOffFile(const char* filename) {
  ofstream os;
  open(os, filename);
  uint i;
  os <<"OFF\n" <<V.d0 <<' ' <<T.d0 <<' ' <<0 <<endl;
  for(i=0; i<V.d0; i++) os <<V(i, 0) <<' ' <<V(i, 1) <<' ' <<V(i, 2) <<endl;
  for(i=0; i<T.d0; i++) os <<3 <<' ' <<T(i, 0) <<' ' <<T(i, 1) <<' ' <<T(i, 2) <<endl;
}

void Mesh::readOffFile(std::istream& is) {
  uint i, k, nVertices, nFaces, nEdges, alpha;
  bool color;
  String tag;
  is >>tag;
  if(tag=="OFF") color=false;
  else if(tag=="COFF") color=true;
  else HALT("");
  is >>nVertices >>nFaces >>nEdges;
  CHECK(!nEdges, "can't read edges in off file");
  V.resize(nVertices, 3);
  T.resize(nFaces, 3);
  if(color) C.resize(nVertices, 3);
  for(i=0; i<V.d0; i++) {
    is >>V(i, 0) >>V(i, 1) >>V(i, 2);
    if(color) is >>C(i, 0) >>C(i, 1) >>C(i, 2) >>alpha;
  }
  for(i=0; i<T.d0; i++) {
    is >>k;
    CHECK_EQ(k, 3, "can only read triangles from OFF");
    is >>T(i, 0) >>T(i, 1) >>T(i, 2);
  }
}

void Mesh::readPlyFile(std::istream& is) {
  uint i, k, nVertices, nFaces;
  String str;
  is >>PARSE("ply") >>PARSE("format") >>str;
  if(str=="ascii") {
    is >>PARSE("1.0");
    is >>PARSE("element vertex") >>nVertices;
    is >>PARSE("property float32 x") >>PARSE("property float32 y") >>PARSE("property float32 z");
    is >>PARSE("property float32 nx") >>PARSE("property float32 ny") >>PARSE("property float32 nz");
    is >>PARSE("element face") >>nFaces;
    is >>PARSE("property list uint8 int32 vertex_indices") >>PARSE("end_header");
    V.resize(nVertices, 3);
    T.resize(nFaces, 3);
    double nx, ny, nz;
    for(i=0; i<V.d0; i++) {
      is >>V(i, 0) >>V(i, 1) >>V(i, 2) >>nx >>ny >>nz;
    }
    for(i=0; i<T.d0; i++) {
      is >>k >>T(i, 0) >>T(i, 1) >>T(i, 2);
      CHECK_EQ(k, 3, "can only read triangles from ply");
    }
  }
}

#if 1 //def RAI_PLY
void Mesh::writePLY(const char* fn, bool bin) {
  struct Face { unsigned char nverts;  int* verts; };
  struct Vertex { float x, y, z;  byte r, g, b;  };

  PlyProperty vert_props[]  = { /* list of property information for a PlyVertex */
    {"x", Float32, Float32, offsetof(Vertex, x), 0, 0, 0, 0},
    {"y", Float32, Float32, offsetof(Vertex, y), 0, 0, 0, 0},
    {"z", Float32, Float32, offsetof(Vertex, z), 0, 0, 0, 0},
    {"red", Uint8, Uint8, offsetof(Vertex, r), 0, 0, 0, 0},
    {"green", Uint8, Uint8, offsetof(Vertex, g), 0, 0, 0, 0},
    {"blue", Uint8, Uint8, offsetof(Vertex, b), 0, 0, 0, 0}
//    {"nx", Float64, Float64, offsetof( Vertex,nx ), 0, 0, 0, 0},
//    {"ny", Float64, Float64, offsetof( Vertex,ny ), 0, 0, 0, 0},
//    {"nz", Float64, Float64, offsetof( Vertex,nz ), 0, 0, 0, 0}
  };

  PlyProperty face_props[1]; /* list of property information for a PlyFace */
  if(V.d0<65535) {
    face_props[0] = {"vertex_indices", Int16, Int32, offsetof(Face, verts), 1, Uint8, Uint8, offsetof(Face, nverts)};
  } else {
    face_props[0] = {"vertex_indices", Int32, Int32, offsetof(Face, verts), 1, Uint8, Uint8, offsetof(Face, nverts)};
  }

  PlyFile*    ply;
  FILE*       fp = fopen(fn, "w");

  const char*  elem_names[]  = { "vertex", "face" };
  ply = write_ply(fp, 2, elem_names, bin? PLY_BINARY_LE : PLY_ASCII);

  /* describe what properties go into the PlyVertex elements */
  describe_element_ply(ply, "vertex", V.d0);
  describe_property_ply(ply, &vert_props[0]);
  describe_property_ply(ply, &vert_props[1]);
  describe_property_ply(ply, &vert_props[2]);
  if(C.N==V.N) {
    describe_property_ply(ply, &vert_props[3]);
    describe_property_ply(ply, &vert_props[4]);
    describe_property_ply(ply, &vert_props[5]);
  }

  /* describe PlyFace properties (just list of PlyVertex indices) */
  describe_element_ply(ply, "face", T.d0);
  describe_property_ply(ply, &face_props[0]);

  header_complete_ply(ply);

  //-- put vertices
  put_element_setup_ply(ply, "vertex");
  Vertex vertex;
  for(uint i = 0; i < V.d0 ; i++) {
    vertex.x = V(i, 0);
    vertex.y = V(i, 1);
    vertex.z = V(i, 2);
    if(C.N==V.N) {
      vertex.r = 255.*C(i, 0);
      vertex.g = 255.*C(i, 1);
      vertex.b = 255.*C(i, 2);
    }
    put_element_ply(ply, (void*) &vertex);
  }

  //-- put tris
  put_element_setup_ply(ply, "face");
  int verts[3];
  Face face;
  face.nverts = 3;
  face.verts = verts;
  for(uint i = 0; i < T.d0; i++) {
    face.verts[0] = T(i, 0);
    face.verts[1] = T(i, 1);
    face.verts[2] = T(i, 2);
    put_element_ply(ply, (void*) &face);
  }

  close_ply(ply); //calls fclose
  free_ply(ply);
}

bool ply_check_property(PlyElement* elem, const char* prop_name) {
  for(int i=0; i<elem->nprops; i++)
    if(!strcmp(prop_name, elem->props[i]->name)) return true;
  return false;
}

void Mesh::readPLY(const char* fn) {
  struct PlyFace {    unsigned char nverts;  int* verts; };
  struct Vertex {    double x,  y,  z ;  byte r, g, b; };
  uint _nverts=0, _ntrigs=0;

  PlyProperty vert_props[]  = { /* list of property information for a PlyVertex */
    {"x", Float64, Float64, offsetof(Vertex, x), 0, 0, 0, 0},
    {"y", Float64, Float64, offsetof(Vertex, y), 0, 0, 0, 0},
    {"z", Float64, Float64, offsetof(Vertex, z), 0, 0, 0, 0},
    //    {"nx", Float64, Float64, offsetof( Vertex,nx ), 0, 0, 0, 0},
    //    {"ny", Float64, Float64, offsetof( Vertex,ny ), 0, 0, 0, 0},
    //    {"nz", Float64, Float64, offsetof( Vertex,nz ), 0, 0, 0, 0}
    {"red", Uint8, Uint8, offsetof(Vertex, r), 0, 0, 0, 0},
    {"green", Uint8, Uint8, offsetof(Vertex, g), 0, 0, 0, 0},
    {"blue", Uint8, Uint8, offsetof(Vertex, b), 0, 0, 0, 0}
  };

  PlyProperty face_props[]  = { /* list of property information for a PlyFace */
    {"vertex_indices", Int32, Int32, offsetof(PlyFace, verts), 1, Uint8, Uint8, offsetof(PlyFace, nverts)},
  };

  FILE*    fp  = fopen(fn, "r");
  CHECK(fp, "coult not open file " <<fn <<" from path "<<getcwd_string())
  PlyFile* ply = read_ply(fp);

  //-- get the number of faces and vertices
  for(uint i = 0; i < (uint)ply->num_elem_types; ++i) {
    int elem_count ;
    char* elem_name = setup_element_read_ply(ply, i, &elem_count);
    if(equal_strings("vertex", elem_name)) _nverts = elem_count;
    if(equal_strings("face",   elem_name)) _ntrigs = elem_count;
  }
  V.resize(_nverts, 3);
  T.resize(_ntrigs, 3);

  //-- examine each element type that is in the file (PlyVertex, PlyFace)
  for(int i = 0; i < ply->num_elem_types; ++i)  {
    int elem_count ;
    char* elem_name = setup_element_read_ply(ply, i, &elem_count);

    if(equal_strings("vertex", elem_name))   {
      /* set up for getting PlyVertex elements */
      int r=1;
      r &= setup_property_ply(ply, &vert_props[0]);
      r &= setup_property_ply(ply, &vert_props[1]);
      r &= setup_property_ply(ply, &vert_props[2]);
      if(!r) HALT("no vertices defined??");
      if(ply_check_property(ply->which_elem, "red")) {
        r &= setup_property_ply(ply, &vert_props[3]);
        r &= setup_property_ply(ply, &vert_props[4]);
        r &= setup_property_ply(ply, &vert_props[5]);
        if(r && C.N!=V.N) C.resize(_nverts, 3); //has color
      }

      Vertex vertex;
      for(uint j = 0; j < _nverts; ++j) {
        get_element_ply(ply, &vertex);
        V(j, 0) = vertex.x;
        V(j, 1) = vertex.y;
        V(j, 2) = vertex.z;
        if(C.N==V.N) {
          C(j, 0) = vertex.r;
          C(j, 1) = vertex.g;
          C(j, 2) = vertex.b;
        }
      }
    } else if(equal_strings("face", elem_name))  {
      /* set up for getting PlyFace elements */
      /* (all we need are PlyVertex indices) */
      setup_property_ply(ply, &face_props[0]) ;
      PlyFace     face ;
      for(uint j = 0; j < _ntrigs; ++j)   {
        get_element_ply(ply, (void*) &face);
        if(face.nverts != 3)
          HALT("not a triangulated surface: polygon " <<j <<" has " <<face.nverts <<" sides") ;

        T(j, 0) = face.verts[0];
        T(j, 1) = face.verts[1];
        T(j, 2) = face.verts[2];

        free(face.verts) ;
      }
    } else {
      /* all non-PlyVertex and non-PlyFace elements are grabbed here */
      PlyOtherElems* other = get_other_element_ply(ply);
      free_other_elements_ply(other);
    }
  }

  free_ply(ply);
  fclose(fp);

  if(C.N && ::max(C)>1.) C /= 255.;
}

#else
void Mesh::writePLY(const char* fn, bool bin) {
  writeAssimp(*this, fn, "ply");
}
void Mesh::readPLY(const char* fn) { NICO }
#endif

void Mesh::writeJson(std::ostream& os) {
  os <<"{\nV: ";
  convert<float>(V).writeJson(os);
  os <<",\nT: ";
  T.writeJson(os);
  os <<"\n}" <<endl;
}

void Mesh::readJson(std::istream& is) {
  parse(is, "{", false);
  parse(is, "V:", false);
  floatA tmp;
  tmp.readJson(is);
  copy(V, tmp);
  parse(is, ",", false);
  parse(is, "T:", false);
  T.readJson(is);
  parse(is, "}", false);
}

void Mesh::writeArr(std::ostream& os) {
  Graph G;
  G.add("V", convert<float>(V));
  if(V.d0<65535) G.add("T", convert<uint16_t>(T)); else G.add("T", T);
  if(C.N) G.add("C", convert<byte>(C*255.f));
  if(cvxParts.N) G.add("cvxParts", cvxParts);
  if(texCoords.N) G.add("textureCoords", texCoords);
  if(_texImg) G.add("textureImg", texImg().img);
  G.write(os, ",\n", "{\n\n}", -1, false, true);
}

void Mesh::writeAssimp(const char* filename, const char* format){
  ::writeAssimp(*this, filename, format);
}

void Mesh::writeH5(const char* filename, const str& group) {
  H5_Writer H(filename);
  H.addGroup(group);
  H.add(group + "/vertices", convert<float>(V));
  if(V.d0<65535) H.add(group+"/faces", convert<uint16_t>(T)); else H.add(group+"/faces", T);
  if(C.N) H.add(group+"/colors", convert<byte>(C*255.));
  if(cvxParts.N) H.add(group+"/parts", cvxParts);
  if(texCoords.N) H.add(group+"/textureCoords", convert<float>(texCoords));
  if(_texImg){
    if(_texImg->file.N) H.add<char>(group+"/textureFile", texImg().file);
    else H.add<byte>(group+"/textureImg", texImg().img);
  }
}

void Mesh::readH5(const char* filename, const str& group) {
  H5_Reader H(filename);
  V = H.read<double>(group+"/vertices");
  Vn = H.read<double>(group+"/normals", true);
  T = H.read<uint>(group+"/faces", true);
  C = convert<double>(H.read<byte>(group+"/colors", true))/255.;
  cvxParts = H.read<uint>(group+"/parts", true);
  texCoords = H.read<double>(group+"/textureCoords", true);
  charA __texFile = H.read<char>(group+"/textureFile", true);
  if(__texFile.N) texImg(__texFile.p);
  else{
    byteA __texImg = H.read<byte>(group+"/textureImg", true);
    if(__texImg.N) texImg().img = __texImg;
  }
}

#if 1
void Mesh::readArr(std::istream& is) {
  clear();
  Graph G(is);
  rai::Node* n;
  n=G.findNode("V"); if(n) { if(n->is<arr>()) V = n->as<arr>(); else V = convert<double>(n->as<floatA>()); }
  n=G.findNode("T"); if(n) { if(n->is<uintA>()) T = n->as<uintA>(); else { if(n->is<Array<int16_t>>()) T = convert<uint>(n->as<Array<int16_t>>()); else T = convert<uint>(n->as<uint16A>()); } }
  n=G.findNode("C"); if(n) { if(n->is<arr>()) C = n->as<arr>(); else { if(n->is<byteA>()) C = convert<double>(n->as<byteA>())/255.; else C = convert<double>(n->as<floatA>()); } }
  G.get(cvxParts, "cvxParts");
  G.get(texCoords, "textureCoords");
  n=G.findNode("textureImg"); if(n) { if(n->is<byteA>()) texImg().img = n->as<byteA>(); }
}
#else //old version...
void rai::Mesh::readArr(std::istream& is){
  V.readTagged(is, "V");
  T.readTagged(is, "T");
  C.readTagged(is, "C");
  tex.readTagged(is, "textureCoords");
  texImg.readTagged(is, "textureImg");
}
#endif

void Mesh::readPts(std::istream& is) {
  floatA pts;
  pts.readJson(is);
  if(pts.d1==3) {
    rai::copy(V, pts);
  } else {
    CHECK_EQ(pts.d1, 6, "need only points (3D), or points and normals (6D)");
    rai::copy(V, pts.sub({0, -1+1},{ 0, 2+1}));
    rai::copy(Vn, pts.sub({0, -1+1},{ 3, 5+1}));
  }
  C = {0., 0., .3};
}

//===========================================================================
// Util
/**
 * @brief Return the position of the submesh in the obj file in bytes (can be
 * used by fseek).
 *
 * @param filename file to parse.
 */
uintA getSubMeshPositions(const char* filename) {
  CHECK(String(filename).endsWith("obj"),
        "getSubMeshPositions parses only obj files.");
  FILE* file;
  char buf[128];
  file = fopen(filename, "r");
  CHECK(file, "can't open data file " <<filename << "; cwd is " <<getcwd_string());

  int flag = 0;
  long start_pos = 0;
  long end_pos = 0;

  uintA result;
  while(fscanf(file, "%s", buf) != EOF) {
    switch(buf[0]) {
      case 'v': {
        if(flag > 0) {
          end_pos = ftell(file) - 1;
          result.append(uintA{(uint)start_pos, (uint)end_pos});
          start_pos = end_pos;
          flag =0;
        }
      } break;
      case 'f': {
        flag=1;
      } break;
    }
  }

  end_pos = ftell(file) - 1;
  result.append(uintA{(uint)start_pos, (uint)end_pos});
  result.reshape(result.N/2, 2);
  return result;
}

//===========================================================================
//
// GJK interface (obsolete - use PairCollision)
//

#if 0 //def RAI_GJK
GJK_point_type& NoPointType = *((GJK_point_type*)nullptr);
template<> const char* Enum<GJK_point_type>::names []= { "GJK_none", "GJK_vertex", "GJK_edge", "GJK_face", nullptr };
double GJK_sqrDistance(const Mesh& mesh1, const Mesh& mesh2,
                       const Transformation& t1, const Transformation& t2,
                       Vector& p1, Vector& p2,
                       Vector& e1, Vector& e2,
                       GJK_point_type& pt1, GJK_point_type& pt2) {
  // convert meshes to 'Object_structures'
  Object_structure m1, m2;
  Array<double*> Vhelp1, Vhelp2;
  m1.numpoints = mesh1.V.d0;  m1.vertices = mesh1.V.getCarray(Vhelp1);  m1.rings=nullptr; //TODO: rings would make it faster
  m2.numpoints = mesh2.V.d0;  m2.vertices = mesh2.V.getCarray(Vhelp2);  m2.rings=nullptr;

  // convert transformations to affine matrices
  arr T1, T2;
  Array<double*> Thelp1, Thelp2;
  if(!!t1) {  T1=t1.getAffineMatrix();  T1.getCarray(Thelp1);  }
  if(!!t2) {  T2=t2.getAffineMatrix();  T2.getCarray(Thelp2);  }

  // call GJK
  simplex_point simplex;
  double d2 = gjk_distance(&m1, Thelp1.p, &m2, Thelp2.p, (!p1?nullptr:p1.p()), (!p2?nullptr:p2.p()), &simplex, 0);

//  cout <<"simplex npts=" <<simplex.npts <<endl;
//  cout <<"simplex lambda=" <<arr(simplex.lambdas, 4) <<endl;
//  cout <<"simplex 1=" <<intA(simplex.simplex1, 4) <<endl;
//  cout <<"simplex 2=" <<intA(simplex.simplex2, 4) <<endl;

//  arr P1=zeros(3), P2=zeros(3);
//  for(int i=0;i<simplex.npts;i++) P1 += simplex.lambdas[i] * arr(simplex.coords1[i],3);
//  for(int i=0;i<simplex.npts;i++) P2 += simplex.lambdas[i] * arr(simplex.coords2[i],3);
//  cout <<"P1=" <<P1 <<", " <<p1 <<endl;
//  cout <<"P2=" <<P2 <<", " <<p2 <<endl;

  // analyze point types
  if(!!e1 && !!e2) {
    e1.setZero();
    e2.setZero();
    pt1=GJK_vertex;
    pt2=GJK_vertex;
    if(d2<1e-6) return d2;

    if(simplex.npts==1) {

    } else if(simplex.npts==2) {

      if(simplex.simplex1[0]==simplex.simplex1[1]) {
        pt1=GJK_vertex;
      } else {
        pt1=GJK_edge;
        for(uint i=0; i<3; i++) e1(i) = simplex.coords1[0][i] - simplex.coords1[1][i];
        e1.normalize();
      }
      if(simplex.simplex2[0]==simplex.simplex2[1]) {
        pt2=GJK_vertex;
      } else {
        pt2=GJK_edge;
        for(uint i=0; i<3; i++) e2(i) = simplex.coords2[0][i] - simplex.coords2[1][i];
        e2.normalize();
      }

    } else if(simplex.npts==3) {

      // 1st point
      if(simplex.simplex1[0]==simplex.simplex1[1] && simplex.simplex1[0]==simplex.simplex1[2]) {
        pt1=GJK_vertex;
      } else if(simplex.simplex1[0]!=simplex.simplex1[1] && simplex.simplex1[0]!=simplex.simplex1[2] && simplex.simplex1[1]!=simplex.simplex1[2]) {
        pt1=GJK_face;
      } else {
        pt1=GJK_edge;
        if(simplex.simplex1[0]==simplex.simplex1[1]) {
          for(uint i=0; i<3; i++) e1(i) = simplex.coords1[0][i] - simplex.coords1[2][i];
        } else {
          for(uint i=0; i<3; i++) e1(i) = simplex.coords1[0][i] - simplex.coords1[1][i];
        }
        e1.normalize();
      }

      // 2nd point
      if(simplex.simplex2[0]==simplex.simplex2[1] && simplex.simplex2[0]==simplex.simplex2[2]) {
        pt2=GJK_vertex;
      } else if(simplex.simplex2[0]!=simplex.simplex2[1] && simplex.simplex2[0]!=simplex.simplex2[2] && simplex.simplex2[1]!=simplex.simplex2[2]) {
        pt2=GJK_face;
      } else {
        pt2=GJK_edge;
        if(simplex.simplex2[0]==simplex.simplex2[1]) {
          for(uint i=0; i<3; i++) e2(i) = simplex.coords2[0][i] - simplex.coords2[2][i];
        } else {
          for(uint i=0; i<3; i++) e2(i) = simplex.coords2[0][i] - simplex.coords2[1][i];
        }
        e2.normalize();
      }

    } else {
      if(d2>EPSILON) THROW("GJK converges to simplex!")
      }

//    cout <<"point types= " <<pt1 <<' ' <<pt2 <<endl;
//    CHECK(!(pt1==3 && pt2==3),"");
//    CHECK(!(pt1==2 && pt2==3),"");
//    CHECK(!(pt1==3 && pt2==2),"");
  }

  return d2;
}
#else
double GJK_distance(Mesh& mesh1, Mesh& mesh2,
                    Transformation& t1, Transformation& t2,
                    Vector& p1, Vector& p2) { NICO }
#endif

//===========================================================================
//
// Lewiner interface
//

#ifdef RAI_Lewiner

void Mesh::setImplicitSurface(std::function<double(const arr& x)> f, double lo, double up, uint res) {
  setImplicitSurface(f, arr{lo, lo, lo, up, up, up}.reshape(2,3), res);
}

void Mesh::setImplicitSurface(std::function<double(const arr& x)> f, const arr& bounds, uint res){
  NIY;
  // MarchingCubes mc(res, res, res);
  // mc.init_all() ;

  // //compute data
  // uint k=0, j=0, i=0;
  // float x, y, z;
  // float xLo=bounds.elem(0), yLo=bounds.elem(1), zLo=bounds.elem(2);
  // float xUp=bounds.elem(3), yUp=bounds.elem(4), zUp=bounds.elem(5);
  // for(k=0; k<res; k++) {
  //   z = zLo+k*(zUp-zLo)/res;
  //   for(j=0; j<res; j++) {
  //     y = yLo+j*(yUp-yLo)/res;
  //     for(i=0; i<res; i++) {
  //       x = xLo+i*(xUp-xLo)/res;
  //       mc.set_data(f(arr{(double)x, (double)y, (double)z}), i, j, k) ;
  //     }
  //   }
  // }

  // mc.run();
  // mc.clean_temps();

  // //convert to Mesh
  // clear();
  // V.resize(mc.nverts(), 3);
  // T.resize(mc.ntrigs(), 3);
  // for(i=0; i<V.d0; i++) {
  //   V(i, 0)=xLo+mc.vert(i)->x*(xUp-xLo)/res;
  //   V(i, 1)=yLo+mc.vert(i)->y*(yUp-yLo)/res;
  //   V(i, 2)=zLo+mc.vert(i)->z*(zUp-zLo)/res;
  // }
  // for(i=0; i<T.d0; i++) {
  //   T(i, 0)=mc.trig(i)->v1;
  //   T(i, 1)=mc.trig(i)->v2;
  //   T(i, 2)=mc.trig(i)->v3;
  // }
}

void Mesh::setImplicitSurface(const arr& gridValues, const arr& size) {
  floatA D;
  copy(D, gridValues);
  setImplicitSurface(D, size);
}

void Mesh::setImplicitSurface(const floatA& gridValues, const arr& size) {
  CHECK_EQ(gridValues.nd, 3, "");

#if 1
  clear();
  std::tie(V, T) = rai::marching_cubes(gridValues, size);
#else
  MarchingCubes mc(gridValues.d0, gridValues.d1, gridValues.d2);
  mc.init_all() ;
  uint k=0, j=0, i=0;
  for(k=0; k<gridValues.d2; k++) {
    for(j=0; j<gridValues.d1; j++) {
      for(i=0; i<gridValues.d0; i++) {
        mc.set_data(gridValues(i, j, k), i, j, k) ;
      }
    }
  }

  mc.run();
  mc.clean_temps();

  //convert to Mesh
  clear();
  V.resize(mc.nverts(), 3);
  T.resize(mc.ntrigs(), 3);
  for(i=0; i<V.d0; i++) {
    V(i, 0)=lo(0)+mc.vert(i)->x*(up(0)-lo(0))/(gridValues.d0-1);
    V(i, 1)=lo(1)+mc.vert(i)->y*(up(1)-lo(1))/(gridValues.d1-1);
    V(i, 2)=lo(2)+mc.vert(i)->z*(up(2)-lo(2))/(gridValues.d2-1);
  }
  for(i=0; i<T.d0; i++) {
    T(i, 0)=mc.trig(i)->v1;
    T(i, 1)=mc.trig(i)->v2;
    T(i, 2)=mc.trig(i)->v3;
  }
#endif
}

#else //Lewiner
void Mesh::setImplicitSurface(const ScalarFunction& f, double lo, double hi, uint res) {  NICO  }
void Mesh::setImplicitSurface(const floatA& gridValues, const arr& center, const arr& size) { NICO }
#endif

void Mesh::setImplicitSurfaceBySphereProjection(ScalarFunction& _f, double rad, uint fineness) {
  setSphere(fineness);
  scale(rad);

  Conv_cfunc2ScalarFunction distSqr( [&_f](arr& g, arr& H, const arr& x) {
    double d = _f.f(g, H, x);
    H *= 2.*d;
    H += 2.*(g^g);
    g *= 2.*d;
    return d*d;
  });

  for(uint i=0; i<V.d0; i++) {
    arr x = V[i];

    OptNewton newton(x, distSqr, OptOptions()
                     .set_verbose(0)
                     .set_stepMax(.5*rad)
                     .set_damping(1e-10));
    newton.run();
  }
}

void Mesh::buildGraph() {
  graph.resize(V.d0);
  for(uint i=0; i<T.d0; i++) {
    graph(T(i, 0)).setAppend(T(i, 1));
    graph(T(i, 0)).setAppend(T(i, 2));
    graph(T(i, 1)).setAppend(T(i, 0));
    graph(T(i, 1)).setAppend(T(i, 2));
    graph(T(i, 2)).setAppend(T(i, 0));
    graph(T(i, 2)).setAppend(T(i, 1));
  }
}

inline double __scalarProduct(const double* p1, const double* p2) {
  return p1[0]*p2[0]+p1[1]*p2[1]+p1[2]*p2[2];
}

uint Mesh::support(const double* dir) {
#if 1

  arr _dir(dir, 3, true);
  arr q = V*_dir;
  return argmax(q);

#elif 0

  double s = __scalarProduct(dir, V.p);
  double ms=s;
  uint mi=0;
  for(uint i=0; i<V.d0; i++) {
    s = __scalarProduct(dir, V.p+3*i);
    if(s>ms) { ms = s;  mi = i; }
  }
  _support_vertex = mi;
  return _support_vertex;

#else
  if(!graph.N) buildGraph();

  uint mi = _support_vertex;
  double s = __scalarProduct(dir, V.p+3*mi);
  double ms=s;
  for(;;) {
    //comput scalar product for all neighbors
    uintA& neigh=graph.p[mi];

    bool stop=true;
    for(uint i:neigh) {
      s = __scalarProduct(dir, V.p+3*i);
      if(s>ms) {
        mi = i;
        ms = s;
        stop = false;
        break;
      }
    }
    if(stop) {
      _support_vertex = mi;
      return _support_vertex;
    }
  }

#endif
}

} //namespace

//==============================================================================


void supportMargin(const arr& V, uintA& verts, const arr& dir, double margin, int initialization) {
  NIY
#if 0
  if(initialization<0 || !graph.N) initialization=support(dir.p);

  arr p = V[initialization];
  double max = scalarProduct(p, dir);

  boolA done(V.d0); done=false;
  uintA queue = { (uint) initialization };
  verts.clear();

  for(; queue.N;) {
    uint i = queue.popFirst();
    if(done(i)) continue;
    done(i) = true;
    if(scalarProduct(V[i], dir)>=max-margin) {
      verts.append(i);
      for(uint j : graph(i)) if(!done(j)) queue.append(j);
    }
  }
#endif
}

//==============================================================================

arr MinkowskiSum(const arr& A, const arr& B) {
  arr S;
  for(uint i=0; i<A.d0; i++) {
    const arr& a = A[i];
    for(uint j=0; j<B.d0; j++) {
      const arr& b = B[j];
      S.append(a+b);
    }
  }
  S.reshape(-1, A.d1);
  return S;
}

//==============================================================================

void inertiaSphere(double* I, double& mass, double density, double radius) {
  double r2=radius*radius;
  if(density) mass=density*4./3.*RAI_PI*r2*radius;
  I[1]=I[2]=I[3]=I[5]=I[6]=I[7]=0.;
  I[0]=.4*mass*r2;
  I[4]=.4*mass*r2;
  I[8]=.4*mass*r2;
}

void inertiaBox(double* I, double& mass, double density, double dx, double dy, double dz) {
  if(density) mass=density*dx*dy*dz;
  I[1]=I[2]=I[3]=I[5]=I[6]=I[7]=0.;
  double x2=dx*dx, y2=dy*dy, z2=dz*dz;
  I[0]=mass/12.*(y2+z2);
  I[4]=mass/12.*(x2+z2);
  I[8]=mass/12.*(x2+y2);
}

void inertiaCylinder(double* I, double& mass, double density, double height, double radius) {
  double r2=radius*radius, h2=height*height;
  if(density) mass=density*RAI_PI*r2*height;
  I[1]=I[2]=I[3]=I[5]=I[6]=I[7]=0.;
  I[0]=mass/12.*(3.*r2+h2);
  I[4]=mass/12.*(3.*r2+h2);
  I[8]=mass/2.*r2;
}

void inertiaSphereSurface(double& mass, double* I, double radius, double density) {
  double area = 4.*RAI_PI*radius;
  if(density>0.) mass = density * area;
  I[1]=I[2]=I[3]=I[5]=I[6]=I[7]=0.;
  double r2=radius*radius;
  I[0]=mass*r2;
  I[4]=mass*r2;
  I[8]=mass*r2;
}

void inertiaBoxSurface(double& mass, double* I, double dx, double dy, double dz, double density) {
  double area = 2.*(dx*dy+dy*dz+dx*dz);
  if(density>0.) mass = density * area;
  I[1]=I[2]=I[3]=I[5]=I[6]=I[7]=0.;
  double x2=dx*dx, y2=dy*dy, z2=dz*dz;
  I[0]=mass*(y2+z2);
  I[4]=mass*(x2+z2);
  I[8]=mass*(x2+y2);
}

void inertiaMeshSurface(double& mass, double* com, double* I, const rai::Mesh& m, double density) {
  //get total area and assign to vertices
  double area=0.;
  arr pts(m.T.d0, 3, 3);
  arr weights(m.T.d0, 3);

  for(uint i=0; i<m.T.d0; i++) {
    double ai = m.getArea(i);
    area += ai;
    rai::Vector a, b, c;
    a.set(m.V.p+3*m.T.p[3*i+0]);
    b.set(m.V.p+3*m.T.p[3*i+1]);
    c.set(m.V.p+3*m.T.p[3*i+2]);
    pts(i,0,{}) = (.5*(a+b)).getArr();  weights(i,0) = ai/3.;
    pts(i,1,{}) = (.5*(a+c)).getArr();  weights(i,1) = ai/3.;
    pts(i,2,{}) = (.5*(b+c)).getArr();  weights(i,2) = ai/3.;
  }
  //assign mass via density-per-area
  if(density>0.){
    mass = density*area; //m.getVolume();
  }else{
    density = mass/area;
  }
  weights *= density;

  //sum up com and inertia tensor
  for(uint i=0;i<3;i++) com[i]=0.;
  for(uint i=0;i<9;i++) I[i]=0.;
  pts.reshape(-1,3);
  weights.reshape(pts.d0);
  for(uint i=0; i<pts.d0; i++) {
    double w = weights(i);
    double x=pts(i,0), y=pts(i,1), z=pts(i,2);
    com[0] += w*x;
    com[1] += w*y;
    com[2] += w*z;
    I[0] += w*(y*y+z*z);
    I[4] += w*(x*x+z*z);
    I[8] += w*(x*x+y*y);
    I[1] -= w*x*y;  I[3] -= w*x*y;
    I[2] -= w*x*z;  I[6] -= w*x*z;
    I[5] -= w*y*z;  I[7] -= w*y*z;
  }
  for(uint i=0;i<3;i++) com[i] /= mass;

  double x=com[0], y=com[1], z=com[2];
  double w=-mass;
  I[0] += w*(y*y+z*z);
  I[4] += w*(x*x+z*z);
  I[8] += w*(x*x+y*y);
  I[1] -= w*x*y;  I[3] -= w*x*y;
  I[2] -= w*x*z;  I[6] -= w*x*z;
  I[5] -= w*y*z;  I[7] -= w*y*z;
}
