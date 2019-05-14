/*  ------------------------------------------------------------------
    Copyright (c) 2017 Marc Toussaint
    email: marc.toussaint@informatik.uni-stuttgart.de

    This code is distributed under the MIT License.
    Please see <root-path>/LICENSE for details.
    --------------------------------------------------------------  */

#include "mesh.h"
#include "qhull.h"
#include "mesh_readAssimp.h"

#include <limits>

#define RAI_extern_ply
#define RAI_extern_GJK
#define RAI_extern_Lewiner

#ifdef RAI_extern_ply
#  include "ply/ply.h"
#endif

#ifdef RAI_extern_GJK
extern "C" {
#  include "GJK/gjk.h"
}
#endif

#ifdef RAI_GL
#  include <GL/gl.h>
#endif

bool orsDrawWires=false;

bool Geo_mesh_drawColors=true;

extern void glColorId(uint id);


//==============================================================================
//
// Mesh code
//

rai::Mesh::Mesh()
  : glX(0)
    /*parsing_pos_start(0),
    parsing_pos_end(std::numeric_limits<long>::max())*/{}

void rai::Mesh::clear() {
  V.clear(); Vn.clear(); T.clear(); Tn.clear(); C.clear(); //strips.clear();
}

void rai::Mesh::setBox() {
  double verts[24] = {
    -.5, -.5, -.5 ,
    +.5, -.5, -.5 ,
    +.5, +.5, -.5 ,
    -.5, +.5, -.5 ,
    -.5, -.5, +.5 ,
    +.5, -.5, +.5 ,
    +.5, +.5, +.5 ,
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
  V.setCarray(verts, 24);
  T.setCarray(tris , 36);
  V.reshape(8, 3);
  T.reshape(12, 3);
  Vn.clear(); Tn.clear();
  //cout <<V <<endl;  for(uint i=0;i<4;i++) cout <<length(V[i]) <<endl;
}

void rai::Mesh::setDot() {
  V.resize(1,3).setZero(); Vn.clear();
  T.clear(); Tn.clear();
}

void rai::Mesh::setLine(double l){
  V.resize(2,3).setZero();
  V(0,2) = -.5*l;
  V(1,2) = +.5*l;
  Vn.clear();
  T.clear(); Tn.clear();
}

void rai::Mesh::setTetrahedron() {
  double s2=RAI_SQRT2/3., s6=sqrt(6.)/3.;
  double verts[12] = { 0., 0., 1. , 2.*s2, 0., -1./3., -s2, s6, -1./3., -s2, -s6, -1./3. };
  uint   tris [12] = { 0, 1, 2, 0, 2, 3, 0, 3, 1, 1, 3, 2 };
  V.setCarray(verts, 12);
  T.setCarray(tris , 12);
  V.reshape(4, 3);
  T.reshape(4, 3);
  Vn.clear(); Tn.clear();
  //cout <<V <<endl;  for(uint i=0;i<4;i++) cout <<length(V[i]) <<endl;
}

void rai::Mesh::setOctahedron() {
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
  T.setCarray(tris , 24);
  V.reshape(6, 3);
  T.reshape(8, 3);
  Vn.clear(); Tn.clear();
  //cout <<V <<endl;  for(uint i=0;i<4;i++) cout <<length(V[i]) <<endl;
}

void rai::Mesh::setDodecahedron() {
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
  T.setCarray(tris , 108);
  V.reshape(20, 3);
  T.reshape(36, 3);
  Vn.clear(); Tn.clear();
}

void rai::Mesh::setSphere(uint fineness) {
  setOctahedron();
//  setDodecahedron();
//  setTetrahedron();
  for(uint k=0; k<fineness; k++) {
    subDivide();
    for(uint i=0; i<V.d0; i++) V[i]() /= length(V[i]);
  }
  makeConvexHull();
}

void rai::Mesh::setHalfSphere(uint fineness) {
  setOctahedron();
  V.resizeCopy(5, 3);
  T.resizeCopy(4, 3);
  for(uint k=0; k<fineness; k++) {
    subDivide();
    for(uint i=0; i<V.d0; i++) V[i]() /= length(V[i]);
  }
  makeConvexHull();
}

void rai::Mesh::setCylinder(double r, double l, uint fineness) {
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
    T(4*i  , 0)=i;
    T(4*i  , 1)=j+div;
    T(4*i  , 2)=j;
    
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
  Vn.clear(); Tn.clear();
}

void rai::Mesh::setSSBox(double x_width, double y_width, double z_height, double r, uint fineness) {
  CHECK(r>=0. && x_width>=2.*r && y_width>=2.*r && z_height>=2.*r, "width/height includes radius!");
  setSphere(fineness);
  scale(r);
  for(uint i=0; i<V.d0; i++) {
    V(i,0) += rai::sign(V(i,0))*(.5*x_width-r);
    V(i,1) += rai::sign(V(i,1))*(.5*y_width-r);
    V(i,2) += rai::sign(V(i,2))*(.5*z_height-r);
  }
  makeConvexHull();
}

void rai::Mesh::setCappedCylinder(double r, double l, uint fineness) {
  uint i;
  setSphere(fineness);
  scale(r);
  for(i=0; i<V.d0; i++) V(i, 2) += .5*rai::sign(V(i, 2))*l;
  makeConvexHull();
}

/** @brief add triangles according to the given grid; grid has to be a 2D
  Array, the elements of which are indices referring to vertices in
  the vertex list (V) */
void rai::Mesh::setGrid(uint X, uint Y) {
  CHECK(X>1 && Y>1, "grid has to be at least 2x2");
  CHECK_EQ(V.d0,X*Y, "don't have X*Y mesh-vertices to create grid faces");
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

void rai::Mesh::setRandom(uint vertices) {
  V.resize(vertices,3);
  rndUniform(V, -1., 1.);
  makeConvexHull();
}

void rai::Mesh::subDivide() {
  uint v=V.d0, t=T.d0;
  V.resizeCopy(v+3*t, 3);
  uintA newT(4*t, 3);
  uint a, b, c, i, k, l;
  for(i=0, k=v, l=0; i<t; i++) {
    a=T(i, 0); b=T(i, 1); c=T(i, 2);
    V[k+0]() = (double).5*(V[a] + V[b]);
    V[k+1]() = (double).5*(V[b] + V[c]);
    V[k+2]() = (double).5*(V[c] + V[a]);
    newT(l, 0)=a;   newT(l, 1)=k+0; newT(l, 2)=k+2; l++;
    newT(l, 0)=k+0; newT(l, 1)=b;   newT(l, 2)=k+1; l++;
    newT(l, 0)=k+0; newT(l, 1)=k+1; newT(l, 2)=k+2; l++;
    newT(l, 0)=k+2; newT(l, 1)=k+1; newT(l, 2)=c;   l++;
    k+=3;
  }
  T = newT;
//  fuseNearVertices();
}

void rai::Mesh::subDivide(uint i) {
  uint v=V.d0, t=T.d0;
  V.resizeCopy(v+3, 3);
  T.resizeCopy(t+3, 3);
  uint a, b, c;
  a=T(i, 0); b=T(i, 1); c=T(i, 2);
  V[v+0]() = (double).5*(V[a] + V[b]);
  V[v+1]() = (double).5*(V[b] + V[c]);
  V[v+2]() = (double).5*(V[c] + V[a]);
  T(i, 0)=a;   T(i, 1)=v+0; T(i, 2)=v+2; //the old ith tri becomes one of the 4 new ones
  T(t, 0)=v+0; T(t, 1)=b;   T(t, 2)=v+1; t++;
  T(t, 0)=v+0; T(t, 1)=v+1; T(t, 2)=v+2; t++;
  T(t, 0)=v+2; T(t, 1)=v+1; T(t, 2)=c;   t++;
}

void rai::Mesh::scale(double f) {  V *= f; }

void rai::Mesh::scale(double sx, double sy, double sz) {
  uint i;
  for(i=0; i<V.d0; i++) {  V(i, 0)*=sx;  V(i, 1)*=sy;  V(i, 2)*=sz;  }
}

void rai::Mesh::translate(double dx, double dy, double dz) {
  uint i;
  for(i=0; i<V.d0; i++) {  V(i, 0)+=dx;  V(i, 1)+=dy;  V(i, 2)+=dz;  }
}

void rai::Mesh::translate(const arr& d) {
  CHECK_EQ(d.N,3,"");
  translate(d.elem(0), d.elem(1), d.elem(2));
}

void rai::Mesh::transform(const rai::Transformation& t) {
  t.applyOnPointArray(V);
}

rai::Vector rai::Mesh::center() {
  arr Vmean = mean(V);
  for(uint i=0; i<V.d0; i++) V[i]() -= Vmean;
  return Vector(Vmean);
}

void rai::Mesh::box() {
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

void rai::Mesh::addMesh(const Mesh& mesh2, const rai::Transformation& X) {
  uint n=V.d0, t=T.d0, tt=Tt.d0;
  V.append(mesh2.V);
  if(V.N==C.N && mesh2.V.N==mesh2.C.N) C.append(mesh2.C); else C.clear();
  tex.append(mesh2.tex);
  T.append(mesh2.T);
  for(; t<T.d0; t++) {  T(t, 0)+=n;  T(t, 1)+=n;  T(t, 2)+=n;  }
  Tt.append(mesh2.Tt);
  for(; tt<Tt.d0; tt++) {  Tt(tt, 0)+=n;  Tt(tt, 1)+=n;  Tt(tt, 2)+=n;  }
  if(!X.isZero()) {
    X.applyOnPointArray(V({n,-1})());
  }
}

void rai::Mesh::makeConvexHull() {
  if(!V.N) return;
#if 1
  V = getHull(V, T);
  if(C.nd==2) C = mean(C);
  Vn.clear();
  Tn.clear();
  Tt.clear();
  tex.clear();
  texImg.clear();
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
    T(i,0) = Hinv(T(i,0));
    T(i,1) = Hinv(T(i,1));
    T(i,2) = Hinv(T(i,2));
  }
  
#endif
}

void rai::Mesh::makeTriangleFan() {
  T.clear();
  for(uint i=1; i+1<V.d0; i++) {
    T.append(TUP(0,i,i+1));
    T.append(TUP(0,i+1,i));
  }
  T.reshape(T.N/3,3);
}

void rai::Mesh::makeLineStrip() {
  T.resize(V.d0, 2);
  T[0] = {V.d0-1, 0};
  for(uint i=1; i<V.d0; i++) {
    T[i] = {i-1, i};
  }
}

void rai::Mesh::setSSCvx(const rai::Mesh& m, double r, uint fineness) {
  if(r>0.) {
    Mesh ball;
    ball.setSphere(fineness);
    ball.scale(r);
    
    arr c=C;
    clear();
    for(uint i=0; i<m.V.d0; i++) {
      ball.translate(m.V(i,0), m.V(i,1), m.V(i,2));
      addMesh(ball);
      ball.translate(-m.V(i,0), -m.V(i,1), -m.V(i,2));
    }
    makeConvexHull();
    C=c;
  } else {
    arr c=C;
    operator=(m);
    C=c;
  }
}

/** @brief calculate the normals of all triangles (Tn) and the average
  normals of the vertices (N); average normals are averaged over
  all adjacent triangles that are in the triangle list or member of
  a strip */
void rai::Mesh::computeNormals() {
  uint i;
  Vector a, b, c;
  Tn.resize(T.d0, 3);
  Tn.setZero();
  Vn.resize(V.d0, 3);
  Vn.setZero();
  //triangle normals and contributions
  for(i=0; i<T.d0; i++) {
    uint *t=T.p+3*i;
    a.set(V.p+3*t[0]);
    b.set(V.p+3*t[1]);
    c.set(V.p+3*t[2]);
    
    b-=a; c-=a; a=b^c; if(!a.isZero) a.normalize();
    Tn(i, 0)=a.x;  Tn(i, 1)=a.y;  Tn(i, 2)=a.z;
    Vn(t[0], 0)+=a.x;  Vn(t[0], 1)+=a.y;  Vn(t[0], 2)+=a.z;
    Vn(t[1], 0)+=a.x;  Vn(t[1], 1)+=a.y;  Vn(t[1], 2)+=a.z;
    Vn(t[2], 0)+=a.x;  Vn(t[2], 1)+=a.y;  Vn(t[2], 2)+=a.z;
  }
  Vector d;
  for(i=0; i<Vn.d0; i++) { d.set(&Vn(i, 0)); Vn[i]()/=d.length(); }
}

/** @brief add triangles according to the given grid; grid has to be a 2D
  Array, the elements of which are indices referring to vertices in
  the vertex list (V) */
/*void rai::Mesh::gridToTriangles(const uintA &grid){
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
/*void rai::Mesh::gridToStrips(const uintA& grid){
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
/*void rai::Mesh::gridToStrips(uint X, uint Y){
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

void deleteZeroTriangles(rai::Mesh& m) {
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

void permuteVertices(rai::Mesh& m, uintA& p) {
  CHECK_EQ(p.N,m.V.d0, "");
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
  uintA y(m.T.d0, 3);
  uintA p2(p.N); //inverse permutation
  for(i=0; i<p.N; i++) p2(p(i))=i;
  for(i=0; i<m.T.d0; i++) { y(i, 0)=p2(m.T(i, 0)); y(i, 1)=p2(m.T(i, 1)); y(i, 2)=p2(m.T(i, 2)); }
  m.T=y;
}

/** @brief delete all void triangles (with vertex indices (0, 0, 0)) and void
  vertices (not used for triangles or strips) */
void rai::Mesh::deleteUnusedVertices() {
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

arr *COMP_V;
bool COMP(uint i, uint j) {
  bool r=(*COMP_V)[i]<(*COMP_V)[j];
  return r;
}

/** @brief delete all void triangles (with vertex indices (0, 0, 0)) and void
  vertices (not used for triangles or strips) */
void rai::Mesh::fuseNearVertices(double tol) {
  if(!V.N) return;
  uintA p;
  uint i, j;
  
  if(C.N==V.N) C.clear();

//  cout <<"fusing vertices: #V=" <<V.d0 <<", sorting.." <<std::flush;
  //cout <<V <<endl;
  //sort vertices lexically
  p.setStraightPerm(V.d0);
  COMP_V=&V;
  uint *pstop=p.p+p.N;
  std::sort(p.p, pstop, COMP);
  permuteVertices(*this, p);
  
//  cout <<"permuting.." <<std::flush;
  //cout <<V <<endl;
  p.setStraightPerm(V.d0);
  for(i=0; i<V.d0; i++) {
    if(p(i)!=i) continue;  //i has already been fused with p(i), and p(i) has already been checked...
    for(j=i+1; j<V.d0; j++) {
      if(V(j, 0)-V(i, 0)>tol) break;
      if(rai::sqr(V(j, 0)-V(i, 0))+rai::sqr(V(j, 1)-V(i, 1))+rai::sqr(V(j, 2)-V(i, 2))<tol*tol) {
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
  
  Tt.clear();
  tex.clear();
  texImg.clear();
}

void getVertexNeighorsList(const rai::Mesh& m, intA& Vt, intA& VT) {
  uint i, j;
  Vt.resize(m.V.d0);  Vt.setZero();
  VT.resize(m.V.d0, 100);
  for(i=0; i<m.T.d0; i++) {
    j=m.T(i, 0);  VT(j, Vt(j))=i;  Vt(j)++;
    j=m.T(i, 1);  VT(j, Vt(j))=i;  Vt(j)++;
    j=m.T(i, 2);  VT(j, Vt(j))=i;  Vt(j)++;
  }
}

void getTriNormals(const rai::Mesh& m, arr& Tn) {
  uint i;
  rai::Vector a, b, c;
  Tn.resize(m.T.d0, 3); //tri normals
  for(i=0; i<m.T.d0; i++) {
    a.set(&m.V(m.T(i, 0), 0)); b.set(&m.V(m.T(i, 1), 0)); c.set(&m.V(m.T(i, 2), 0));
    b-=a; c-=a; a=b^c; a.normalize();
    Tn(i, 0)=a.x;  Tn(i, 1)=a.y;  Tn(i, 2)=a.z;
  }
}

/// flips all faces
void rai::Mesh::flipFaces() {
  uint i, a;
  for(i=0; i<T.d0; i++) {
    a=T(i, 0);
    T(i, 0)=T(i, 1);
    T(i, 1)=a;
  }
}

/// check whether this is really a closed mesh, and flip inconsistent faces
void rai::Mesh::clean() {
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
      setSection(neighbors, VT[A], VT[B]);
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

  Tt.clear();
  tex.clear();
  texImg.clear();
  computeNormals();
}

void getEdgeNeighborsList(const rai::Mesh& m, uintA& EV, uintA& Et, intA& ET) {
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

void getTriNeighborsList(const rai::Mesh& m, uintA& Tt, intA& TT) {
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

void rai::Mesh::skin(uint start) {
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

arr rai::Mesh::getMean() const {
  return mean(V);
}

rai::Vector rai::Mesh::getCenter() const {
  return Vector(getMean());
}

void rai::Mesh::getBox(double& dx, double& dy, double& dz) const {
  dx=dy=dz=0.;
  for(uint i=0; i<V.d0; i++) {
    dx=rai::MAX(dx, fabs(V(i,0)));
    dy=rai::MAX(dy, fabs(V(i,1)));
    dz=rai::MAX(dz, fabs(V(i,2)));
  }
}

arr rai::Mesh::getBox() const {
  arr a,b;
  a = b = V[0];
  for(uint i=0; i<V.d0; i++) {
    a = elemWiseMin(a, V[i]);
    b = elemWiseMax(b, V[i]);
  }
  return cat(a,b).reshape(2,3);
}

double rai::Mesh::getRadius() const {
  double r=0.;
  for(uint i=0; i<V.d0; i++) r=rai::MAX(r, sumOfSqr(V[i]));
  return sqrt(r);
}

double triArea(const arr& a, const arr& b, const arr& c) {
  return .5*length(crossProduct(b-a, c-a));
}

double rai::Mesh::getArea() const {
  CHECK_EQ(T.d1, 3,"");
  double A=0.;
  for(uint i=0; i<T.d0; i++) A += getArea(i);
  return .5*A;
}

double rai::Mesh::getArea(uint i) const {
  CHECK_EQ(T.d1, 3,"");
  rai::Vector a,b,c;
  a.set(V.p+3*T.p[3*i+0]);
  b.set(V.p+3*T.p[3*i+1]);
  c.set(V.p+3*T.p[3*i+2]);
  return ((b-a)^(c-a)).length();
}

double rai::Mesh::getVolume() const {
  CHECK_EQ(T.d1, 3,"");
  rai::Vector z = getMean();
  rai::Vector a,b,c;
  double vol=0.;
  for(uint i=0; i<T.d0; i++) {
    a.set(V.p+3*T.p[3*i+0]);
    b.set(V.p+3*T.p[3*i+1]);
    c.set(V.p+3*T.p[3*i+2]);
    vol += (a-z) * ((b-a)^(c-a));
  }
  return vol/6.;
}

double rai::Mesh::meshMetric(const rai::Mesh& trueMesh, const rai::Mesh& estimatedMesh) {
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
  
  return rai::MAX(haussdorfDistanceOneSide(trueMesh.V, estimatedMesh.V), haussdorfDistanceOneSide(estimatedMesh.V, trueMesh.V));
}

double rai::Mesh::getCircum() const {
  if(!T.N) return 0.;
  CHECK_EQ(T.d1, 2,"");
  double A=0.;
  for(uint i=0; i<T.d0; i++) A += length(V[T(i,0)] - V[T(i,1)]);
  return A;
}

double rai::Mesh::getCircum(uint i) const {
  if(!T.N) return 0.;
  CHECK_EQ(T.d1, 3,"");
  double A=0.;
  A += length(V[T(i,0)] - V[T(i,1)]);
  A += length(V[T(i,1)] - V[T(i,2)]);
  A += length(V[T(i,2)] - V[T(i,0)]);
  return A;
}

void rai::Mesh::write(std::ostream& os) const {
  os <<"Mesh: " <<V.d0 <<" vertices, " <<T.d0 <<" triangles" <<endl;
}

void rai::Mesh::readFile(const char* filename) {
  const char* fileExtension = filename+(strlen(filename)-3);
//  if(!strcmp(fileExtension, "obj")) { *this = mesh_readAssimp(filename); } else
  if(!strcmp(fileExtension, "dae") || !strcmp(fileExtension, "DAE")) { *this = AssimpLoader(filename).getSingleMesh(); }
  else read(FILE(filename).getIs(), fileExtension, filename);
}

void rai::Mesh::read(std::istream& is, const char* fileExtension, const char* filename) {
  bool loaded=false;
  if(!strcmp(fileExtension, "obj")) { readObjFile(is); loaded=true; }
  if(!strcmp(fileExtension, "off")) { readOffFile(is); loaded=true; }
  if(!strcmp(fileExtension, "ply")) { readPLY(filename); loaded=true; }
  if(!strcmp(fileExtension, "tri")) { readTriFile(is); loaded=true; }
  if(!strcmp(fileExtension, "arr")) { readArr(is); loaded=true; }
  if(!strcmp(fileExtension, "stl") || !strcmp(fileExtension, "STL")) { loaded = readStlFile(is); }
  if(!strcmp(fileExtension, "dae") || !strcmp(fileExtension, "DAE")) { *this = AssimpLoader(filename).getSingleMesh(); loaded=true; }
  if(!loaded) HALT("can't read fileExtension '" <<fileExtension <<"' file '" <<filename <<"'");
}

void rai::Mesh::writeTriFile(const char* filename) {
  ofstream os;
  rai::open(os, filename);
  os <<"TRI" <<endl <<endl
     <<V.d0 <<endl
     <<T.d0 <<endl <<endl;
     
  V.write(os, " ", "\n ", "  ");
  os <<endl <<endl;
  T.write(os, " ", "\n ", "  ");
}

void rai::Mesh::readTriFile(std::istream& is) {
  uint i, nV, nT;
  is >>PARSE("TRI") >>nV >>nT;
  V.resize(nV, 3);
  T.resize(nT, 3);
  for(i=0; i<V.N; i++) is >>V.elem(i);
  for(i=0; i<T.N; i++) is >>T.elem(i);
}

void rai::Mesh::writeOffFile(const char* filename) {
  ofstream os;
  rai::open(os, filename);
  uint i;
  os <<"OFF\n" <<V.d0 <<' ' <<T.d0 <<' ' <<0 <<endl;
  for(i=0; i<V.d0; i++) os <<V(i, 0) <<' ' <<V(i, 1) <<' ' <<V(i, 2) <<endl;
  for(i=0; i<T.d0; i++) os <<3 <<' ' <<T(i, 0) <<' ' <<T(i, 1) <<' ' <<T(i, 2) <<endl;
}

void rai::Mesh::readOffFile(std::istream& is) {
  uint i, k, nVertices, nFaces, nEdges, alpha;
  bool color;
  rai::String tag;
  is >>tag;
  if(tag=="OFF") color=false;
  else if(tag=="COFF") color=true;
  else HALT("");
  is >>nVertices >>nFaces >>nEdges;
  CHECK(!nEdges, "can't read edges in off file");
  V.resize(nVertices, 3);
  T.resize(nFaces   , 3);
  if(color) C.resize(nVertices, 3);
  for(i=0; i<V.d0; i++) {
    is >>V(i, 0) >>V(i, 1) >>V(i, 2);
    if(color) is >>C(i,0) >>C(i,1) >>C(i,2) >>alpha;
  }
  for(i=0; i<T.d0; i++) {
    is >>k;
    CHECK_EQ(k,3, "can only read triangles from OFF");
    is >>T(i, 0) >>T(i, 1) >>T(i, 2);
  }
}

void rai::Mesh::readPlyFile(std::istream& is) {
  uint i, k, nVertices, nFaces;
  rai::String str;
  is >>PARSE("ply") >>PARSE("format") >>str;
  if(str=="ascii") {
    is >>PARSE("1.0");
    is >>PARSE("element vertex") >>nVertices;
    is >>PARSE("property float32 x") >>PARSE("property float32 y") >>PARSE("property float32 z");
    is >>PARSE("property float32 nx") >>PARSE("property float32 ny") >>PARSE("property float32 nz");
    is >>PARSE("element face") >>nFaces;
    is >>PARSE("property list uint8 int32 vertex_indices") >>PARSE("end_header");
    V.resize(nVertices, 3);
    T.resize(nFaces   , 3);
    double nx, ny, nz;
    for(i=0; i<V.d0; i++) {
      is >>V(i, 0) >>V(i, 1) >>V(i, 2) >>nx >>ny >>nz;
    }
    for(i=0; i<T.d0; i++) {
      is >>k >>T(i, 0) >>T(i, 1) >>T(i, 2);
      CHECK_EQ(k,3, "can only read triangles from ply");
    }
  }
}

#ifdef RAI_extern_ply
void rai::Mesh::writePLY(const char *fn, bool bin) {
  struct PlyFace { unsigned char nverts;  int *verts; };
  struct Vertex { float x,  y,  z ;  };
  uint _nverts = V.d0;
  floatA Vfloat; copy(Vfloat, V);
  Vertex *_vertices  = (Vertex*) Vfloat.p;
  
  PlyProperty vert_props[]  = { /* list of property information for a PlyVertex */
    {"x", Float32, Float32, offsetof(Vertex,x), 0, 0, 0, 0},
    {"y", Float32, Float32, offsetof(Vertex,y), 0, 0, 0, 0},
    {"z", Float32, Float32, offsetof(Vertex,z), 0, 0, 0, 0}
//    {"nx", Float64, Float64, offsetof( Vertex,nx ), 0, 0, 0, 0},
//    {"ny", Float64, Float64, offsetof( Vertex,ny ), 0, 0, 0, 0},
//    {"nz", Float64, Float64, offsetof( Vertex,nz ), 0, 0, 0, 0}
  };
  
  PlyProperty face_props[]  = { /* list of property information for a PlyFace */
    {"vertex_indices", Int32, Int32, offsetof(PlyFace,verts), 1, Uint8, Uint8, offsetof(PlyFace,nverts)},
  };
  
  PlyFile    *ply;
  FILE       *fp = fopen(fn, "w");
  
  const char  *elem_names[]  = { "vertex", "face" };
  ply = write_ply(fp, 2, elem_names, bin? PLY_BINARY_LE : PLY_ASCII);
  
  /* describe what properties go into the PlyVertex elements */
  describe_element_ply(ply, "vertex", _nverts);
  describe_property_ply(ply, &vert_props[0]);
  describe_property_ply(ply, &vert_props[1]);
  describe_property_ply(ply, &vert_props[2]);
//  describe_property_ply(ply, &vert_props[3]);
//  describe_property_ply(ply, &vert_props[4]);
//  describe_property_ply(ply, &vert_props[5]);

  /* describe PlyFace properties (just list of PlyVertex indices) */
  describe_element_ply(ply, "face", T.d0);
  describe_property_ply(ply, &face_props[0]);
  
  header_complete_ply(ply);
  
  //-- put vertices
  put_element_setup_ply(ply, "vertex");
  for(uint i = 0; i < _nverts; i++)  put_element_ply(ply, (void *) &(_vertices[i]));
  
  //-- put tris
  put_element_setup_ply(ply, "face");
  int verts[3] ;
  PlyFace     face ;
  face.nverts = 3 ;
  face.verts  = verts ;
  for(uint i = 0; i < T.d0; i++) {
    face.verts[0] = T(i,0);
    face.verts[1] = T(i,1);
    face.verts[2] = T(i,2);
    put_element_ply(ply, (void *) &face);
  }
  
  close_ply(ply); //calls fclose
  free_ply(ply);
}

bool ply_check_property( PlyElement *elem, const char *prop_name){
  for(int i=0; i<elem->nprops; i++)
    if(!strcmp(prop_name, elem->props[i]->name)) return true;
  return false;
}

void rai::Mesh::readPLY(const char *fn) {
  struct PlyFace {    unsigned char nverts;  int *verts; };
  struct Vertex {    double x,  y,  z ;  byte r,g,b; };
  uint _nverts=0, _ntrigs=0;
  
  PlyProperty vert_props[]  = { /* list of property information for a PlyVertex */
    {"x", Float64, Float64, offsetof(Vertex,x), 0, 0, 0, 0},
    {"y", Float64, Float64, offsetof(Vertex,y), 0, 0, 0, 0},
    {"z", Float64, Float64, offsetof(Vertex,z), 0, 0, 0, 0},
    //    {"nx", Float64, Float64, offsetof( Vertex,nx ), 0, 0, 0, 0},
    //    {"ny", Float64, Float64, offsetof( Vertex,ny ), 0, 0, 0, 0},
    //    {"nz", Float64, Float64, offsetof( Vertex,nz ), 0, 0, 0, 0}
    {"red", Uint8, Uint8, offsetof(Vertex,r), 0,0,0,0},
    {"green", Uint8, Uint8, offsetof(Vertex,g), 0,0,0,0},
    {"blue", Uint8, Uint8, offsetof(Vertex,b), 0,0,0,0}
  };
  
  PlyProperty face_props[]  = { /* list of property information for a PlyFace */
    {"vertex_indices", Int32, Int32, offsetof(PlyFace,verts), 1, Uint8, Uint8, offsetof(PlyFace,nverts)},
  };
  
  FILE    *fp  = fopen(fn, "r");
  CHECK(fp, "coult not open file " <<fn <<" from path "<<getcwd_string())
  PlyFile *ply = read_ply(fp);
  
  //-- get the number of faces and vertices
  for(uint i = 0; i < (uint)ply->num_elem_types; ++i) {
    int elem_count ;
    char *elem_name = setup_element_read_ply(ply, i, &elem_count);
    if(equal_strings("vertex", elem_name)) _nverts = elem_count;
    if(equal_strings("face",   elem_name)) _ntrigs = elem_count;
  }
  V.resize(_nverts,3);
  T.resize(_ntrigs,3);

  //-- examine each element type that is in the file (PlyVertex, PlyFace)
  for(int i = 0; i < ply->num_elem_types; ++i)  {
    int elem_count ;
    char *elem_name = setup_element_read_ply(ply, i, &elem_count);
    
    if(equal_strings("vertex", elem_name))   {
      /* set up for getting PlyVertex elements */
      int r=1;
      r &= setup_property_ply(ply, &vert_props[0]);
      r &= setup_property_ply(ply, &vert_props[1]);
      r &= setup_property_ply(ply, &vert_props[2]);
      if(!r) HALT("no vertices defined??");
      if(ply_check_property(ply->which_elem, "red")){
        r &= setup_property_ply(ply, &vert_props[3]);
        r &= setup_property_ply(ply, &vert_props[4]);
        r &= setup_property_ply(ply, &vert_props[5]);
        if(r && C.N!=V.N) C.resize(_nverts,3); //has color
      }

      Vertex vertex;
      for(uint j = 0; j < _nverts; ++j) {
        get_element_ply(ply, &vertex);
        V(j,0) = vertex.x;
        V(j,1) = vertex.y;
        V(j,2) = vertex.z;
        if(C.N==V.N){
          C(j,0) = vertex.r;
          C(j,1) = vertex.g;
          C(j,2) = vertex.b;
        }
      }
    } else if(equal_strings("face", elem_name))  {
      /* set up for getting PlyFace elements */
      /* (all we need are PlyVertex indices) */
      setup_property_ply(ply, &face_props[0]) ;
      PlyFace     face ;
      for(uint j = 0; j < _ntrigs; ++j)   {
        get_element_ply(ply, (void *) &face);
        if(face.nverts != 3)
          HALT("not a triangulated surface: polygon " <<j <<" has " <<face.nverts <<" sides") ;
          
        T(j,0) = face.verts[0];
        T(j,1) = face.verts[1];
        T(j,2) = face.verts[2];
        
        free(face.verts) ;
      }
    } else /* all non-PlyVertex and non-PlyFace elements are grabbed here */
      get_other_element_ply(ply);
  }
  
  close_ply(ply); //calls fclose
  free_ply(ply);
}

#else
void rai::Mesh::writePLY(const char *fn, bool bin) { NICO }
void rai::Mesh::readPLY(const char *fn) { NICO }
#endif

void rai::Mesh::writeArr(std::ostream& os) {
  V.writeTagged(os, "V", true);
  T.writeTagged(os, "T", true);
  C.writeTagged(os, "C", true);
  tex.writeTagged(os, "tex", true);
  texImg.writeTagged(os, "texImg", true);
}

void rai::Mesh::readArr(std::istream& is) {
  V.readTagged(is, "V");
  T.readTagged(is, "T");
  C.readTagged(is, "C");
  tex.readTagged(is, "tex");
  texImg.readTagged(is, "texImg");
}

bool rai::Mesh::readStlFile(std::istream& is) {
  //first check if binary
  if(rai::parse(is, "solid", true)) { //is ascii
    rai::String name;
    is >>name;
    uint i, k=0, k0;
    double x, y, z;
//    cout <<"reading STL file -- object name '" <<name <<"'..." <<endl;
    V.resize(10000);
    //1st pass
    for(i=0, k=0;; i++) {
      k0=k;
      if(k>V.N-10) V.resizeCopy(2*V.N);
      if(!(i%100)) cout <<"\r" <<i <<' ' <<i*7;
      if(rai::peerNextChar(is)!='f') break;
      is >>PARSE("facet");
      is >>PARSE("normal") >>x >>y >>z;  rai::skip(is);
      is >>PARSE("outer") >>PARSE("loop");      rai::skip(is);
      is >>PARSE("vertex")>>V(k++); is>>V(k++); is>>V(k++);   rai::skip(is);
      is >>PARSE("vertex")>>V(k++); is>>V(k++); is>>V(k++);   rai::skip(is);
      is >>PARSE("vertex")>>V(k++); is>>V(k++); is>>V(k++);   rai::skip(is);
      is >>PARSE("endloop");             rai::skip(is);
      is >>PARSE("endfacet");            rai::skip(is);
      if(!is.good()) {
        RAI_MSG("reading error - skipping facet " <<i <<" (line " <<i*7+2 <<")");
        is.clear();
        cout <<1 <<endl;
        rai::skipUntil(is, "endfacet");
        cout <<2 <<endl;
        k=k0;
      }
    }
    is >>PARSE("endsolid");
    if(!is.good()) RAI_MSG("couldn't read STL end tag (line" <<i*7+2);
    cout <<"... STL file read: #tris=" <<i <<" #lines=" <<i*7+2 <<endl;
    CHECK(!(k%9), "not mod 9..");
    V.resizeCopy(k/3, 3);
    T.resize(k/9, 3);
    for(i=0; i<T.N; i++) { T.elem(i)=i; }
  } else { //is binary
    is.clear();
    is.seekg(0, std::ios::beg);
    char header[80];
    is.read(header, 80);
    uint ntri;
    is.read((char*)&ntri, sizeof(ntri));
    T.resize(ntri,3);
    floatA Vfloat(3*ntri,3);
    float normal[3];
    uint16_t att;
    for(uint i=0; i<ntri; i++) {
      is.read((char*)&normal, 3*Vfloat.sizeT);
      is.read((char*)&Vfloat(3*i,0), 9*Vfloat.sizeT);
      T(i,0)=3*i+0;  T(i,1)=3*i+1;  T(i,2)=3*i+2;
      is.read((char*)&att, 2);
      CHECK_EQ(att,0,"this stl file is broke");
    }
    copy(V,Vfloat);
  }
  return true;
}

/*void rai::Mesh::getOBJ(char* filename){
  if(!glm){
  glm = glmReadOBJ(filename);
  glmReverseWinding(glm);
  }

  ////glmUnitize(glm);
  glmFacetNormals(glm);
  glmVertexNormals(glm, 90.0);

  // creates a display list for the OBJ
  ////  g._pmodel_displaylist = glmList(glm, GLM_SMOOTH | GLM_MATERIAL);
  }*/

uint& Tni(uint, uint) { static uint dummy; return dummy; } //normal index

rai::String str;

char *strn(std::istream& is) {
  str.read(is," \n\t\r"," \n\t\r",true); //we once had a character '\d' in there -- for Windows?
  CHECK(is.good(),"could not read line");
  return str.p;
}

/** initialises the ascii-obj file "filename"*/
void rai::Mesh::readObjFile(std::istream& is) {
  // make a first pass through the file to get a count of the number
  // of vertices, normals, texcoords & triangles
  uint nV, nN, nTex, nT;
  nV = nN = nTex = nT = 0;
  int v, n, t;
  
//  // we only want to parse the relevant subpart/submesh of the mesh therefore
//  // jump to the right position and stop parsing at the right positon.
//  if (parsing_pos_start > -1)
//    is.seekg(parsing_pos_start); //  fseek(file, parsing_pos_start, SEEK_SET);

//  while ((sscanf(strn(is), "%s", str.p) != EOF) && (ftell(file) < parsing_pos_end)) {
  strn(is);
  for(bool ex=false; !ex;) {
//    if(parsing_pos_start>-1 && is.tellg()>=parsing_pos_end) break;
    switch(str.p[0]) {
      case '\0':
        is.clear();
        ex=true;
        break; //EOF
      case '#':
        rai::skipRestOfLine(is);
        strn(is);
        break;
      case 'v':
        switch(str.p[1]) {
          case '\0': nV++;    rai::skipRestOfLine(is); break;  // vertex
          case 'n':  nN++;    rai::skipRestOfLine(is); break;  // normal
          case 't':  nTex++;  rai::skipRestOfLine(is); break;  // texcoord
          default: HALT("firstPass(): Unknown token '" <<str.p <<"'");  break;
        }
        strn(is);
        break;
      case 'f':               // face
        v = n = t = 0;
        strn(is);
        // can be one of %d, %d//%d, %d/%d, %d/%d/%d %d//%d
        if(strstr(str.p, "//")) {
          // v//n
          CHECK(sscanf(str.p   , "%d//%d", &v, &n), "fscan failed");
          CHECK(sscanf(strn(is), "%d//%d", &v, &n), "fscan failed");
          CHECK(sscanf(strn(is), "%d//%d", &v, &n), "fscan failed");
          nT++;
          while(sscanf(strn(is), "%d//%d", &v, &n) > 0) nT++;
        } else if(sscanf(str.p, "%d/%d/%d", &v, &t, &n) == 3) {
          // v/t/n
          CHECK(sscanf(strn(is), "%d/%d/%d", &v, &t, &n), "fscan failed");
          CHECK(sscanf(strn(is), "%d/%d/%d", &v, &t, &n), "fscan failed");
          nT++;
          while(sscanf(strn(is), "%d/%d/%d", &v, &t, &n) > 0) nT++;
        } else if(sscanf(str.p, "%d/%d", &v, &t) == 2) {
          // v/t
          CHECK(sscanf(strn(is), "%d/%d", &v, &t), "fscan failed");
          CHECK(sscanf(strn(is), "%d/%d", &v, &t), "fscan failed");
          nT++;
          while(sscanf(strn(is), "%d/%d", &v, &t) > 0) nT++;
        } else {
          // v
          CHECK(sscanf(strn(is), "%d", &v), "fscan failed");
          CHECK(sscanf(strn(is), "%d", &v), "fscan failed");
          nT++;
          while(sscanf(strn(is), "%d", &v) > 0) nT++;
        }
        break;
        
      default:  RAI_MSG("unsupported .obj file tag '" <<str <<"'");  rai::skipRestOfLine(is);  strn(is);  break;
    }
  }
  
  //allocate memory
  V.resize(nV, 3);
  Vn.resize(nN, 3);
  T.resize(nT, 3);
  Tn.resize(nT, 3);
  if(nTex) Tt.resize(nT, 3);
  //if(nVN) N.resize(nVN, 3);
  if(nTex) tex.resize(nTex, 2);
  
  // rewind to beginning of file and read in the data this pass
  is.seekg(0);
  is.clear();
//  if (parsing_pos_start > -1)
//    is.seekg(parsing_pos_start); //  fseek(file, parsing_pos_start, SEEK_SET);

  /* on the second pass through the file, read all the data into the
     allocated arrays */
  nV = nN = nTex = nT = 0;
  ////_material = 0;
  
//  while ((sscanf(strn(is), "%s", str.p) != EOF) && (ftell(file) < parsing_pos_end)) {
  strn(is);
  for(bool ex=false; !ex;) {
//    if(parsing_pos_start>-1 && is.tellg()>=parsing_pos_end) break;
    switch(str.p[0]) {
      case '\0':
        is.clear();
        ex=true;
        break; //EOF
      case '#':
        rai::skipRestOfLine(is);
        strn(is);
        break;  //comment
      case 'v':               // v, vn, vt
        switch(str.p[1]) {
          case '\0': is >>V(nV, 0) >>V(nV, 1) >> V(nV, 2);  nV++;  break;  //vertex
          case 'n':  is >>Vn(nN, 0) >>Vn(nN, 1) >>Vn(nN, 2);  nN++;  break;  //normal
          case 't':  is >>tex(nTex, 0) >>tex(nTex, 1);   nTex++;  break;  //texcoord
        }
        strn(is);
        break;
      case 'f':               // face
        v = n = t = 0;
        strn(is);
        if(strstr(str.p, "//")) {
          // v//vn
          sscanf(str.p, "%d//%d", &v, &n);
          
          T(nT, 0) = v < 0 ? v + nV : v;
          Tni(nT, 0) = n < 0 ? n + nN : n;
          CHECK(sscanf(strn(is), "%d//%d", &v, &n), "fscan failed");
          T(nT, 1) = v < 0 ? v + nV : v;
          Tni(nT, 1) = n < 0 ? n + nN : n;
          CHECK(sscanf(strn(is), "%d//%d", &v, &n), "fscan failed");
          T(nT, 2) = v < 0 ? v + nV : v;
          Tni(nT, 2) = n < 0 ? n + nN : n;
          //// group->triangles[group->nT++] = nT;
          nT++;
          while(sscanf(strn(is), "%d//%d", &v, &n) > 0) {
            T(nT, 0) = T(nT-1, 0);
            Tni(nT, 0) = Tni(nT-1, 0);
            T(nT, 1) = T(nT-1, 2);
            Tni(nT, 1) = Tni(nT-1, 2);
            T(nT, 2) = v < 0 ? v + nV : v;
            Tni(nT, 2) = n < 0 ? n + nN : n;
            //// group->triangles[group->numtriangles++] = numtriangles;
            nT++;
          }
        } else if(sscanf(str.p, "%d/%d/%d", &v, &t, &n) == 3) {
          // v/vt/vn
          T(nT, 0) = v < 0 ? v + nV : v;
          Tt(nT, 0) = t < 0 ? t + nTex : t;
          Tni(nT, 0) = n < 0 ? n + nN : n;
          CHECK(sscanf(strn(is), "%d/%d/%d", &v, &t, &n), "fscan failed");
          T(nT, 1) = v < 0 ? v + nV : v;
          Tt(nT, 1) = t < 0 ? t + nTex : t;
          Tni(nT, 1) = n < 0 ? n + nN : n;
          CHECK(sscanf(strn(is), "%d/%d/%d", &v, &t, &n), "fscan failed");
          T(nT, 2) = v < 0 ? v + nV : v;
          Tt(nT, 2) = t < 0 ? t + nTex : t;
          Tni(nT, 2) = n < 0 ? n + nN : n;
          //// group->triangles[group->numtriangles++] = numtriangles;
          nT++;
          while(sscanf(strn(is), "%d/%d/%d", &v, &t, &n) > 0) {
            T(nT, 0) = T(nT-1, 0);  Tt(nT, 0) = Tt(nT-1, 0);  Tni(nT, 0) = Tni(nT-1, 0);
            T(nT, 1) = T(nT-1, 2);  Tt(nT, 1) = Tt(nT-1, 2);  Tni(nT, 1) = Tni(nT-1, 2);
            T(nT, 2) = v < 0 ? v + nV : v;
            Tt(nT, 2) = t < 0 ? t + nTex : t;
            Tni(nT, 2) = n < 0 ? n + nN : n;
            //// group->triangles[group->numtriangles++] = numtriangles;
            nT++;
          }
        } else if(sscanf(str.p, "%d/%d", &v, &t) == 2) {
          // v/vt
          T(nT, 0) = v < 0 ? v + nV : v;
          Tt(nT, 0) = t < 0 ? t + nTex : t;
          CHECK(sscanf(strn(is), "%d/%d", &v, &t), "fscan failed");
          T(nT, 1) = v < 0 ? v + nV : v;
          Tt(nT, 1) = t < 0 ? t + nTex : t;
          CHECK(sscanf(strn(is), "%d/%d", &v, &t), "fscan failed");
          T(nT, 2) = v < 0 ? v + nV : v;
          Tt(nT, 2) = t < 0 ? t + nTex : t;
          //// group->triangles[group->numtriangles++] = numtriangles;
          nT++;
          while(sscanf(strn(is), "%d/%d", &v, &t) > 0) {
            T(nT, 0) = T(nT-1, 0);  Tt(nT, 0) = Tt(nT-1, 0);  T(nT, 1) = T(nT-1, 2);
            Tt(nT, 1) = Tt(nT-1, 2);  T(nT, 2) = v < 0 ? v + nV : v;  Tt(nT, 2) = t < 0 ? t + nTex : t;
            //// group->triangles[group->numtriangles++] = numtriangles;
            nT++;
          }
        } else {
          // v
          sscanf(str.p, "%d", &v);
          T(nT, 0) = v < 0 ? v + nV : v;
          CHECK(sscanf(strn(is), "%d", &v), "fscan failed");
          T(nT, 1) = v < 0 ? v + nV : v;
          CHECK(sscanf(strn(is), "%d", &v), "fscan failed");
          T(nT, 2) = v < 0 ? v + nV : v;
          //// group->triangles[group->numtriangles++] = nT;
          nT++;
          while(sscanf(strn(is), "%d", &v) > 0) {
            T(nT, 0) = T(nT-1, 0);
            T(nT, 1) = T(nT-1, 2);
            T(nT, 2) = v < 0 ? v + nV : v;
            //// group->triangles[group->numtriangles++] = numtriangles;
            nT++;
          }
        }
        break;
        
      default:  rai::skipRestOfLine(is);  strn(is);  break;
    }
  }
  
  //CONVENTION!: start counting vertex indices from 0!!
  T -= 1u;
  CHECK(T.max() < nV, "");
  if(nTex) {
    Tt -= 1u;
    CHECK(Tt.max() < nTex, "");
  }
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
  CHECK(rai::String(filename).endsWith("obj"),
        "getSubMeshPositions parses only obj files.");
  FILE* file;
  char buf[128];
  file = fopen(filename, "r");
  CHECK(file,
        "can't open data file " << filename << "; cwd is " << getcwd_string());
        
  int flag = 0;
  long start_pos = 0;
  long end_pos = 0;
  
  uintA result;
  while(fscanf(file, "%s", buf) != EOF) {
    switch(buf[0]) {
      case 'v': {
        if(flag > 0) {
          end_pos = ftell(file) - 1;
          result.append(TUP((uint)start_pos, (uint)end_pos));
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
  result.append(TUP((uint)start_pos, (uint)end_pos));
  result.reshape(result.N/2,2);
  return result;
}

#ifdef RAI_GL

extern void glColor(float r, float g, float b, float alpha);

/// GL routine to draw a rai::Mesh
void rai::Mesh::glDraw(struct OpenGL& gl) {
  if(Geo_mesh_drawColors) {
    if(C.nd==1) {
      CHECK(C.N==3 || C.N==4, "need a basic color");
      GLboolean light=true;
      glGetBooleanv(GL_LIGHTING, &light);
      GLfloat col[4] = { (float)C(0), (float)C(1), (float)C(2), (C.N==3?1.f:(float)C(3)) };
      glColor4fv(col);
      if(light) glMaterialfv(GL_FRONT_AND_BACK, GL_AMBIENT_AND_DIFFUSE, col);
    }
  }
  
  if(!T.N) { //-- draw point cloud
    if(!V.N) return;
    CHECK(V.nd==2 && V.d1==3, "wrong dimension");
//    glPointSize(3.);
    glDisable(GL_LIGHTING);
    
    glEnableClientState(GL_VERTEX_ARRAY);
    if(C.N==V.N) glEnableClientState(GL_COLOR_ARRAY); else glDisableClientState(GL_COLOR_ARRAY);
    
    glVertexPointer(3, GL_DOUBLE, V.d1-3, V.p);
    if(C.N==V.N) glColorPointer(3, GL_DOUBLE, C.d1-3, C.p);
    
    glDrawArrays(GL_POINTS, 0, V.d0);
    glDisableClientState(GL_VERTEX_ARRAY);
    
    glEnable(GL_LIGHTING);
//    glPointSize(1.);
    return;
  }
  
  if(T.d1==2) { //-- draw lines
//    glLineWidth(3.f);
    glBegin(GL_LINES);
    for(uint t=0; t<T.d0; t++) {
      glVertex3dv(&V(T(t, 0), 0));
      glVertex3dv(&V(T(t, 1), 0));
    }
    glEnd();
    return;
  }
  
  //-- draw a mesh
  if(V.d0!=Vn.d0 || T.d0!=Tn.d0) computeNormals();
  
  //-- if not yet done, GenTexture
  if(texImg.N && Geo_mesh_drawColors){
    if(texture<0) {
      GLuint texName;
      glGenTextures(1, &texName);
      texture = texName;
      glBindTexture(GL_TEXTURE_2D, texture);
    
      if(texImg.d2==4) glTexImage2D(GL_TEXTURE_2D, 0, GL_RGBA, texImg.d1, texImg.d0, 0, GL_RGBA, GL_UNSIGNED_BYTE, texImg.p);
      else if(texImg.d2==3) glTexImage2D(GL_TEXTURE_2D, 0, GL_RGB, texImg.d1, texImg.d0, 0, GL_RGB, GL_UNSIGNED_BYTE, texImg.p);
      else NIY;
      glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_CLAMP_TO_EDGE);
      glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_CLAMP_TO_EDGE);
      glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR);
      glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR);
    } else {
      glBindTexture(GL_TEXTURE_2D, texture);
    }
  }
  
  //-- draw the mesh
  if((!C.N || C.nd==1 || C.d0==V.d0)  //we have colors for each vertex
      && (!tex.N || !Tt.N)) { //we have no tex or tex coords for each vertex -> use index arrays
      
    //  glShadeModel(GL_FLAT);
    glShadeModel(GL_SMOOTH);
    glEnableClientState(GL_VERTEX_ARRAY);
    if(Geo_mesh_drawColors){
      if(tex.N) CHECK_EQ(tex.d0, V.d0, "this needs tex coords for each vertex; if you have it face wise, render the slow way..");
      if(tex.N) glEnable(GL_TEXTURE_2D);

      glEnableClientState(GL_NORMAL_ARRAY);
      if(C.N==V.N) glEnableClientState(GL_COLOR_ARRAY); else glDisableClientState(GL_COLOR_ARRAY);
      if(C.N==V.N) glDisable(GL_LIGHTING); //because lighting requires ambiance colors to be set..., not just color..
      if(tex.N) glEnableClientState(GL_TEXTURE_COORD_ARRAY); else glDisableClientState(GL_TEXTURE_COORD_ARRAY);
    }
    
    glVertexPointer(3, GL_DOUBLE, 0, V.p);
    if(Geo_mesh_drawColors){
      glNormalPointer(GL_DOUBLE, 0, Vn.p);
      if(C.N==V.N) glColorPointer(3, GL_DOUBLE, 0, C.p);
      if(tex.N) glTexCoordPointer(2, GL_DOUBLE, 0, tex.p);
    }
    
    glDrawElements(GL_TRIANGLES, T.N, GL_UNSIGNED_INT, T.p);
    
    if(C.N) glEnable(GL_LIGHTING);
    
    if(tex.N) glDisable(GL_TEXTURE_2D);
    
//  } else if(C.d0==T.d0){ //we have colors for each tri -> render tris directly and with tri-normals

//    CHECK_EQ(C.d0, T.d0, "");
//    CHECK_EQ(Tn.d0, T.d0, "");
//    glShadeModel(GL_FLAT);
//    glBegin(GL_TRIANGLES);
//    GLboolean light=true;
//    glGetBooleanv(GL_LIGHTING, &light); //this doesn't work!!?? even when disabled, returns true; never changes 'light'
//    for(uint t=0; t<T.d0; t++) {
//      uint   *tri  = T.p  + 3*t; //&T(t, 0);
//      double *col  = C.p  + 3*t; //&C(t, 0);
//      double *norm = Tn.p + 3*t; //&Tn(t, 0);

//      GLfloat ambient[4] = { (float)col[0], (float)col[1], (float)col[2], 1.f };
//      if(!light) glColor4fv(ambient);
//      else       glMaterialfv(GL_FRONT_AND_BACK, GL_AMBIENT_AND_DIFFUSE, ambient);

//      glNormal3dv(norm);
//      glVertex3dv(V.p + 3*tri[0]); //&V(tri[0],0);
//      glVertex3dv(V.p + 3*tri[1]);
//      glVertex3dv(V.p + 3*tri[2]);
//    }
//    glEnd();
  } else { //basic vertex-wise
    uint i, v;
    
    if(tex.N) CHECK_EQ(Tt.d0, T.d0, "this needs tex coords for each tri");
    if(tex.N && Geo_mesh_drawColors) glEnable(GL_TEXTURE_2D);
    
    glBegin(GL_TRIANGLES);
    for(i=0; i<T.d0; i++) {
      if(C.d0==T.d0) {
        if(C.d1==3) glColor(C(i, 0), C(i, 1), C(i, 2), 1.);
        if(C.d1==1) glColorId(C(i,0));
      }
      v=T(i, 0);  glNormal3dv(&Vn(v, 0));  if(C.d0==V.d0) glColor3dv(&C(v, 0));  if(Tt.N) glTexCoord2dv(&tex(Tt(i, 0), 0));  glVertex3dv(&V(v, 0));
      v=T(i, 1);  glNormal3dv(&Vn(v, 0));  if(C.d0==V.d0) glColor3dv(&C(v, 0));  if(Tt.N) glTexCoord2dv(&tex(Tt(i, 1), 0));  glVertex3dv(&V(v, 0));
      v=T(i, 2);  glNormal3dv(&Vn(v, 0));  if(C.d0==V.d0) glColor3dv(&C(v, 0));  if(Tt.N) glTexCoord2dv(&tex(Tt(i, 2), 0));  glVertex3dv(&V(v, 0));
    }
    glEnd();
    if(Tt.N && texImg.N &&  Geo_mesh_drawColors) {
      glDisable(GL_TEXTURE_2D);
      glEnable(GL_LIGHTING);
    }
#if 0 //draw normals //simple with triangle normals
    glColor(.5, 1., .0);
    Vector a, b, c, x;
    for(i=0; i<T.d0; i++) {
      glBegin(GL_LINES);
      a.set(&V(T(i, 0), 0)); b.set(&V(T(i, 1), 0)); c.set(&V(T(i, 2), 0));
      x.setZero(); x+=a; x+=b; x+=c; x/=3;
      glVertex3dv(x.v);
      a.set(&Tn(i, 0));
      x+=.05*a;
      glVertex3dv(x.v);
      glEnd();
    }
#endif
  }
  
  if(orsDrawWires) { //on top of mesh
#if 0
    uint t;
    for(t=0; t<T.d0; t++) {
      glBegin(GL_LINE_LOOP);
      glVertex3dv(&V(T(t, 0), 0));
      glVertex3dv(&V(T(t, 1), 0));
      glVertex3dv(&V(T(t, 2), 0));
      glEnd();
    }
#else
    glColor(0.,0.,0.,1.);
    glEnableClientState(GL_VERTEX_ARRAY);
    glDisableClientState(GL_NORMAL_ARRAY);
    if(C.N==V.N) glEnableClientState(GL_COLOR_ARRAY); else glDisableClientState(GL_COLOR_ARRAY);
    
    glVertexPointer(3, GL_DOUBLE, 0, V.p);
    if(C.N==V.N) glColorPointer(3, GL_DOUBLE, 0, C.p);
    glDrawElements(GL_LINE_STRIP, T.N, GL_UNSIGNED_INT, T.p);
#endif
  }
  
}

#else //RAI_GL

void rai::Mesh::glDraw(struct OpenGL&) { NICO }
void glDrawMesh(void*) { NICO }
void glTransform(const rai::Transformation&) { NICO }
#endif

//==============================================================================

extern OpenGL& NoOpenGL;

void glDrawMeshes(void *P, OpenGL&) {
#ifdef RAI_GL
  MeshA& meshes = *((MeshA*)P);
  double GLmatrix[16];
  for(rai::Mesh& mesh:meshes) {
    glPushMatrix();
    mesh.glX.getAffineMatrixGL(GLmatrix);
    glLoadMatrixd(GLmatrix);
    mesh.glDraw(NoOpenGL);
    glPopMatrix();
  }
#endif
}

void rai::MeshCollection::glDraw(OpenGL& gl){
  CHECK_EQ(X.nd, 2, "");
  CHECK_EQ(X.d0, M.N, "");
  CHECK_EQ(X.d1, 7, "");

  double GLmatrix[16];
  rai::Transformation t;
  for(uint i=0;i<M.N;i++) {
    glPushMatrix();
    t.set(&X(i,0));
    glLoadMatrixd(t.getAffineMatrixGL(GLmatrix));
    M(i)->glDraw(gl);
    glPopMatrix();
  }
}

//==============================================================================

void inertiaSphere(double *I, double& mass, double density, double radius) {
  double r2=radius*radius;
  if(density) mass=density*4./3.*RAI_PI*r2*radius;
  I[1]=I[2]=I[3]=I[5]=I[6]=I[7]=0.;
  I[0]=.4*mass*r2;
  I[4]=.4*mass*r2;
  I[8]=.4*mass*r2;
}

void inertiaBox(double *I, double& mass, double density, double dx, double dy, double dz) {
  if(density) mass=density*dx*dy*dz;
  I[1]=I[2]=I[3]=I[5]=I[6]=I[7]=0.;
  double x2=dx*dx, y2=dy*dy, z2=dz*dz;
  I[0]=mass/12.*(y2+z2);
  I[4]=mass/12.*(x2+z2);
  I[8]=mass/12.*(x2+y2);
}

void inertiaCylinder(double *I, double& mass, double density, double height, double radius) {
  double r2=radius*radius, h2=height*height;
  if(density) mass=density*RAI_PI*r2*height;
  I[1]=I[2]=I[3]=I[5]=I[6]=I[7]=0.;
  I[0]=mass/12.*(3.*r2+h2);
  I[4]=mass/12.*(3.*r2+h2);
  I[8]=mass/2.*r2;
}

//===========================================================================
//
// GJK interface
//

#ifdef RAI_extern_GJK
GJK_point_type& NoPointType = *((GJK_point_type*)NULL);
template<> const char* rai::Enum<GJK_point_type>::names []= { "GJK_none", "GJK_vertex", "GJK_edge", "GJK_face", NULL };
double GJK_sqrDistance(const rai::Mesh& mesh1, const rai::Mesh& mesh2,
                       const rai::Transformation& t1, const rai::Transformation& t2,
                       rai::Vector& p1, rai::Vector& p2,
                       rai::Vector& e1, rai::Vector& e2,
                       GJK_point_type& pt1, GJK_point_type& pt2) {
  // convert meshes to 'Object_structures'
  Object_structure m1,m2;
  rai::Array<double*> Vhelp1, Vhelp2;
  m1.numpoints = mesh1.V.d0;  m1.vertices = mesh1.V.getCarray(Vhelp1);  m1.rings=NULL; //TODO: rings would make it faster
  m2.numpoints = mesh2.V.d0;  m2.vertices = mesh2.V.getCarray(Vhelp2);  m2.rings=NULL;
  
  // convert transformations to affine matrices
  arr T1,T2;
  rai::Array<double*> Thelp1, Thelp2;
  if(!!t1) {  T1=t1.getAffineMatrix();  T1.getCarray(Thelp1);  }
  if(!!t2) {  T2=t2.getAffineMatrix();  T2.getCarray(Thelp2);  }
  
  // call GJK
  simplex_point simplex;
  double d2 = gjk_distance(&m1, Thelp1.p, &m2, Thelp2.p, (!p1?NULL:p1.p()), (!p2?NULL:p2.p()), &simplex, 0);
  
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
      if(d2>EPSILON) LOG(-2) <<"GJK converges to simplex!";
    }
    
//    cout <<"point types= " <<pt1 <<' ' <<pt2 <<endl;
//    CHECK(!(pt1==3 && pt2==3),"");
//    CHECK(!(pt1==2 && pt2==3),"");
//    CHECK(!(pt1==3 && pt2==2),"");
  }
  
  return d2;
}
#else
double GJK_distance(rai::Mesh& mesh1, rai::Mesh& mesh2,
                    rai::Transformation& t1, rai::Transformation& t2,
                    rai::Vector& p1, rai::Vector& p2) { NICO }
#endif

//===========================================================================
//
// Lewiner interface
//

#ifdef RAI_extern_Lewiner
#  include "Lewiner/MarchingCubes.h"

void rai::Mesh::setImplicitSurface(ScalarFunction f, double lo, double hi, uint res) {
  MarchingCubes mc(res, res, res);
  mc.init_all() ;
  double startTime = rai::timerRead();
  //compute data
  uint k=0, j=0, i=0;
  float x=lo, y=lo, z=lo;
  for(k=0; k<res; k++) {
    z = lo+k*(hi-lo)/res;
    for(j=0; j<res; j++) {
      y = lo+j*(hi-lo)/res;
      for(i=0; i<res; i++) {
        x = lo+i*(hi-lo)/res;
        mc.set_data(f(NoArr, NoArr, ARR((double)x, (double)y, (double)z)), i, j, k) ;
      }
    }
  }
  cout << "calculation of data took: " << rai::timerRead() - startTime << " seconds" << endl;
  mc.run();
  mc.clean_temps();
  
  //convert to Mesh
  clear();
  V.resize(mc.nverts(), 3);
  T.resize(mc.ntrigs(), 3);
  for(i=0; i<V.d0; i++) {
    V(i, 0)=lo+mc.vert(i)->x*(hi-lo)/res;
    V(i, 1)=lo+mc.vert(i)->y*(hi-lo)/res;
    V(i, 2)=lo+mc.vert(i)->z*(hi-lo)/res;
  }
  for(i=0; i<T.d0; i++) {
    T(i, 0)=mc.trig(i)->v1;
    T(i, 1)=mc.trig(i)->v2;
    T(i, 2)=mc.trig(i)->v3;
  }
}

void rai::Mesh::setImplicitSurface(ScalarFunction f, double xLo, double xHi, double yLo, double yHi, double zLo, double zHi, uint res) {
  MarchingCubes mc(res, res, res);
  mc.init_all() ;
  
  //compute data
  uint k=0, j=0, i=0;
  float x, y, z;
  for(k=0; k<res; k++) {
    z = zLo+k*(zHi-zLo)/res;
    for(j=0; j<res; j++) {
      y = yLo+j*(yHi-yLo)/res;
      for(i=0; i<res; i++) {
        x = xLo+i*(xHi-xLo)/res;
        mc.set_data(f(NoArr, NoArr, ARR((double)x, (double)y, (double)z)), i, j, k) ;
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
    V(i, 0)=xLo+mc.vert(i)->x*(xHi-xLo)/res;
    V(i, 1)=yLo+mc.vert(i)->y*(yHi-yLo)/res;
    V(i, 2)=zLo+mc.vert(i)->z*(zHi-zLo)/res;
  }
  for(i=0; i<T.d0; i++) {
    T(i, 0)=mc.trig(i)->v1;
    T(i, 1)=mc.trig(i)->v2;
    T(i, 2)=mc.trig(i)->v3;
  }
}

#else //extern_Lewiner
void rai::Mesh::setImplicitSurface(ScalarFunction f, double lo, double hi, uint res) {
  NICO
}
#endif
/** @} */

//===========================================================================

DistanceFunction_Sphere::DistanceFunction_Sphere(const rai::Transformation& _t, double _r):t(_t),r(_r) {
  ScalarFunction::operator=([this](arr& g, arr& H, const arr& x)->double{ return f(g,H,x); });
}

double DistanceFunction_Sphere::f(arr& g, arr& H, const arr& x) {
  arr d = x-conv_vec2arr(t.pos);
  double len = length(d);
  if(!!g) g = d/len;
  if(!!H) H = 1./len * (eye(3) - (d^d)/(len*len));
  return len-r;
}

//===========================================================================

//double DistanceFunction_InfCylinder::fs(arr& g, arr& H, const arr& x){
//  z = z / length(z);
//  arr a = (x-c) - scalarProduct((x-c), z) * z;
//  arr I(x.d0,x.d0);
//  uint i;
//  double na = length(a);

//  if(!!g) g = s*a/na;
//  if(!!H){
//    I.setZero();
//    for(i=0;i<x.d0;++i) I(i,i)=1;
//    H = s/na * (I - z*(~z) - 1/(na*na) * a*(~a));
//  }
//  return s*(na-r);
//}

//===========================================================================

DistanceFunction_Cylinder::DistanceFunction_Cylinder(const rai::Transformation& _t, double _r, double _dz):t(_t),r(_r),dz(_dz) {
  ScalarFunction::operator=([this](arr& g, arr& H, const arr& x)->double{ return f(g,H,x); });
}

double DistanceFunction_Cylinder::f(arr& g, arr& H, const arr& x) {
  arr z = conv_vec2arr(t.rot.getZ());
  arr c = conv_vec2arr(t.pos);
  arr b = scalarProduct(x-c, z) * z;
  arr a = (x-c) - b;
  arr I(3,3);
  double la = length(a);
  double lb = length(b);
  arr aaTovasq = 1/(la*la) * (a^a);
  arr zzT = z^z;
  
  if(lb < dz/2.) {   // x projection on z is inside cyl
    if(la<r && (dz/2.-lb)<(r-la)) { // x is INSIDE the cyl and closer to the lid than the wall
      if(!!g) g = 1./lb*b; //z is unit: s*z*|z|*sgn(b*z) = s*b/nb
      if(!!H) { I.setZero(); H=I; }
      return lb-dz/2.;
    } else { // closer to the side than to a lid (inc. cases in- and outside the tube, because (r-na)<0 then)
      if(!!g) g = a/la;
      if(!!H) {
        I.setId(3);
        H = 1./la * (I - zzT - aaTovasq);
      }
      return la-r;
    }
  } else { // x projection on z is outside cylinder
    if(la < r) {  // inside the infinite cylinder
      if(!!g) g = b/lb;
      if(!!H) H.resize(3,3).setZero();
      return lb-dz/2.;
    } else { // outside the infinite cyl
      arr v =  b/lb * (lb-dz/2.)  + a/la * (la-r); //MT: good! (note: b/nb is the same as z) SD: well, b/nb is z or -z.
      double nv=length(v);
      if(!!g) g = v/nv;
      if(!!H) {
        I.setId(3);
        arr dvdx = (la-r)/la*(I - zzT - aaTovasq)
                   + aaTovasq + zzT;
        H = 1./nv* (dvdx - 1/nv/nv * (v^v) * (~dvdx));
      }
      return nv;
    }
  }
  HALT("You shouldn't be here!");
}

//===========================================================================

/// dx, dy, dz are box-wall-coordinates: width=2*dx...; t is box transform; x is query point in world
void closestPointOnBox(arr& closest, arr& signs, const rai::Transformation& t, double dx, double dy, double dz, const arr& x) {
  arr rot = t.rot.getArr();
  arr a_rel = (~rot)*(x-conv_vec2arr(t.pos)); //point in box coordinates
  arr dim = {dx, dy, dz};
  signs.resize(3);
  signs.setZero();
  closest = a_rel;
  arr del_abs = fabs(a_rel)-dim;
  if(del_abs.max()<0.) { //inside
    uint side=del_abs.maxIndex(); //which side are we closest to?
    //in positive or neg direction?
    if(a_rel(side)>0) { closest(side) = dim(side);  signs(side)=+1.; }
    else             { closest(side) =-dim(side);  signs(side)=-1.; }
  } else { //outside
    for(uint side=0; side<3; side++) {
      if(closest(side)<-dim(side)) { signs(side)=-1.; closest(side)=-dim(side); }
      if(closest(side)> dim(side)) { signs(side)=+1.; closest(side)= dim(side); }
    }
  }
  closest = rot*closest + t.pos.getArr();
}

//===========================================================================

DistanceFunction_Box::DistanceFunction_Box(const rai::Transformation& _t, double _dx, double _dy, double _dz, double _r):t(_t),dx(_dx),dy(_dy),dz(_dz), r(_r) {
  ScalarFunction::operator=([this](arr& g, arr& H, const arr& x)->double{ return f(g,H,x); });
}

double DistanceFunction_Box::f(arr& g, arr& H, const arr& x) {
  arr rot = t.rot.getArr();
  arr a_rel = (~rot)*(x-conv_vec2arr(t.pos)); //point in box coordinates
  arr dim = {dx, dy, dz};
  
  arr closest = a_rel;
  arr del_abs = fabs(a_rel)-dim;
  //-- find closest point on box and distance to it
  if(del_abs.max()<0.) { //inside
    uint side=del_abs.maxIndex(); //which side are we closest to?
    if(a_rel(side)>0) closest(side) = dim(side);  else  closest(side)=-dim(side); //in positive or neg direction?
  } else { //outside
    closest = elemWiseMax(-dim,closest);
    closest = elemWiseMin(dim,closest);
  }
  
  arr del = a_rel-closest;
  double d = length(del);
  if(!!g) g = rot*del/d; //transpose(R) rotates the gradient back to world coordinates
  if(!!H) {
    if(d<0.) { //inside
      H.resize(3,3).setZero();
    } else { //outside
      if(del_abs.min()>0.) { //outside on all 3 axis
        H = 1./d * (eye(3) - (del^del)/(d*d));
      } else {
        arr edge=del_abs;
        for(double& z: edge) z=(z<0.)?0.:1.;
        if(sum(edge)<=1.1) { //closest to the plane (equals 1.)
          H.resize(3,3).setZero();
        } else { //closest to an edge
          edge = 1.-edge;
          H = 1./d * (eye(3) - (del^del)/(d*d) - (edge^edge));
        }
      }
      H = rot*H*(~rot);
    }
  }
  
  return d-r;
}

ScalarFunction DistanceFunction_SSBox = [](arr& g, arr& H, const arr& x) -> double{
  // x{0,2} are box-wall-coordinates, not width!
  CHECK_EQ(x.N, 14, "query-pt + abcr + pose");
  rai::Transformation t;
  t.pos.set(x({7,9}));
  t.rot.set(x({10,13}));
  t.rot.normalize();
  arr closest, signs;
  closestPointOnBox(closest, signs, t, x(3), x(4), x(5), x({0,2}));
  arr grad = x({0,2}) - closest;
  double d = length(grad);
  grad /= d;
  d -= x(6);
  if(!!g) {
    g.resize(14);
    g.setZero();
    g({0,2}) = grad;
    g({7,9}) = - grad;
    g({3,5}) = - signs%(t.rot / rai::Vector(grad)).getArr();
    g(6) = -1.;
    g({10,13}) = ~grad*crossProduct(t.rot.getJacobian(), (x({0,2})-t.pos.getArr()));
    g({10,13})() /= -sqrt(sumOfSqr(x({10,13}))); //account for the potential non-normalization of q
  }
  return d;
};

uint rai::Mesh::support(const arr &dir) {
  if(!graph.N) { //build graph
    graph.resize(V.d0);
    for(uint i=0; i<T.d0; i++) {
      graph(T(i,0)).setAppend(T(i,1));
      graph(T(i,0)).setAppend(T(i,2));
      graph(T(i,1)).setAppend(T(i,0));
      graph(T(i,1)).setAppend(T(i,2));
      graph(T(i,2)).setAppend(T(i,0));
      graph(T(i,2)).setAppend(T(i,1));
    }
  }
  
  arr q(V.d0);
  for(uint i=0; i<V.d0; i++) q(i) = scalarProduct(dir, V[i]);
  return argmax(q);
  
#if 0
  uint v=0;
  arr q;
  double ma = scalarProduct(dir, V[v]);
  for(;;) {
    //comput scalar product for all neighbors
    uintA &neigh=graph(v);
    q.resize(neigh.N);
    for(uint i=0; i<neigh.N; i++) q(i) = scalarProduct(dir, V[neigh(i)]);
    uint bestNeighbor = argmax(q);
    if(q(bestNeighbor)>ma) {
      v = neigh(bestNeighbor);
      ma = q(bestNeighbor);
    } else {
      return v;
    }
  }
  return -1;
#endif
}

void rai::Mesh::supportMargin(uintA &verts, const arr &dir, double margin, int initialization) {
  if(initialization<0 || !graph.N) initialization=support(dir);
  
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
}

