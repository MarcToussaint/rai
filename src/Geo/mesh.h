/*  ------------------------------------------------------------------
    Copyright (c) 2011-2024 Marc Toussaint
    email: toussaint@tu-berlin.de

    This code is distributed under the MIT License.
    Please see <root-path>/LICENSE for details.
    --------------------------------------------------------------  */

#pragma once

#include "geo.h"

struct OpenGL;

//fwd decl
namespace rai { struct Mesh; }
typedef rai::Array<rai::Mesh> MeshA;
typedef rai::Array<rai::Mesh*> MeshL;
struct ANN;

namespace rai {

enum ShapeType { ST_none=-1, ST_box=0, ST_sphere, ST_capsule, ST_mesh, ST_cylinder, ST_marker, ST_pointCloud, ST_ssCvx, ST_ssBox, ST_ssCylinder, ST_ssBoxElip, ST_quad, ST_camera, ST_sdf, ST_density, ST_lines };

//===========================================================================
/// a mesh (arrays of vertices, triangles, colors & normals)
struct Mesh {
  arr V;                ///< vertices
  arr Vn;               ///< vertex normals (optional)
  arr C;                ///< vertex colors (optional, may be just 3 numbers -> global color)

  uintA T;              ///< triangles (faces, empty -> point cloud)
  arr   Tn;             ///< triangle normals (optional)

  uintA Tt;             ///< triangle texture indices (pointing into coordinates)
  arr tex;              ///< texture coordinates
  byteA texImg;         ///< texture image
  int texture=-1;       ///< GL texture name created with glBindTexture

  uintA cvxParts;
  uintAA graph;         ///< for every vertex, the set of neighboring vertices
  shared_ptr<ANN> ann;

  rai::Transformation glX; ///< transform (only used for drawing! Otherwise use applyOnPoints)  (optional)

  int version = 0;
  long parsing_pos_start;
  long parsing_pos_end;

  uint _support_vertex=0;
  bool isArrayFormatted=false;

  Mesh();
  ~Mesh();

  /// @name set or create
  void clear();
  void setBox(bool edgesOnly=false);
  void setBox(const arr& lo, const arr& up, bool edgesOnly=true);
  Mesh& setDot(); ///< an awkward mesh: just a single dot, not tris (e.g. cvx core of a sphere...)
  void setLine(double l); ///< an awkward mesh: just a single line, not tris (e.g. cvx core of a sphere...)
  void setQuad(double x_width=1., double y_width=1., const byteA& _texImg= {}, bool flipY=false, bool texByReference=false); ///< a quat, optionally with texture
  void setTetrahedron();
  void setOctahedron();
  void setDodecahedron();
  void setIcosahedron();
  void setSphere(uint fineness=2);
  void setHalfSphere(uint fineness=2);
  void setCylinder(double r, double l, uint fineness=2);
  void setCone(double r, double h, uint fineness=2);
  void setCapsule(double r, double l, uint fineness=2);
  void setSSBox(double x_width, double y_width, double z_height, double r, uint fineness=2);
  void setSSCvx(const arr& core, double r, uint fineness=2);
  void setImplicitSurface(const ScalarFunction& f, double lo=-10., double hi=+10., uint res=100);
  void setImplicitSurface(const ScalarFunction& f, double xLo, double xHi, double yLo, double yHi, double zLo, double zHi, uint res);
  void setImplicitSurface(const arr& gridValues, const arr& lo, const arr& hi);
  void setImplicitSurface(const floatA& gridValues, const arr& lo, const arr& hi);
  void setImplicitSurfaceBySphereProjection(const ScalarFunction& f, double rad, uint fineness=3);
  Mesh& setRandom(uint vertices=10);
  void setGrid(uint X, uint Y);

  /// @name transform and modify
  void subDivide();
  void subDivide(uint tri);
  void scale(double s);
  void scale(double sx, double sy, double sz);
  void scale(const arr& s);
  void translate(double dx, double dy, double dz);
  void translate(const arr& d);
  void transform(const Transformation& t);
  Vector center();
  void box();
  void addMesh(const rai::Mesh& mesh2, const rai::Transformation& X=0);
  void addConvex(const arr& points, const arr& color=NoArr);
  void makeConvexHull();
  void makeTriangleFan();
  void makeLines();
  void makeArrayFormatted(double avgNormalsThreshold=.9);

  /// @name convex decomposition
  rai::Mesh decompose();
  uint getComponents();

  /// @name support function
  uint support(const double* dir);
  void supportMargin(uintA& verts, const arr& dir, double margin, int initialization=-1);

  /// @name internal computations & cleanup
  void computeTriNormals();
  void computeFaceColors();
  arr computeTriDistances();
  void buildGraph();
  void deleteUnusedVertices();
  void fuseNearVertices(double tol=1e-5);
  void deleteVertices(uintA& delLabels);
  void clean();
  void flipFaces();
  rai::Vector getCenter() const;
  arr getMean() const;
  void getBox(double& dx, double& dy, double& dz) const;
  arr getBounds() const;
  double getRadius() const;
  double getArea() const;
  double getArea(uint tri) const;
  double getCircum() const;
  double getCircum(uint tri) const;
  double getVolume() const;
  uintA getVertexDegrees() const;

  ANN& ensure_ann();

  /// Comparing two Meshes - static function
  static double meshMetric(const Mesh& trueMesh, const Mesh& estimatedMesh); // Haussdorf metric

  //[preliminary]]
  void skin(uint i);
  void deleteGlTexture();

  /// @name IO
  void write(std::ostream&) const; ///< only writes generic info
  void read(std::istream&, const char* fileExtension, const char* filename=nullptr);
  void readFile(const char* filename);
  void readTriFile(std::istream& is);
  void readOffFile(std::istream& is);
  void readPlyFile(std::istream& is);
  void writeTriFile(const char* filename);
  void writeOffFile(const char* filename);
  void writePLY(const char* fn, bool bin=true);
  void readPLY(const char* fn);
  void writeJson(std::ostream&);
  void readJson(std::istream&);
  void writeArr(std::ostream&);
  void writeH5(const char* filename, const String& group);
  void readH5(const char* filename, const String& group);
  void readArr(std::istream&);
  void readPts(std::istream&);
};

stdOutPipe(Mesh)

} //namespace

//===========================================================================

//
// operators
//

uintA getSubMeshPositions(const char* filename);
arr MinkowskiSum(const arr& A, const arr& B);

//===========================================================================
//
// C-style functions
//

void inertiaSphere(double* Inertia, double& mass, double density, double radius);
void inertiaBox(double* Inertia, double& mass, double density, double dx, double dy, double dz);
void inertiaCylinder(double* Inertia, double& mass, double density, double height, double radius);
void inertiaMesh(double* I, double& mass, double density, const rai::Mesh& m);

