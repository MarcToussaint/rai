/*  ------------------------------------------------------------------
    Copyright (c) 2011-2020 Marc Toussaint
    email: toussaint@tu-berlin.de

    This code is distributed under the MIT License.
    Please see <root-path>/LICENSE for details.
    --------------------------------------------------------------  */

#pragma once

#include "geo.h"
#include "../Core/array.h"

namespace rai { struct Mesh; }
typedef rai::Array<rai::Mesh> MeshA;
typedef rai::Array<rai::Mesh*> MeshL;
void glDrawMeshes(void*, OpenGL&);

namespace rai {

enum ShapeType { ST_none=-1, ST_box=0, ST_sphere, ST_capsule, ST_mesh, ST_cylinder, ST_marker, ST_pointCloud, ST_ssCvx, ST_ssBox, ST_ssBoxElip };

//===========================================================================
/// a mesh (arrays of vertices, triangles, colors & normals)
struct Mesh : GLDrawer {
  arr V;                ///< vertices
  arr Vn;               ///< vertex normals (optional)
  arr C;                ///< vertex colors (optional, may be just 3 numbers -> global color)

  uintA T;              ///< triangles (faces, empty -> point cloud)
  arr   Tn;             ///< triangle normals (optional)

  uintA Tt;             ///< triangle texture indices
  arr tex;              ///< texture coordinates
  byteA texImg;         ///< texture image
  int texture=-1;       ///< GL texture name created with glBindTexture

  uintAA graph;         ///< for every vertex, the set of neighboring vertices

  rai::Transformation glX; ///< transform (only used for drawing! Otherwise use applyOnPoints)  (optional)

  long parsing_pos_start;
  long parsing_pos_end;

  uint _support_vertex=0;

  Mesh();

  /// @name set or create
  void clear();
  void setBox();
  void setDot(); ///< an awkward mesh: just a single dot, not tris (e.g. cvx core of a sphere...)
  void setLine(double l); ///< an awkward mesh: just a single line, not tris (e.g. cvx core of a sphere...)
  void setTetrahedron();
  void setOctahedron();
  void setDodecahedron();
  void setSphere(uint fineness=2);
  void setHalfSphere(uint fineness=2);
  void setCylinder(double r, double l, uint fineness=2);
  void setCapsule(double r, double l, uint fineness=2);
  void setSSBox(double x_width, double y_width, double z_height, double r, uint fineness=2);
  void setSSCvx(const arr& core, double r, uint fineness=2);
  void setImplicitSurface(ScalarFunction f, double lo=-10., double hi=+10., uint res=100);
  void setImplicitSurface(ScalarFunction f, double xLo, double xHi, double yLo, double yHi, double zLo, double zHi, uint res);
  void setImplicitSurface(const arr& gridValues, const arr& lo, const arr& hi);
  void setImplicitSurfaceBySphereProjection(ScalarFunction f, double rad, uint fineness=3);
  Mesh& setRandom(uint vertices=10);
  void setGrid(uint X, uint Y);

  /// @name transform and modify
  void subDivide();
  void subDivide(uint tri);
  void scale(double f);
  void scale(double sx, double sy, double sz);
  void translate(double dx, double dy, double dz);
  void translate(const arr& d);
  void transform(const Transformation& t);
  Vector center();
  void box();
  void addMesh(const rai::Mesh& mesh2, const rai::Transformation& X=0);
  void makeConvexHull();
  void makeTriangleFan();
  void makeLineStrip();

  /// @name support function
  uint support(const double* dir);
  void supportMargin(uintA& verts, const arr& dir, double margin, int initialization=-1);

  /// @name internal computations & cleanup
  void computeNormals();
  arr computeTriDistances();
  void buildGraph();
  void deleteUnusedVertices();
  void fuseNearVertices(double tol=1e-5);
  void clean();
  void flipFaces();
  rai::Vector getCenter() const;
  arr getMean() const;
  void getBox(double& dx, double& dy, double& dz) const;
  arr getBox() const;
  double getRadius() const;
  double getArea() const;
  double getArea(uint tri) const;
  double getCircum() const;
  double getCircum(uint tri) const;
  double getVolume() const;

  /// Comparing two Meshes - static function
  static double meshMetric(const Mesh& trueMesh, const Mesh& estimatedMesh); // Haussdorf metric

  //[preliminary]]
  void skin(uint i);

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
  void writeArr(std::ostream&);
  void readArr(std::istream&);

  void glDraw(struct OpenGL&);
};

stdOutPipe(Mesh)

} //END of namespace

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


//===========================================================================
//
// GJK interface
//

enum GJK_point_type { GJK_none=0, GJK_vertex, GJK_edge, GJK_face };
extern GJK_point_type& NoPointType;
double GJK_sqrDistance(const rai::Mesh& mesh1, const rai::Mesh& mesh2,
                       const rai::Transformation& t1, const rai::Transformation& t2,
                       rai::Vector& p1, rai::Vector& p2,
                       rai::Vector& e1, rai::Vector& e2,
                       GJK_point_type& pt1, GJK_point_type& pt2);
