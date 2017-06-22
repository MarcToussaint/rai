/*  ------------------------------------------------------------------
    Copyright 2016 Marc Toussaint
    email: marc.toussaint@informatik.uni-stuttgart.de
    
    This program is free software: you can redistribute it and/or modify
    it under the terms of the GNU General Public License as published by
    the Free Software Foundation, either version 3 of the License, or (at
    your option) any later version. This program is distributed without
    any warranty. See the GNU General Public License for more details.
    You should have received a COPYING file of the full GNU General Public
    License along with this program. If not, see
    <http://www.gnu.org/licenses/>
    --------------------------------------------------------------  */


#ifndef MLR_mesh_h
#define MLR_mesh_h

#include <Core/array.h>
#include "geo.h"

namespace mlr { struct Mesh; }
typedef mlr::Array<mlr::Mesh> MeshA;
void glDrawMeshes(void*);

/// @file
/// @ingroup group_geo
/// @addtogroup group_geo
/// @{

namespace mlr {

//===========================================================================
/// a mesh (arrays of vertices, triangles, colors & normals)
struct Mesh : GLDrawer {
  arr V;                ///< vertices
  arr Vn;               ///< triangle normals (optional)
  arr C;                ///< vertex colors (optional, may be just 3 numbers -> global color)
  
  uintA T;              ///< triangles (faces, empty -> point cloud)
  arr   Tn;             ///< triangle normals (optional)

  mlr::Transformation glX; ///< transform (only used for drawing! Otherwise use applyOnPoints)  (optional)

  long parsing_pos_start;
  long parsing_pos_end;
  
  Mesh();
  
  /// @name set or create
  void clear();
  void setBox();
  void setTetrahedron();
  void setOctahedron();
  void setDodecahedron();
  void setSphere(uint fineness=3);
  void setHalfSphere(uint fineness=3);
  void setCylinder(double r, double l, uint fineness=3);
  void setCappedCylinder(double r, double l, uint fineness=3);
  void setSSBox(double x_width, double y_width, double z_height, double r, uint fineness=3);
  void setSSCvx(const mlr::Mesh& m, double r, uint fineness=3);
  void setImplicitSurface(ScalarFunction f, double lo=-10., double hi=+10., uint res=100);
  void setImplicitSurface(ScalarFunction f, double xLo, double xHi, double yLo, double yHi, double zLo, double zHi, uint res);
  void setRandom(uint vertices=10);
  void setGrid(uint X, uint Y);

  /// @name transform and modify
  void subDivide();
  void scale(double f);
  void scale(double sx, double sy, double sz);
  void translate(double dx, double dy, double dz);
  void translate(const arr& d);
  void transform(const Transformation& t);
  mlr::Vector center();
  void box();
  void addMesh(const mlr::Mesh& mesh2);
  void makeConvexHull();
  void makeTriangleFan();
  void makeLineStrip();
  
  /// @name internal computations & cleanup
  void computeNormals();
  void deleteUnusedVertices();
  void fuseNearVertices(double tol=1e-5);
  void clean();
  void flipFaces();
  arr getMean() const;
  void getBox(double& dx, double& dy, double& dz) const;
  double getRadius() const;
  double getArea() const;
  double getCircum() const;
  double getVolume() const;

  /// Comparing two Meshes - static function
  static double meshMetric(const Mesh &trueMesh, const Mesh &estimatedMesh); // Haussdorf metric

  //[preliminary]]
  void skin(uint i);
  
  /// @name IO
  void write(std::ostream&) const; ///< only writes generic info
  void read(std::istream&, const char* fileExtension, const char* filename=NULL);
  void readFile(const char* filename);
  void readTriFile(std::istream& is);
  void readObjFile(std::istream& is);
  void readOffFile(std::istream& is);
  void readPlyFile(std::istream& is);
  bool readStlFile(std::istream& is);
  void writeTriFile(const char* filename);
  void writeOffFile(const char* filename);
  void writePLY(const char *fn, bool bin);
  void readPLY(const char *fn);
  void glDraw(struct OpenGL&);
};
} //END of namespace
stdOutPipe(mlr::Mesh)

//===========================================================================

//
// operators
//

uintA getSubMeshPositions(const char* filename);



//===========================================================================
//
// C-style functions
//

void inertiaSphere(double *Inertia, double& mass, double density, double radius);
void inertiaBox(double *Inertia, double& mass, double density, double dx, double dy, double dz);
void inertiaCylinder(double *Inertia, double& mass, double density, double height, double radius);


/// @} end of group_geo


/** @} */


//===========================================================================
//
// analytic distance functions
//

struct DistanceFunction_Sphere:ScalarFunction{
  mlr::Transformation t; double r;
  DistanceFunction_Sphere(const mlr::Transformation& _t, double _r);
  double f(arr& g, arr& H, const arr& x);
};

struct DistanceFunction_Box:ScalarFunction{
  mlr::Transformation t; double dx, dy, dz, r;
  DistanceFunction_Box(const mlr::Transformation& _t, double _dx, double _dy, double _dz, double _r=0.);
  double f(arr& g, arr& H, const arr& x);
};

struct DistanceFunction_Cylinder:ScalarFunction{
  mlr::Transformation t; double r, dz;
  DistanceFunction_Cylinder(const mlr::Transformation& _t, double _r, double _dz);
  double f(arr& g, arr& H, const arr& x);
};

extern ScalarFunction DistanceFunction_SSBox;


//===========================================================================
//
// GJK interface
//

enum GJK_point_type { GJK_vertex=1, GJK_edge, GJK_face };
extern GJK_point_type& NoPointType;
double GJK_sqrDistance(const mlr::Mesh& mesh1, const mlr::Mesh& mesh2,
                    const mlr::Transformation& t1, const mlr::Transformation& t2,
                    mlr::Vector& p1, mlr::Vector& p2,
                    mlr::Vector& e1, mlr::Vector& e2,
                    GJK_point_type& pt1, GJK_point_type& pt2);


#endif
