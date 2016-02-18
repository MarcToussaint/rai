/*  ---------------------------------------------------------------------
    Copyright 2014 Marc Toussaint
    email: marc.toussaint@informatik.uni-stuttgart.de
    
    This program is free software: you can redistribute it and/or modify
    it under the terms of the GNU General Public License as published by
    the Free Software Foundation, either version 3 of the License, or
    (at your option) any later version.
    
    This program is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
    GNU General Public License for more details.
    
    You should have received a COPYING file of the GNU General Public License
    along with this program. If not, see <http://www.gnu.org/licenses/>
    -----------------------------------------------------------------  */


#ifndef MLR_mesh_h
#define MLR_mesh_h

#include <Core/array.h>
#include "geo.h"

/// @file
/// @ingroup group_geo
/// @addtogroup group_geo
/// @{

namespace ors {

//===========================================================================
/// a mesh (arrays of vertices, triangles, colors & normals)
struct Mesh : GLDrawer {
  arr V;                ///< vertices
  arr Vn;               ///< triangle normals
  arr C;                ///< vertex colors
  
  uintA T;              ///< triangles (faces)
  arr   Tn;             ///< triangle normals

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
  void setSSBox(double x, double y, double z, double r, uint fineness=3);
  void setSSCvx(const ors::Mesh& m, double r, uint fineness=3);
  void setImplicitSurface(ScalarFunction f, double lo=-10., double hi=+10., uint res=100);
  void setRandom(uint vertices=10);
  void setGrid(uint X, uint Y);

  /// @name transform and modify
  void subDevide();
  void scale(double f);
  void scale(double sx, double sy, double sz);
  void translate(double dx, double dy, double dz);
  Vector center();
  void box();
  void addMesh(const ors::Mesh& mesh2);
  void makeConvexHull();
  void makeSSBox(arr& x, ors::Transformation& t, const arr& X, uint trials=10, int verbose=0);
  
  /// @name internal computations & cleanup
  void computeNormals();
  void deleteUnusedVertices();
  void fuseNearVertices(double tol=1e-5);
  void clean();
  void flipFaces();
  Vector getMeanVertex() const;
  double getRadius() const;
  double getArea() const;
  double getCircum() const;
  double getVolume() const;


  //[preliminary]]
  void skin(uint i);
  
  /// @name IO
  void write(std::ostream&) const; ///< only writes generic info
  void read(std::istream&, const char* fileExtension);
  void readFile(const char* filename);
  void readTriFile(std::istream& is);
  void readObjFile(std::istream& is);
  void readOffFile(std::istream& is);
  void readPlyFile(std::istream& is);
  void readStlFile(std::istream& is);
  void writeTriFile(const char* filename);
  void writeOffFile(const char* filename);
  void writePLY(const char *fn, bool bin);
  void readPLY(const char *fn);
  void glDraw(struct OpenGL&);
};
} //END of namespace
stdOutPipe(ors::Mesh)

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
  ors::Transformation t; double r;
  DistanceFunction_Sphere(const ors::Transformation& _t, double _r);
  double f(arr& g, arr& H, const arr& x);
};

struct DistanceFunction_Box:ScalarFunction{
  ors::Transformation t; double dx, dy, dz, r;
  DistanceFunction_Box(const ors::Transformation& _t, double _dx, double _dy, double _dz, double _r=0.);
  double f(arr& g, arr& H, const arr& x);
};

struct DistanceFunction_Cylinder:ScalarFunction{
  ors::Transformation t; double r, dz;
  DistanceFunction_Cylinder(const ors::Transformation& _t, double _r, double _dz);
  double f(arr& g, arr& H, const arr& x);
};

extern ScalarFunction DistanceFunction_SSBox;


//===========================================================================
//
// GJK interface
//

enum GJK_point_type { GJK_vertex=1, GJK_edge, GJK_face };
extern GJK_point_type& NoPointType;
double GJK_sqrDistance(const ors::Mesh& mesh1, const ors::Mesh& mesh2,
                    const ors::Transformation& t1, const ors::Transformation& t2,
                    ors::Vector& p1, ors::Vector& p2,
                    ors::Vector& e1, ors::Vector& e2,
                    GJK_point_type& pt1, GJK_point_type& pt2);


#endif
