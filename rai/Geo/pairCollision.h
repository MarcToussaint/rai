/*  ------------------------------------------------------------------
    Copyright (c) 2011-2020 Marc Toussaint
    email: toussaint@tu-berlin.de

    This code is distributed under the MIT License.
    Please see <root-path>/LICENSE for details.
    --------------------------------------------------------------  */

#pragma once

#include "mesh.h"

struct PairCollision : GLDrawer, NonCopyable {
  //INPUTS
  const rai::Mesh* mesh1=0;
  const rai::Mesh* mesh2=0;
  const rai::Transformation* t1=0;
  const rai::Transformation* t2=0;
  double rad1=0., rad2=0.; ///< only kinVector and glDraw account for this; the basic collision geometry (OUTPUTS below) is computed neglecting radii!!

  //OUTPUTS
  double distance=0.; ///< negative=penetration
  arr p1, p2;      ///< closest points on the shapes
  arr normal;      ///< normal such that "<normal, p1-p2> = distance" is guaranteed (pointing from obj2 to obj1)
  arr simplex1;    ///< simplex on obj1 defining the collision geometry
  arr simplex2;    ///< simplex on obj2 defining the collision geometry

//  arr m1, m2, eig1, eig2; ///< output of marginAnalysis: mean and eigenvalues of ALL point on the objs (not only simplex) that define the collision

  arr poly, polyNorm;

  PairCollision(const rai::Mesh& mesh1, const rai::Mesh& mesh2,
                const rai::Transformation& t1, const rai::Transformation& t2,
                double rad1=0., double rad2=0.);
  PairCollision(ScalarFunction func1, ScalarFunction func2, const arr& seed);
  ~PairCollision() {}

  void write(std::ostream& os) const;

  void glDraw(struct OpenGL&);

  double getDistance() { return distance-rad1-rad2; }

  void kinDistance(arr& y, arr& J,
                   const arr& Jp1, const arr& Jp2);
  void kinNormal(arr& y, arr& J,
                 const arr& Jp1, const arr& Jp2,
                 const arr& Jx1, const arr& Jx2);
  void kinVector(arr& y, arr& J,
                 const arr& Jp1, const arr& Jp2,
                 const arr& Jx1, const arr& Jx2);
  void kinPointP1(arr& y, arr& J,
                  const arr& Jp1, const arr& Jp2,
                  const arr& Jx1, const arr& Jx2);
  void kinPointP2(arr& y, arr& J,
                  const arr& Jp1, const arr& Jp2,
                  const arr& Jx1, const arr& Jx2);
  void kinCenter(arr& y, arr& J,
                 const arr& Jp1, const arr& Jp2,
                 const arr& Jx1, const arr& Jx2);

  void nearSupportAnalysis(double eps=1e-6); ///< analyses not only closest obj support (the simplex) but all points within a margin

  void computeSupportPolygon();

 private:
  //wrappers of external libs
  enum CCDmethod { _ccdGJKIntersect,  _ccdGJKSeparate, _ccdGJKPenetration, _ccdMPRIntersect, _ccdMPRPenetration };
  void libccd(rai::Mesh& m1, rai::Mesh& m2, CCDmethod method); //calls ccdMPRPenetration of libccd
  void GJK_sqrDistance(); //gjk_distance of libGJK
  bool simplexType(uint i, uint j) { return simplex1.d0==i && simplex2.d0==j; } //helper
};

//return normals and closes points for 1-on-3 simplices or 2-on-2 simplices
double coll_1on2(arr& p2, arr& normal, double& s, const arr& pts1, const arr& pts2);
double coll_1on3(arr& p2, arr& normal, const arr& pts1, const arr& pts2);
double coll_2on2(arr& p1, arr& p2, arr& normal, const arr& pts1, const arr& pts2);
double coll_2on3(arr& p1, arr& p2, arr& normal, const arr& pts1, const arr& pts2, const arr& center);
double coll_3on3(arr& p1, arr& p2, arr& normal, const arr& pts1, const arr& pts2, const arr& center);

stdOutPipe(PairCollision)
