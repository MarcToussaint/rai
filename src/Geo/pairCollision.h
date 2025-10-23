/*  ------------------------------------------------------------------
    Copyright (c) 2011-2024 Marc Toussaint
    email: toussaint@tu-berlin.de

    This code is distributed under the MIT License.
    Please see <root-path>/LICENSE for details.
    --------------------------------------------------------------  */

#pragma once

#include "mesh.h"
#include "../Algo/ann.h"

namespace rai {

struct PairCollision {
  //INPUTS
  double rad1=0., rad2=0.; ///< only kinVector and glDraw account for this; the basic collision geometry (OUTPUTS below) is computed neglecting radii!!

  //OUTPUTS
  double distance=0.; ///< negative=penetration
  arr p1, p2;         ///< witness points on the shapes
  arr normal;         ///< normal such that "<normal, p1-p2> = distance" is guaranteed (pointing from obj2 to obj1)
  arr simp1, simp2;   ///< simplices on the shapes defining the collision geometry

  double getDistance() { return distance-rad1-rad2; }
  void write(std::ostream& os) const;

  // differentiable readout methods (Jp1 and Jx1 are linear and angular Jacobians of mesh1)
  void kinDistance(arr& y, arr& J, const arr& Jp1, const arr& Jp2);
  void kinNormal(arr& y, arr& J, const arr& Jp1, const arr& Jp2, const arr& Jx1, const arr& Jx2);
  void kinVector(arr& y, arr& J, const arr& Jp1, const arr& Jp2, const arr& Jx1, const arr& Jx2);
  void kinPointP1(arr& y, arr& J, const arr& Jp1, const arr& Jp2, const arr& Jx1, const arr& Jx2);
  void kinPointP2(arr& y, arr& J, const arr& Jp1, const arr& Jp2, const arr& Jx1, const arr& Jx2);
  void kinCenter(arr& y, arr& J, const arr& Jp1, const arr& Jp2, const arr& Jx1, const arr& Jx2);

protected:
  bool simplexType(uint i, uint j) { return simp1.d0==i && simp2.d0==j; } //helper
};

//===========================================================================

/* A class to represent a basic function: distance between two convex (decomposed) meshes
 * The constructor compute the collision geometry, the other methods are mostly readouts
 * The default is distance between two convex meshes
 * Also distance between point (=mesh1) and points (=mesh2)
 * Also distance between point (=mesh1) and decomposed mesh (=mesh2)
 */
struct PairCollision_CvxCvx : PairCollision, NonCopyable {

  //mesh-to-mesh
  PairCollision_CvxCvx(const arr& pts1, const arr& pts2,
                    const rai::Transformation& t1, const rai::Transformation& t2,
                    double rad1=0., double rad2=0.);
  //sdf-to-sdf -- TODO: own class!
  PairCollision_CvxCvx(ScalarFunction func1, ScalarFunction func2, const arr& seed);


  // void nearSupportAnalysis(double eps=1e-6); ///< analyses not only closest obj support (the simplex) but all points within a margin

private:
  //wrappers of external libs
  enum CCDmethod { _ccdGJKIntersect,  _ccdGJKSeparate, _ccdGJKPenetration, _ccdMPRIntersect, _ccdMPRPenetration };
  void libccd(const arr& m1, const arr& m2, CCDmethod method); //calls ccdMPRPenetration of libccd
  void GJK_sqrDistance(const arr& pts1, const arr& pts2, const rai::Transformation& t1, const rai::Transformation& t2); //gjk_distance of libGJK
};

//===========================================================================

struct PairCollision_CvxDecomp : PairCollision, NonCopyable {
  PairCollision_CvxDecomp(const arr& x, Mesh& mesh,
                         const rai::Transformation& t1, const rai::Transformation& t2,
                         double rad1=0., double rad2=0.);
};

//===========================================================================

struct PairCollision_PtPcl : PairCollision, NonCopyable {
  PairCollision_PtPcl(const arr& x, ANN& ann,
               const rai::Transformation& t1,
               const rai::Transformation& t2,
               double rad1=0., double rad2=0.);
};

//===========================================================================

//return normals and closes points for 1-on-3 simplices or 2-on-2 simplices
double coll_1on2(arr& p2, arr& normal, double& s, const arr& pts1, const arr& pts2);
double coll_1on3(arr& p2, arr& normal, const arr& pts1, const arr& pts2);
double coll_2on2(arr& p1, arr& p2, arr& normal, const arr& pts1, const arr& pts2);
double coll_2on3(arr& p1, arr& p2, arr& normal, const arr& pts1, const arr& pts2, const arr& center);
double coll_3on3(arr& p1, arr& p2, arr& normal, const arr& pts1, const arr& pts2, const arr& center);

} //namespace

stdOutPipe(rai::PairCollision)
