#pragma once

#include "mesh.h"

struct PairCollision : GLDrawer{
  //INPUTS
  const mlr::Mesh& mesh1;
  const mlr::Mesh& mesh2;
  mlr::Transformation& t1;
  mlr::Transformation& t2;
  double rad1, rad2; ///< only kinVector and glDraw account for this; the basic collision geometry (OUTPUTS below) is computed neglecting radii!!

  //OUTPUTS
  double distance; ///< negative=penetration
  arr p1, p2;      ///< closest points on the shapes
  arr normal;      ///< normal such that "<normal, p1-p2> = distance" is guaranteed (pointing from obj2 to obj1)
  arr simplex1;    ///< simplex on obj1 defining the collision geometry
  arr simplex2;    ///< simplex on obj2 defining the collision geometry
  arr dSimplex1, dSimplex2;
  arr m1, m2, eig1, eig2; ///< output of marginAnalysis: mean and eigenvalues of ALL point on the objs (not only simplex) that define the collision

  PairCollision(const mlr::Mesh& mesh1, const mlr::Mesh& mesh2,
                mlr::Transformation& t1, mlr::Transformation& t2,
                double rad1=0., double rad2=0.);

  void write(std::ostream& os) const;

  void glDraw(struct OpenGL&);

  double getDistance(){ return distance-rad1-rad2; }
  void kinVector(arr& y, arr& J,
                 const arr& Jp1, const arr& Jp2,
                 const arr& Jx1, const arr& Jx2);

  void kinDistance(arr& y, arr& J,
                   const arr& Jp1, const arr& Jp2);
  void kinDistance2(arr &y, arr& J,
                    const arr& JSimplex1, const arr& JSimplex2);

  void nearSupportAnalysis(double eps=1e-6); ///< analyses not only closest obj support (the simplex) but all points within a margin

private:
  double libccd_MPR(const mlr::Mesh& m1,const mlr::Mesh& m2); //calls ccdMPRPenetration of libccd
  double GJK_sqrDistance(); //gjk_distance of libGJK
  bool simplexType(uint i, uint j){ return simplex1.d0==i && simplex2.d0==j; } //helper
};

//return normals and closes points for 1-on-3 simplices or 2-on-2 simplices
double coll_1on3(arr& pInTri, arr& normal, const arr& pts1, const arr& pts2);
double coll_2on2(arr &p1, arr& p2, arr& normal, const arr &pts1, const arr &pts2);
double coll_2on3(arr &p1, arr& p2, arr& normal, const arr &pts1, const arr &pts2);
double coll_3on3(arr &p1, arr& p2, arr& normal, const arr &pts1, const arr &pts2);

stdOutPipe(PairCollision)
