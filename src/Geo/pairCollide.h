#include "mesh.h"

struct PairCollision : GLDrawer{
  const mlr::Mesh &mesh1;
  const mlr::Mesh& mesh2;
  const mlr::Transformation& t1;
  const mlr::Transformation& t2;

  double distance;
  arr p1, p2; //closest points
  arr normal;
  arr simplex1;
  arr simplex2;

  PairCollision(const mlr::Mesh& mesh1, const mlr::Mesh& mesh2,
                const mlr::Transformation& t1, const mlr::Transformation& t2);

  void write(std::ostream& os) const;

  void glDraw(struct OpenGL&);

  void kinVector(arr& y, arr& J,
                const arr& Jp1, const arr& Jp2,
                const arr& Jx1, const arr& Jx2);

  void kinDistance(arr& y, arr& J,
                   const arr& Jp1, const arr& Jp2,
                   const arr& Jx1, const arr& Jx2,
                   double rad1=0., double rad2=0.);

private:
  double GJK_libccd_penetration(const mlr::Mesh& m1,const mlr::Mesh& m2);
  double GJK_sqrDistance();
  bool simplexType(uint i, uint j){ return simplex1.d0==i && simplex2.d0==j; }
};

double coll_1on3(arr& pInTri, arr& normal, const arr& pts1, const arr& pts2);
double coll_2on2(arr &p1, arr& p2, arr& normal, const arr &pts1, const arr &pts2);

stdOutPipe(PairCollision)
