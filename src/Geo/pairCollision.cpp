/*  ------------------------------------------------------------------
    Copyright (c) 2011-2024 Marc Toussaint
    email: toussaint@tu-berlin.de

    This code is distributed under the MIT License.
    Please see <root-path>/LICENSE for details.
    --------------------------------------------------------------  */

#include "pairCollision.h"

#include "../Gui/opengl.h"
#include "../Optim/newton.h"
#include "../Algo/ann.h"
#include "../Core/util.h"

#ifdef RAI_GJK
extern "C" {
#  include "GJK/gjk.h"
}
#endif

#ifdef RAI_CCD
#  include "./ccd_rai/ccd.h"
#  include "./ccd_rai/quat.h"
#endif

#include "qhull.h"

#ifndef RAI_GJK
#  define FCLmode
#endif

namespace rai {

PairCollision::PairCollision(rai::Mesh& _mesh1, rai::Mesh& _mesh2, const rai::Transformation& _t1, const rai::Transformation& _t2, double rad1, double rad2)
  : t1(&_t1), t2(&_t2), rad1(rad1), rad2(rad2) {

  mesh1.V.referTo(_mesh1.V); mesh1.T.referTo(_mesh1.T);
  mesh2.V.referTo(_mesh2.V); mesh2.T.referTo(_mesh2.T);

  distance=-1.;

  //-- special cases: point to pcl
  if(mesh1.V.d0==1 && mesh2.V.d0>2 && !mesh2.T.N) {
    _mesh2.ensure_ann();

    arr x = mesh1.V;
    x.reshape(3);
    if(!t1->isZero() || !t2->isZero()) {
      rai::Transformation T = *t1 / *t2;
      x += T.pos.getArr();
    }

    arr sqrDists;
    uintA idx;
    uint K=20;
    _mesh2.ann->getkNN(sqrDists, idx, x, K);

    p2 = zeros(3);
    for(uint k=0; k<K; k++) p2 += _mesh2.V[idx(k)];
    p2 /= double(K);

    if(_mesh2.Vn.N) {
      normal = zeros(3);
      for(uint k=0; k<K; k++) normal += _mesh2.Vn[idx(k)]; //points from obj2 to obj1, as desired
      normal /= double(K);
    } else {
      normal.clear();
    }
//    normal.clear();

    p1 = x;

    if(!t2->isZero()) { //we computed everything relative to t2
      t2->applyOnPoint(p1);
      t2->applyOnPoint(p2);
      normal = t2->rot.getArr() * normal;
    }

    arr del = p1-p2;
    distance = length(del);
    if(normal.N && scalarProduct(del, normal)<.0) {
      distance *= -1.; //devision by distance below also flips normal
    }
    normal = del;
    if(fabs(distance)>1e-10) normal/=distance;
    CHECK_GE(rai::sign(distance) * scalarProduct(normal, p1-p2), -1e-10, ""); //just to check, as below
    simplex1 = ~p1;
    simplex2 = ~p2;
    return;
  }

  //-- special cases: point to multiple cvx parts
  if(mesh1.V.d0==1 && _mesh2.cvxParts.N) {
    arr x = mesh1.V;
    //directly call gjk for each part to get nearest
    Object_structure m1, m2;
    rai::Array<double*> Vhelp1 = getCarray(x);
    rai::Array<double*> Vhelp2 = getCarray(mesh2.V);
    if(!t1->isZero() || !t2->isZero()) {
      x.reshape(3);
      rai::Transformation T = *t1 / *t2;
      x += T.pos.getArr();
    }
    double dmin=-1.;
    int imin=0;
    //LOG(0) <<_mesh2.cvxParts <<_mesh2.V.d0 <<endl;
    for(uint i=0; i<_mesh2.cvxParts.N; i++) {
      int start = _mesh2.cvxParts(i);
      int end = i+1<_mesh2.cvxParts.N ? _mesh2.cvxParts(i+1)-1 : _mesh2.V.d0-1;
      CHECK_LE(start+1, end, "");
      m1.numpoints = 1;  m1.vertices = Vhelp1.p;  m1.rings=nullptr; //TODO: rings would make it faster
      m2.numpoints = end-start;  m2.vertices = Vhelp2.p+start;  m2.rings=nullptr;

      double d = gjk_distance(&m1, 0, &m2, 0, 0, 0, 0, 0);
      //cout <<" part " <<i <<" d:" <<d <<endl;
      if(d<dmin || dmin<0.) { imin=i; dmin=d; }
    }
    //cout <<" part " <<imin <<" is min" <<endl;
    //imin is the closest part... do the below with imin..
    int start = _mesh2.cvxParts(imin);
    int end = imin+1<(int)_mesh2.cvxParts.N ? _mesh2.cvxParts(imin+1)-1 : _mesh2.V.d0-1;
    mesh2.V.clear();
    mesh2.T.clear();
    mesh2.V = _mesh2.V({start, end});
  }

  //-- standard case

#ifdef FCLmode
  //THIS IS COSTLY! DO WITHIN THE SUPPORT FUNCTION?
  rai::Mesh M1(*mesh1); if(!t1->isZero()) t1->applyOnPointArray(M1.V);
  rai::Mesh M2(*mesh2); if(!t2->isZero()) t2->applyOnPointArray(M2.V);

  libccd(M1, M2, _ccdGJKIntersect);
#else
  GJK_sqrDistance();
#endif

  CHECK_EQ(distance, distance, "distance is nan");
//  libccd(M1, M2, _ccdMPRIntersect);
//  if(distance<1e-10) libccd(M1, M2, _ccdGJKIntersect);
//  if(distance<1e-10) GJK_sqrDistance();

#ifndef FCLmode
  if(distance<1e-10) { //WARNING: Setting this to zero does not work when using
    //THIS IS COSTLY! DO WITHIN THE SUPPORT FUNCTION?
    rai::Mesh M1(mesh1); if(!t1->isZero()) t1->applyOnPointArray(M1.V);
    rai::Mesh M2(mesh2); if(!t2->isZero()) t2->applyOnPointArray(M2.V);
    libccd(M1, M2, _ccdMPRPenetration);
  }
#else
  if(distance<0.) {
    libccd(M1, M2, _ccdMPRPenetration);
  }
#endif

  CHECK_EQ(p1.N, 3, "PairCollision failed");
  CHECK_EQ(p2.N, 3, "PairCollision failed");

  if(fabs(distance)<1e-10) { //exact touch: the GJK computed things, let's make them consistent
    p1 = p2 = .5*(p1+p2);
  }

  //ensure that the normal always points 'against obj1' (along p1-p2 relative to NON-penetration)
  if(rai::sign(distance) * scalarProduct(normal, p1-p2) < 0.)
    normal *= -1.;

  if(distance>1e-10) {
    CHECK_ZERO(length(normal) - 1., 1e-5, "");
  }

  CHECK_ZERO(scalarProduct(normal, p1-p2) - distance, 1e-5, "");

  CHECK_GE(rai::sign(distance) * scalarProduct(normal, p1-p2), -1e-10, "");

  //in current state, the rad1, rad2, have not been used at all!!
}

PairCollision::PairCollision(ScalarFunction func1, ScalarFunction func2, const arr& seed) {

  ScalarFunction f = [&func1, &func2](arr& g, arr& H, const arr& x) {
    arr g1, g2, H1, H2;
#if 0
    double d1 = func1(g1, H1, x);
    d1 += 1.; //boundingRadius1;
    CHECK_GE(d1, 0., "");
    H = (2.*d1)*H1 + 2.*(g1^g1);
    g = (2.*d1)*g1;
    double d2 = func2(g2, H2, x);
    d2 += 1.; //boundingRadius2;
    CHECK_GE(d2, 0., "");
    H += (2.*d2)*H2 + 2.*(g2^g2);
    g += (2.*d2)*g2;
    return d1*d1+d2*d2;
#else
    double d1 = func1(g1, H1, x);
    double d2 = func2(g2, H2, x);
    double dd = d1 - d2;
    H = H1 + H2 + (2.*dd)*(H1-H2) + 2.*((g1-g2)^(g1-g2));
    g = g1 + g2 + (2.*dd)*(g1-g2);
    return d1+d2+dd*dd;
#endif
  };

  arr x = seed;
  CHECK_EQ(x.N, 3, "");
  OptNewton newton(x, f, rai::OptOptions()
                   .set_verbose(0)
                   .set_stopTolerance(1e-4)
                   .set_maxStep(1.)
                   .set_damping(1e-10));
  newton.run();

  arr g1, g2;
  double d1 = func1(g1, NoArr, x);
  double d2 = func2(g2, NoArr, x);

//  cout <<"d1^2+d2^2:" <<newton.fx <<" d1:" <<d1 <<" d2:" <<d2 <<" (g1+g2):" <<sumOfSqr(g1+g2) <<endl;

//  CHECK_ZERO(d1-d2, 1e-4, "point should have equal distance (pos or negative) to both surfaces!");
//  CHECK_ZERO(sumOfSqr(g1+g2), 1e-4, "gradients should exactly oppose!");

  if(d1<d2) { //deeper into d1 -- use g1 as normal!
    normal = g1;
    normal /= length(normal);
    p1 = x - d1*normal;
    p2 = x + d1*normal;
    distance = 2.*d1;
  } else {
    normal = -g2;
    normal /= length(normal);
    p1 = x - d2*normal;
    p2 = x + d2*normal;
    distance = 2.*d2;
  }
  rad1=rad2=0.;

//  normal = p1-p2;
//  distance = d1+d2;
////  normal = g1-g2;
//  normal = g1-g2;
//  normal /= length(normal);

  if(rai::sign(distance) * scalarProduct(normal, p1-p2) < 0.)
    normal *= -1.;

  simplex1 = p1;  simplex1.reshape(1, 3);
  simplex2 = p2;  simplex2.reshape(1, 3);
}

void PairCollision::write(std::ostream& os) const {
  os <<"PairCollision INFO" <<endl;
  if(distance>0.) {
    os <<"  distance=" <<distance <<endl;
  } else {
    os <<"  penetration=" <<distance <<endl;
  }
  os <<"  witness points: " <<p1 <<"  " <<p2 <<endl;
  os <<"  simplex #: " <<simplex1.d0 <<"  " <<simplex2.d0 <<endl;
//  if(eig1.N || eig2.N) os <<"  EIG #: " <<eig1.d0<<'-' <<eig2.d0 <<endl;
}

#ifdef RAI_CCD
void support_mesh(const void* _obj, const ccd_vec3_t* dir, ccd_vec3_t* v) {
  rai::Mesh* m = (rai::Mesh*)_obj;
  uint vertex = m->support(dir->v);
  memmove(v->v, m->V.p+3*vertex, 3*m->V.sizeT);
}

void center_mesh(const void* obj, ccd_vec3_t* center) {
  rai::Mesh* m = (rai::Mesh*)obj;
  rai::Vector c = m->getCenter();
  memmove(center->v, &c.x, 3*sizeof(double));
}

bool _legal(double* a) {
  return a[0]==a[0]
         && a[1]==a[1]
         && a[2]==a[2]
         && -1e10<a[0] && a[0]<1e10
         && -1e10<a[1] && a[1]<1e10
         && -1e10<a[2] && a[2]<1e10;
}

bool _equal(double* a, double* b) {
  return a[0]==b[0] && a[1]==b[1] && a[2]==b[2];
}

bool _approxEqual(double* a, double* b) {
  return fabs(a[0]-b[0])<1e-10 && fabs(a[1]-b[1])<1e-10 && fabs(a[2]-b[2])<1e-10;
}

bool _zero(double* a) {
  return !a[0] && !a[1] && !a[2];
//  double eps=1e-10;
//  return a[0]>-eps && a[0]<eps && a[1]>-eps && a[1]<eps && a[2]>-eps && a[2]<eps;
}

void _getSimplex(arr& S, ccd_vec3_t* simplex, const arr& mean) {
  int select[4] = {-1, -1, -1, -1};
  uint n=0;
  bool sel;
  //first count and select
  for(uint i=0; i<4; i++) {
    double* s=simplex[i].v;
    if(!_legal(s)) continue;  //don't append nan!
    if(_equal(s, mean.p)) continue;
    sel=true;
    for(uint j=0; j<i; j++) if(_approxEqual(s, simplex[j].v)) { sel=false; break; }
    if(sel) select[n++] = i;
  }
  //then copy the selected
  S.resize(n, 3);
  for(uint i=0; i<n; i++) {
    memmove(&S(i, 0), simplex[select[i]].v, 3*S.sizeT);
  }

  /* SAFETY CHECK (slow!):
  if(S.d0>3){
    cout <<"select: " <<select[0] <<' ' <<select[1] <<' ' <<select[2] <<' ' <<select[3] <<endl;
    for(uint i=0;i<n;i++) cout <<simplex[i].v[0] <<' ' <<simplex[i].v[1] <<' ' <<simplex[i].v[2] <<endl;
    THROW("4-simplex does not work!:\n" <<S)
  }

  for(uint i=0;i<n;i++) for(uint j=i+1;j<n;j++){
    CHECK_GE(maxDiff(S[i], S[j]), 1e-10, "they are equal??");
  }
  */

}

void PairCollision::libccd(rai::Mesh& m1, rai::Mesh& m2, CCDmethod method) {
  ccd_t ccd;
  CCD_INIT(&ccd); // initialize ccd_t struct

  // set up ccd_t struct
  ccd.support1       = support_mesh; // support function for first object
  ccd.support2       = support_mesh; // support function for second object
  ccd.max_iterations = 100;   // maximal number of iterations
  ccd.epa_tolerance  = 1e-4;  // maximal tolerance for EPA part
  ccd.center1       = center_mesh; // support function for first object
  ccd.center2       = center_mesh; // support function for second object

  ccd_real_t _depth;
  ccd_vec3_t _dir, _pos, _v1, _v2;
  ccd_vec3_t simplex[8];

  bool penetration=false;

  if(method==_ccdMPRPenetration) {
    int ret = ccdMPRPenetrationRai(&m1, &m2, &ccd, &_depth, &_dir, &_pos, simplex);
    if(ret<0) {
      LOG(0) <<"WARNING: called MPR penetration for non intersecting meshes...";
      m1._support_vertex = rnd(m1.V.d0);
      m2._support_vertex = rnd(m2.V.d0);
      libccd(m1, m2, _ccdGJKIntersect);
      if(distance<0.) {
        LOG(0) <<"WARNING: but GJK says intersection";
        distance=0;
      }
      return;
    }

    penetration=true;

    p1.setCarray(_pos.v, 3);
    p2.setCarray(_pos.v, 3);
    normal.setCarray(_dir.v, 3);
    distance = -_depth;
    p1 += (.5*distance)*normal;
    p2 -= (.5*distance)*normal;

    if(distance>-1e-10) return; //minimal penetration -> simplices below are not robust

    //grab simplex points
    if(m1.V.d0==1) simplex1 = m1.V; //m1 is a point/sphere
    else _getSimplex(simplex1, simplex, m1.getMean());
    if(m2.V.d0==1) simplex2 = m2.V; //m2 is a point/sphere
    else _getSimplex(simplex2, simplex+4, m2.getMean());
    if(simplex1.d0>3) simplex1.resizeCopy(3, 3);
    if(simplex2.d0>3) simplex2.resizeCopy(3, 3);

  } else if(method==_ccdGJKPenetration) {
    int ret = ccdGJKPenetration(&m1, &m2, &ccd, &_depth, &_dir, &_pos);
    if(ret<0) {
      LOG(0) <<"WARNING: called MPR penetration for non intersecting meshes...";
      m1._support_vertex = rnd(m1.V.d0);
      m2._support_vertex = rnd(m2.V.d0);
      libccd(m1, m2, _ccdGJKIntersect);
      if(distance<0.) {
        LOG(0) <<"WARNING: but GJK says intersection";
        distance=0;
      }
      return;
    }

    penetration=true;

    p1.setCarray(_pos.v, 3);
    p2.setCarray(_pos.v, 3);
    normal.setCarray(_dir.v, 3);
    distance = -_depth;
    p1 += (.5*distance)*normal;
    p2 -= (.5*distance)*normal;

    if(distance>-1e-10) return; //minimal penetration -> simplices below are not robust

    //grab simplex points
//      simplex1 = ~p1; //m1 is a point/sphere
//      simplex2 = ~p2; //m2 is a point/sphere

  } else if(method==_ccdGJKIntersect) {
    int ret = ccdGJKIntersectRai(&m1, &m2, &ccd, &_v1, &_v2, simplex);
    if(ret) {
      distance = -1.;
      return;
    }

    penetration = false;

    p1.setCarray(_v1.v, 3);
    p2.setCarray(_v2.v, 3);
    normal = p1-p2;
    distance = length(normal);
    if(distance>1e-10) normal/=distance;

    //grab simplex points
    arr mean = zeros(3);
    _getSimplex(simplex1, simplex, mean);
    _getSimplex(simplex2, simplex+4, mean);
    if(simplex1.d0>3) simplex1.resizeCopy(3, 3);
    if(simplex2.d0>3) simplex2.resizeCopy(3, 3);
  } else {
    NIY;
  }

  //compute witness points
  double d=0.;
  if(simplexType(1, 1)) {
    p1=simplex1[0];
    p2=simplex2[0];
    normal = p1-p2;
    distance = length(normal);
    if(distance>1e-10) normal/=distance;
    d = distance;
  } else if(simplexType(1, 2)) {
    double s;
    p1 = simplex1[0];
    d=coll_1on2(p2, normal, s, simplex1, simplex2);
  } else if(simplexType(2, 1)) {
    double s;
    p2 = simplex2[0];
    d=coll_1on2(p1, normal, s, simplex2, simplex1);
  } else if(simplexType(1, 3)) {
    p1 = simplex1[0];
    d=coll_1on3(p2, normal, simplex1, simplex2);
  } else if(simplexType(3, 1)) {
    p2 = simplex2[0];
    d=coll_1on3(p1, normal, simplex2, simplex1);
  } else if(simplexType(2, 2)) {
    d=coll_2on2(p1, p2, normal, simplex1, simplex2);
  } else if(simplexType(2, 3)) {
    d=coll_2on3(p1, p2, normal, simplex1, simplex2, arr(_pos.v, 3, true));
  } else if(simplexType(3, 2)) {
    d=coll_2on3(p2, p1, normal, simplex2, simplex1, arr(_pos.v, 3, true));
  } else if(simplexType(3, 3)) {
    d=coll_3on3(p2, p1, normal, simplex2, simplex1, mean(simplex1)); //arr(_pos.v, 3));
  } else HALT("simplex types " <<simplex1.d0 <<' ' <<simplex2.d0 <<" not handled");
  CHECK_EQ(p1.N, 3, "PairCollision failed")
  CHECK_EQ(p2.N, 3, "PairCollision failed")

  //  CHECK_ZERO(_depth - fabs(d), 1e-4, ""); //compare depth by ccd with ours
  if(fabs(d) < 1e-10) {
    checkNan(p1);
    checkNan(p2);
  }

  if(!penetration)
    distance=fabs(d);
  else
    distance=-fabs(d);

//  }else if(method==_ccdGJKSeparate){
////    intersect = !ccdGJKSeparate(&m1, &m2, &ccd, &v1, &v2);
//    NIY
//  }else if(method==_ccdMPRIntersect){
//    int ret = ccdMPRIntersect(&m1, &m2, &ccd);
//    if(ret) distance=1.;
//    else distance=-1.;
//    return;
//  }else if(method==_ccdGJKPenetration){
////    intersect = !ccdGJKPenetrationRai(&m1, &m2, &ccd, &depth, &_dir, &_pos);
//    NIY
//  }
//  HALT("should not be here");
}
#else
void PairCollision::libccd(rai::Mesh& m1, rai::Mesh& m2, CCDmethod method) {
  NICO
}
#endif

void PairCollision::GJK_sqrDistance() {
#ifdef RAI_GJK
  // convert meshes to 'Object_structures'
  Object_structure m1, m2;
  rai::Array<double*> Vhelp1 = getCarray(mesh1.V);
  rai::Array<double*> Vhelp2 = getCarray(mesh2.V);
  m1.numpoints = mesh1.V.d0;  m1.vertices = Vhelp1.p;  m1.rings=nullptr; //TODO: rings would make it faster
  m2.numpoints = mesh2.V.d0;  m2.vertices = Vhelp2.p;  m2.rings=nullptr;

  // convert transformations to affine matrices
  arr T1, T2;
  rai::Array<double*> Thelp1, Thelp2;
  if(!!t1) {  T1=t1->getAffineMatrix();  Thelp1 = getCarray(T1);  }
  if(!!t2) {  T2=t2->getAffineMatrix();  Thelp2 = getCarray(T2);  }

  // call GJK
  simplex_point simplex;
  p1.resize(3).setZero();
  p2.resize(3).setZero();
  gjk_distance(&m1, Thelp1.p, &m2, Thelp2.p, p1.p, p2.p, &simplex, 0);

  normal = p1-p2;
  distance = length(normal);
  if(distance>1e-10) normal /= distance;

  //grab simplex points
  simplex1.resize(0, 3);
  simplex2.resize(0, 3);
  if(simplex.npts>=1) {
    simplex1.append(arr(simplex.coords1[0], 3, true));
    simplex2.append(arr(simplex.coords2[0], 3, true));
  }
  if(simplex.npts>=2) {
    if(simplex.simplex1[1]!=simplex.simplex1[0]) simplex1.append(arr(simplex.coords1[1], 3, true));
    if(simplex.simplex2[1]!=simplex.simplex2[0]) simplex2.append(arr(simplex.coords2[1], 3, true));
  }
  if(simplex.npts>=3) {
    if(simplex.simplex1[2]!=simplex.simplex1[0] && simplex.simplex1[2]!=simplex.simplex1[1]) simplex1.append(arr(simplex.coords1[2], 3, true));
    if(simplex.simplex2[2]!=simplex.simplex2[0] && simplex.simplex2[2]!=simplex.simplex2[1]) simplex2.append(arr(simplex.coords2[2], 3, true));
  }
#else
  NICO
#endif
}

void PairCollision::kinDistance(arr& y, arr& J,
                                const arr& Jp1, const arr& Jp2) {
  y = arr{distance-rad1-rad2};
  if(!!J) {
    arr Jdiff = Jp1 - Jp2;
    J = ~normal*Jdiff;
  }
}

void PairCollision::kinNormal(arr& y, arr& J,
                              const arr& Jp1, const arr& Jp2,
                              const arr& Jx1, const arr& Jx2) {
  y = normal;
  if(!!J) {
    if(simplexType(1, 3)) {
      J = crossProduct(Jx2, y);
    } else if(simplexType(3, 1)) {
      J = crossProduct(Jx1, y);
    } else if(simplexType(2, 2)) {
      arr a = simplex1[1]-simplex1[0];  a/=length(a);
      arr b = simplex2[1]-simplex2[0];  b/=length(b);
      double ab=scalarProduct(a, b);
      if(1.-ab*ab>1e-8) { //the edges are not colinear
        double nn = ::sqrt(1.-ab*ab);
        double sign = rai::sign(scalarProduct(normal, crossProduct(b, a)));
        J = ((sign/nn) * (eye(3, 3) - normal*~normal)) * (skew(b) * crossProduct(Jx1, a) - skew(a) * crossProduct(Jx2, b));
      }
    } else if(simplexType(2, 1)) {
      y = p1 - p2;
      J = Jp1 - Jp2;
      normalizeWithJac(y, J);
      arr a = simplex1[1]-simplex1[0];  a/=length(a);
      arr aa = a^a;
      J -= aa*J;
      J += aa*crossProduct(Jx1, y);
    } else if(simplexType(1, 2)) {
      y = p1 - p2;
      J = Jp1 - Jp2;
      normalizeWithJac(y, J);
      arr b = simplex2[1]-simplex2[0];  b/=length(b);
      arr bb = b^b;
      J -= bb*J;
      J += bb*crossProduct(Jx2, y);
    } else if(simplexType(1, 1)) {
      y = p1 - p2;
      J = Jp1 - Jp2;
      normalizeWithJac(y, J);
    } else if(simplexType(2, 3)) {
      J = Jp1;
      J.setZero();
    } else if(simplexType(3, 2)) {
      J = Jp1;
      J.setZero();
    } else if(simplexType(3, 3)) {
      J = Jp1;
      J.setZero();
    } else NIY;
    checkNan(J);
  }
}

void PairCollision::kinVector(arr& y, arr& J,
                              const arr& Jp1, const arr& Jp2,
                              const arr& Jx1, const arr& Jx2) {
  y = p1 - p2;
  if(!!J) {
    J = Jp1 - Jp2;
    if(simplexType(1, 3)) {
      J = (normal^normal)*J;
      J += crossProduct(Jx2, p1-p2);
    }
    if(simplexType(3, 1)) {
      J = (normal^normal)*J;
      J += crossProduct(Jx1, p1-p2);
    }
    if(simplexType(2, 2)) {
      J = (normal^normal)*J;
      arr a = simplex1[1]-simplex1[0];  a/=length(a);
      arr b = simplex2[1]-simplex2[0];  b/=length(b);
      double ab=scalarProduct(a, b);
      if(1.-ab*ab>1e-8) { //the edges are not colinear
        double nn = ::sqrt(1.-ab*ab);
        double sign = rai::sign(scalarProduct(normal, crossProduct(b, a)));
        J += ((distance * sign/nn) * (eye(3, 3) - normal*~normal)) * (skew(b) * crossProduct(Jx1, a) - skew(a) * crossProduct(Jx2, b));
      }
    }
    if(simplexType(2, 1)) {
      arr a = simplex1[1]-simplex1[0];  a/=length(a);
      arr aa = a^a;
      J -= aa*J;
      J += aa*crossProduct(Jx1, p1-p2);
    }
    if(simplexType(1, 2)) {
      arr b = simplex2[1]-simplex2[0];  b/=length(b);
      arr bb = b^b;
      J -= bb*J;
      J += bb*crossProduct(Jx2, p1-p2);
    }
    checkNan(J);
  }

  //-- account for radii
  if(rad1>0. || rad2>0.) {
    double rad=rad1+rad2;
    double eps = 1e-6;
    double fac = (distance-rad)/(distance+eps);
    if(!!J) {
      arr d_fac = ((1.-fac)/(distance+eps)) *((~normal)*J);
      J = J*fac + y.reshape(3, 1)*d_fac;
      y.reshape(3);
      checkNan(J);
    }
    y *= fac;
  }
}

void PairCollision::kinPointP1(arr& y, arr& J, const arr& Jp1, const arr& Jp2, const arr& Jx1, const arr& Jx2) {
  y = p1;
  if(!!J) {
    J = Jp1;
    if(simplexType(3, 1)) {
      J = Jp2;
      J += (normal^normal)*(Jp1-Jp2);
      J += crossProduct(Jx1, p1-p2);
    }
    if(simplexType(2, 2)) {
      arr a = simplex1[1]-simplex1[0];  a/=length(a);
      arr b = simplex2[1]-simplex2[0];  b/=length(b);
      double ab=scalarProduct(a, b);

      J = Jp1;
      arr c = b*ab-a;
      double ac = scalarProduct(a, c);
      if(fabs(ac)>1e-10) { //otherwise the Jacobian is singular...
        J += ((1./ac) * (a^c))*(Jp2-Jp1);

        arr x = p1-p2;
        arr Jc = (b*~b-eye(3, 3))* crossProduct(Jx1, a) + (ab*eye(3, 3) + b*~a - 2.*a*~b)*crossProduct(Jx2, b);
        J += (1./ac) * scalarProduct(c, x) * (eye(3, 3) - (1./ac)*a*~c) * crossProduct(Jx1, a);
        J -= (1./ac) * (a * ~x) * (eye(3, 3) - (1./ac)*c*~a) * Jc;
      }
    }
    if(simplexType(2, 1)) {
      arr a = simplex1[1]-simplex1[0];  a/=length(a);
      arr aa = a^a;
      J += aa*(Jp2-Jp1);
      J += aa*crossProduct(Jx1, p1-p2);
    }
    checkNan(J);
  }

  //-- account for radii
  if(rad1>0.) {
    arr norm, Jnorm;
    if(!J) Jnorm.setNoArr();
    kinNormal(norm, Jnorm, Jp1, Jp2, Jx1, Jx2);
    y -= rad1 * norm;
    if(!!J) J -= rad1 * Jnorm;
  }
}

void PairCollision::kinPointP2(arr& y, arr& J, const arr& Jp1, const arr& Jp2, const arr& Jx1, const arr& Jx2) {
  y = p2;
  if(!!J) {
    J = Jp2;
    if(simplexType(1, 3)) {
      J = Jp1;
      J += (normal^normal)*(Jp2-Jp1);
      J += crossProduct(Jx2, p2-p1);
    }
    if(simplexType(2, 2)) {
      arr a = simplex2[1]-simplex2[0];  a/=length(a);
      arr b = simplex1[1]-simplex1[0];  b/=length(b);
      double ab=scalarProduct(a, b);

      J = Jp2;
      arr c = b*ab-a;
      double ac = scalarProduct(a, c);
      if(fabs(ac)>1e-10) { //otherwise the Jacobian is singular...
        J += ((1./ac) * (a^c))*(Jp1-Jp2);

        arr x = p2-p1;
        arr Jc = (b*~b-eye(3, 3))* crossProduct(Jx2, a) + (ab*eye(3, 3) + b*~a - 2.*a*~b)*crossProduct(Jx1, b);
        J += (1./ac) * scalarProduct(c, x) * (eye(3, 3) - (1./ac)*a*~c) * crossProduct(Jx2, a);
        J -= (1./ac) * (a * ~x) * (eye(3, 3) - (1./ac)*c*~a) * Jc;
      }
    }
    if(simplexType(1, 2)) {
      arr b = simplex2[1]-simplex2[0];  b/=length(b);
      arr bb = b^b;
      J += bb*(Jp1-Jp2);
      J += bb*crossProduct(Jx2, p2-p1);
    }
    checkNan(J);
  }

  //-- account for radii
  if(rad2>0.) {
    arr norm, Jnorm;
    if(!J) Jnorm.setNoArr();
    kinNormal(norm, Jnorm, Jp1, Jp2, Jx1, Jx2);
    y += rad2 * norm;
    if(!!J) J += rad2 * Jnorm;
  }
}

void PairCollision::kinCenter(arr& y, arr& J, const arr& Jp1, const arr& Jp2, const arr& Jx1, const arr& Jx2) {
  arr _p1, _p2, JP1, JP2;
  if(!J) { JP1.setNoArr(); JP2.setNoArr(); }
  kinPointP1(_p1, JP1, Jp1, Jp2, Jx1, Jx2);
  kinPointP2(_p2, JP2, Jp1, Jp2, Jx1, Jx2);
  y = .5 * (_p1 + _p2);
  if(!!J) J = .5 * (JP1 + JP2);
}

void PairCollision::nearSupportAnalysis(double eps) {
  rai::Mesh M1(mesh1); t1->applyOnPointArray(M1.V);
  rai::Mesh M2(mesh2); t2->applyOnPointArray(M2.V);

  //get the set of vertices that are maximal/minimal in normal direction
  //(these might be more than the simplex: esp 4 points for box)
  uintA pts1, pts2;
  M1.supportMargin(pts1, -normal, eps);
  M2.supportMargin(pts2, normal, eps);

  //collect these points in S1 and S2; accounts for radius
  arr S1, S2;
  for(uint i:pts1) S1.append(M1.V[i] - rad1*normal);
  for(uint i:pts2) S2.append(M2.V[i] + rad2*normal);
  S1.reshape(pts1.N, 3);
  S2.reshape(pts2.N, 3);

  //centers
//  m1 = mean(S1);
//  m2 = mean(S2);
  arr cen = .5*(mean(S1) + mean(S2));

  //get projection onto the normal plane
  rai::Quaternion R;
  R.setDiff(normal, Vector_z);
  arr P = R.getArr();
  P.delRows(2);

  //compute the convex intersection polygon of projected points, and unproject back
  poly = convconv_intersect(S1*~P, S2*~P);
  poly = poly*P;
  for(uint i=0; i<poly.d0; i++) poly[i] += cen - ~P*P*cen;

//  simplex1 = simplex2 = C*P;
//  for(uint i=0; i<simplex1.d0; i++) simplex1[i] += p1 - ~P*P*p1;
//  for(uint i=0; i<simplex2.d0; i++) simplex2[i] += p2 - ~P*P*p2;

//  poly = simplex1 + simplex2;
//  poly *= .5;

  //for each edge of the polygon, compute an outward pointing normal (to define inequalities)
  polyNorm.resizeAs(poly);
  uint n=polyNorm.d0;
  for(uint i=0; i<n; i++) {
    arr norm = crossProduct(poly[(i+1)%n] - poly[i], normal);
    polyNorm[i] = norm/length(norm);
  }
}

//===========================================================================

double coll_1on2(arr& p2, arr& normal, double& s, const arr& pts1, const arr& pts2) {
  CHECK(pts1.nd==2 && pts1.d0==1 && pts1.d1==3, "I need a set of 1 pts1");
  CHECK(pts2.nd==2 && pts2.d0==2 && pts2.d1==3, "I need a set of 2 pts2");

  rai::Vector p1(pts1.p);
  rai::Vector p20(pts2.p);
  rai::Vector p21(pts2.p+3);
  rai::Vector _p2;

  rai::Vector b = p21-p20;

  s = (p1-p20) * b;
  s /= b.lengthSqr();

  if(s<=0.) { //1on1 with pts2[0]
    _p2 = p20;
  } else if(s>=1.) { //1on1 with pts2[1]
    _p2 = p21;
  } else {
    _p2 = p20;
    _p2 += s*b;
  }

  rai::Vector _normal = p1-_p2;
  double d = _normal.length();
  if(d>1e-10) _normal /= d;

  p2.setCarray(_p2.p(), 3);
  normal.setCarray(_normal.p(), 3);
  CHECK_EQ(d, d, "distance is nan; p1:" <<p1 <<" p20:" <<p20 <<" p21" <<p21 <<" p2:" <<_p2 <<" normal:" <<_normal);
  return d;
}

double coll_1on3(arr& p2, arr& normal, const arr& pts1, const arr& pts2) {
  CHECK(pts1.nd==2 && pts1.d0==1 && pts1.d1==3, "I need a set of 1 pts1");
  CHECK(pts2.nd==2 && pts2.d0==3 && pts2.d1==3, "I need a set of 3 pts2");

  //choose pts1-vertex as origin
  arr tri = pts2;
  for(uint i=0; i<tri.d0; i++) tri[i] -= pts1[0];

  //compute normal of tri (plane eq first three parameters)
  arr a=tri[1]-tri[0], b=tri[2]-tri[0];
  normal = crossProduct(b, a);
  normal /= length(normal);

  //find plane eq offset parameter
  double d = scalarProduct(normal, tri[0]);

  p2 = d*normal + pts1[0];
  checkNan(p2);

  return d;
}

double coll_2on2(arr& p1, arr& p2, arr& normal, const arr& pts1, const arr& pts2) {
  CHECK(pts1.nd==2 && pts1.d0==2 && pts1.d1==3, "I need a set of 2 pts1");
  CHECK(pts2.nd==2 && pts2.d0==2 && pts2.d1==3, "I need a set of 2 pts2");

  rai::Vector p10(pts1.p);
  rai::Vector p11(pts1.p+3);
  rai::Vector p20(pts2.p);
  rai::Vector p21(pts2.p+3);
  rai::Vector _p1, _p2;

  //compute normal
  rai::Vector a=p11-p10, b=p21-p20;
  rai::Vector _normal = b ^ a;
  double n_len = _normal.length();
  if(n_len<1e-10) {
    double s;
    p1.setCarray(p10.p(), 3);
    arr PTS1 = p1;
    PTS1.reshape(1, 3);
    return coll_1on2(p2, normal, s, PTS1, pts2);
  }
  _normal /= n_len;

  //distance
  double d = _normal * (p20-p10);

  //2nd plane
  rai::Vector n = _normal ^ b;
  double t = ((p20-p10) * n)/(a*n);//https://en.wikipedia.org/wiki/Line%E2%80%93plane_intersection

  _p1 = p10 + t*a;
  _p2 = _p1 + d*_normal;

  p1.setCarray(_p1.p(), 3);
  p2.setCarray(_p2.p(), 3);
  normal.setCarray(_normal.p(), 3);
  return d;
}

double coll_2on3(arr& p1, arr& p2, arr& normal, const arr& pts1, const arr& pts2, const arr& center) {
  CHECK(pts1.nd==2 && pts1.d0==2 && pts1.d1==3, "I need a set of 2 pts1");
  CHECK(pts2.nd==2 && pts2.d0==3 && pts2.d1==3, "I need a set of 3 pts2");
  //collide center on line to get p1:
  arr cen = center;
  cen.reshape(1, 3);
  double s;
  coll_1on2(p1, normal, s, cen, pts1);
//  p1 = .5*(pts1[0]+pts1[1]); //take center of line segment as single point
  p1.reshape(1, 3);
  double d = coll_1on3(p2, normal, p1, pts2);
  p1.reshape(3);
  CHECK_EQ(d, d, "distance is nan");
  return d;
}

double coll_3on3(arr& p1, arr& p2, arr& normal, const arr& pts1, const arr& pts2, const arr& center) {
  CHECK(pts1.nd==2 && pts1.d0==3 && pts1.d1==3, "I need a set of 3 pts1");
  CHECK(pts2.nd==2 && pts2.d0==3 && pts2.d1==3, "I need a set of 3 pts2");
  //collide center on tri1 to get p1:
  arr cen = center;
  cen.reshape(1, 3);
  coll_1on3(p1, normal, cen, pts1);
//  p1 = (1./3.)*(pts1[0]+pts1[1]+pts1[2]); //take center of line tri as single point
  p1.reshape(1, 3);
  double d = coll_1on3(p2, normal, p1, pts2);
  p1.reshape(3);
  return d;
}

//===========================================================================

PclCollision::PclCollision(const arr& _x, ANN& ann,
                           const rai::Transformation& t1, const arr& Jp1, const arr& Jx1,
                           const rai::Transformation& t2, const arr& Jp2, const arr& Jx2,
                           double _rad1, double _rad2, bool returnVector) {

  Vector x(_x);

  //-- transform x only relative to pcl
  if(!t1.isZero() || !t2.isZero()) {
    rai::Transformation T = t1 / t2;
    x = T * x;
  }

  arr sqrDists;
  uintA idx;
  uint K=10;
  ann.getkNN(sqrDists, idx, x.getArr(), K);

  arr normal;
  double y_dist=0;
  Vector y_vec(0);
  arr J_dist, J_vec;
  x = t2.rot * x; //relative to center2, but in world axes!
  for(uint k=0; k<K; k++) {
    Vector z = ann.X[idx(k)];
    z = t2.rot*z;
    Vector del = x - z;
    double d = del.length();
    normal = del.getArr();
    if(d>1e-10) normal /= d;
    if(!J_dist.N) {
      J_dist = ~normal * (Jp1 - Jp2 - crossProduct(Jx2, z.getArr()));
      J_vec = (Jp1 - Jp2 - crossProduct(Jx2, z.getArr()));
    } else {
      J_dist += ~normal * (Jp1 - Jp2 - crossProduct(Jx2, z.getArr()));
      J_vec += (Jp1 - Jp2 - crossProduct(Jx2, z.getArr()));
    }
    y_dist += d;
    y_vec += del;
  }
  y_dist /= double(K);
  y_vec /= double(K);
  J_dist /= double(K);
  J_vec /= double(K);

  if(!returnVector) {
    y = arr{y_dist-_rad1-_rad2};
    if(!!J) J = J_dist;
  } else {
    y = y_vec.getArr();
    if(!!J) J = J_vec;
  }
}

} //namespace
