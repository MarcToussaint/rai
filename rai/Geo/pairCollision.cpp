/*  ------------------------------------------------------------------
    Copyright (c) 2017 Marc Toussaint
    email: marc.toussaint@informatik.uni-stuttgart.de

    This code is distributed under the MIT License.
    Please see <root-path>/LICENSE for details.
    --------------------------------------------------------------  */

#include "pairCollision.h"

#define RAI_extern_GJK
#ifdef RAI_extern_GJK
extern "C" {
#  include "GJK/gjk.h"
}
#endif

#include <Gui/opengl.h>
#include "ccd/ccd.h"
#include "ccd/quat.h"
#include <Geo/qhull.h>

PairCollision::PairCollision(const rai::Mesh &_mesh1, const rai::Mesh &_mesh2, rai::Transformation &_t1, rai::Transformation &_t2, double rad1, double rad2)
  : mesh1(&_mesh1), mesh2(&_mesh2), t1(&_t1), t2(&_t2), rad1(rad1), rad2(rad2) {
  
  double d2 = GJK_sqrDistance();
  
  if(d2>1e-10) {
    distance = sqrt(d2);
  } else {
    //THIS IS COSTLY? DO WITHIN THE SUPPORT FUNCTION!
    rai::Mesh M1(*mesh1); t1->applyOnPointArray(M1.V);
    rai::Mesh M2(*mesh2); t2->applyOnPointArray(M2.V);
    distance = - libccd_MPR(M1, M2);
  }

  if(fabs(distance)<1e-10){ //exact touch: the GJK computed things, let's make them consisten
    p1 = p2 = .5*(p1+p2);
  }
  
  //ensure that the normal always points 'against obj1' (along p1-p2 relative to NON-penetration)
  if(rai::sign(distance) * scalarProduct(normal, p1-p2) < 0.)
    normal *= -1.;

  if(distance>1e-10){
    CHECK_ZERO(length(normal) - 1., 1e-5, "");
  }

  CHECK_ZERO(scalarProduct(normal, p1-p2) - distance, 1e-5, "");

  CHECK_GE(rai::sign(distance) * scalarProduct(normal, p1-p2), -1e-10, "");
  
  //in current state, the rad1, rad2, have not been used at all!!
}

void PairCollision::write(std::ostream &os) const {
  os <<"PairCollide INFO" <<endl;
  if(distance>0.) {
    os <<"  distance=" <<distance <<endl;
  } else {
    os <<"  penetration=" <<distance <<endl;
  }
  os <<"  closest points: " <<p1 <<"  " <<p2 <<endl;
  os <<"  simplex #: " <<simplex1.d0 <<"  " <<simplex2.d0 <<endl;
//  if(eig1.N || eig2.N) os <<"  EIG #: " <<eig1.d0<<'-' <<eig2.d0 <<endl;
}

void support_mesh(const void *_obj, const ccd_vec3_t *_dir, ccd_vec3_t *v) {
  rai::Mesh *m = (rai::Mesh*)_obj;
  arr dir(_dir->v, 3, true);
  uint vertex = m->support(dir);
  memmove(v->v, &m->V(vertex, 0), 3*m->V.sizeT);
}

void center_mesh(const void *obj, ccd_vec3_t *center) {
  rai::Mesh *m = (rai::Mesh*)obj;
  rai::Vector c = m->getCenter();
  memmove(center->v, &c.x, 3*sizeof(double));
}

double PairCollision::libccd_MPR(const rai::Mesh& m1,const rai::Mesh& m2) {
  ccd_t ccd;
  CCD_INIT(&ccd); // initialize ccd_t struct
  
  // set up ccd_t struct
  ccd.support1       = support_mesh; // support function for first object
  ccd.support2       = support_mesh; // support function for second object
  ccd.max_iterations = 100;   // maximal number of iterations
  ccd.epa_tolerance  = 1e-4;  // maximal tolerance fro EPA part
  ccd.center1       = center_mesh; // support function for first object
  ccd.center2       = center_mesh; // support function for second object
  
  ccd_real_t _depth;
  ccd_vec3_t _dir, _pos;
  ccd_vec3_t simplex[8];
  
  //    S.m1.translate(randn(3));
  //    int intersect = ccdGJKIntersect(&S.m1, &S.m2, &ccd, &v1, &v2);
  //  int non_intersect = ccdGJKPenetration(&m1, &m2, &ccd, &depth, &_dir, &_pos);
  //    int intersect = ccdMPRIntersect(&S.m1, &S.m2, &ccd);
  int ret = ccdMPRPenetration(&m1, &m2, &ccd, &_depth, &_dir, &_pos, simplex);
  
  if(ret<0) return 0.; //no intersection
  //res == 1  // Touching contact on portal's v1.
  //res == 2  // Origin lies on v0-v1 segment.
  //res == 0  // penetration

  normal.setCarray(_dir.v, 3);

  simplex1.resize(0,3);
  simplex2.resize(0,3);
  arr c1=m1.getMean();
  arr c2=m2.getMean();
  arr s(3);

  if(m1.V.d0==1) simplex1.append(c1); //m1 is a point/sphere
  if(m2.V.d0==1) simplex2.append(c2); //m1 is a point/sphere

  //grab simplex points
  bool append;
  for(uint i=0; i<4; i++) {
    s = arr(simplex[0+i].v, 3);
    append=true;
    if(sqrDistance(s, c1)<1e-10) append=false;
    for(uint i=0; i<simplex1.d0; i++) if(sqrDistance(s, simplex1[i])<1e-10) { append=false; break; }
    if(append) simplex1.append(s);
    
    s = arr(simplex[4+i].v, 3);
    append=true;
    if(sqrDistance(s, c2)<1e-10) append=false;
    for(uint i=0; i<simplex2.d0; i++) if(sqrDistance(s, simplex2[i])<1e-10) { append=false; break; }
    if(append) simplex2.append(s);
  }
  
  //compute witness points
  double d=0.;
  if(simplexType(1, 3)) {
    p1 = simplex1[0];
    d=coll_1on3(p2, normal, simplex1, simplex2);
  }
  if(simplexType(3, 1)) {
    p2 = simplex2[0];
    d=coll_1on3(p1, normal, simplex2, simplex1);
  }
  if(simplexType(2, 2)) {
    d=coll_2on2(p1, p2, normal, simplex1, simplex2);
  }
  if(simplexType(2, 3)) {
    d=coll_2on3(p1, p2, normal, simplex1, simplex2, arr(_pos.v, 3));
  }
  if(simplexType(3, 2)) {
    d=coll_2on3(p2, p1, normal, simplex2, simplex1, arr(_pos.v, 3));
  }
  if(simplexType(3, 3)) {
    d=coll_3on3(p2, p1, normal, simplex2, simplex1, arr(_pos.v, 3));
  }
  
//  CHECK_ZERO(_depth - fabs(d), 1e-4, ""); //compare depth by ccd with ours

  return fabs(d);
}

double PairCollision::GJK_sqrDistance() {
  // convert meshes to 'Object_structures'
  Object_structure m1,m2;
  rai::Array<double*> Vhelp1, Vhelp2;
  m1.numpoints = mesh1->V.d0;  m1.vertices = mesh1->V.getCarray(Vhelp1);  m1.rings=NULL; //TODO: rings would make it faster
  m2.numpoints = mesh2->V.d0;  m2.vertices = mesh2->V.getCarray(Vhelp2);  m2.rings=NULL;
  
  // convert transformations to affine matrices
  arr T1,T2;
  rai::Array<double*> Thelp1, Thelp2;
  if(!!t1) {  T1=t1->getAffineMatrix();  T1.getCarray(Thelp1);  }
  if(!!t2) {  T2=t2->getAffineMatrix();  T2.getCarray(Thelp2);  }
  
  // call GJK
  simplex_point simplex;
  p1.resize(3).setZero();
  p2.resize(3).setZero();
  double d2 = gjk_distance(&m1, Thelp1.p, &m2, Thelp2.p, p1.p, p2.p, &simplex, 0);
  
  normal = p1-p2;
  double l = length(normal);
  if(l){
    normal /= l;
  }
  
  //grab simplex points
  simplex1.resize(0,3);
  simplex2.resize(0,3);
  if(simplex.npts>=1) {
    simplex1.append(arr(simplex.coords1[0], 3));
    simplex2.append(arr(simplex.coords2[0], 3));
  }
  if(simplex.npts>=2) {
    if(simplex.simplex1[1]!=simplex.simplex1[0]) simplex1.append(arr(simplex.coords1[1], 3));
    if(simplex.simplex2[1]!=simplex.simplex2[0]) simplex2.append(arr(simplex.coords2[1], 3));
  }
  if(simplex.npts>=3) {
    if(simplex.simplex1[2]!=simplex.simplex1[0] && simplex.simplex1[2]!=simplex.simplex1[1]) simplex1.append(arr(simplex.coords1[2], 3));
    if(simplex.simplex2[2]!=simplex.simplex2[0] && simplex.simplex2[2]!=simplex.simplex2[1]) simplex2.append(arr(simplex.coords2[2], 3));
  }
  
  return d2;
}

void PairCollision::glDraw(OpenGL &) {
#ifdef RAI_GL
  arr P1=p1, P2=p2;
  if(rad1>0.) P1 -= rad1*normal;
  if(rad2>0.) P2 += rad2*normal;
  
  glColor(0., 1., 0., 1.);
  glDrawDiamond(P1(0), P1(1), P1(2), .005, .005, .005);
  for(uint i=0; i<simplex1.d0; i++) simplex1[i] -= rad1*normal;
  glDrawPolygon(simplex1);
  for(uint i=0;i<simplex1.d0;i++) simplex1[i] += rad1*normal;

  glColor(0., 0., 1., 1.);
  glDrawDiamond(P2(0), P2(1), P2(2), .005, .005, .005);
  for(uint i=0; i<simplex2.d0; i++) simplex2[i] += rad2*normal;
  glDrawPolygon(simplex2);
  for(uint i=0;i<simplex2.d0;i++) simplex2[i] -= rad2*normal;

  glColor(1., 0., 0., 1.);
  glLineWidth(2.f);
  glDrawProxy(P1, P2, .02);
  glLineWidth(1.f);
  glLoadIdentity();
  
  if(poly.N) {
    glColor(0., 1., 1., 1.);
    glLineWidth(1.f);
    glDrawPolygon(poly);
    uint n=poly.d0;
    for(uint i=0; i<n; i++) {
      rai::Transformation T;
      T.pos = .5 *(poly[(i+1)%n] + poly[i]);
      T.rot.setDiff(Vector_x, polyNorm[i]);
//      cout <<polyNorm[i] <<' ' <<T.rot <<' ' <<T.rot.getDeg() <<endl;
      glTransform(T);
      glDrawAxis(.02);
    }
  }
#endif
}

void PairCollision::kinDistance(arr &y, arr &J,
                                const arr &Jp1, const arr &Jp2) {
  y = ARR(distance-rad1-rad2);
  if(!!J) {
    arr Jdiff = Jp1 - Jp2;
    J = ~normal*Jdiff;
  }
}

void PairCollision::kinNormal(arr& y, arr& J,
                              const arr &Jp1, const arr &Jp2,
                              const arr &Jx1, const arr &Jx2) {
  y = normal;
  if(!!J) {
    J.resize(3, Jp1.d1).setZero();
    if(simplexType(1, 3)) {
      J = crossProduct(Jx2, y);
    }
    if(simplexType(3, 1)) {
      J = crossProduct(Jx1, y);
    }
    if(simplexType(2, 2)) {
      arr a = simplex1[1]-simplex1[0];  a/=length(a);
      arr b = simplex2[1]-simplex2[0];  b/=length(b);
      double ab=scalarProduct(a,b);
      if(1.-ab*ab>1e-8) { //the edges are not colinear
        double nn = ::sqrt(1.-ab*ab);
        double sign = ::sign(scalarProduct(normal, crossProduct(b,a)));
        J += ((sign/nn) * (eye(3,3) - normal*~normal)) * (skew(b) * crossProduct(Jx1, a) - skew(a) * crossProduct(Jx2, b));
      }
    }
    if(simplexType(2, 1)) {
      y = p1 - p2;
      J = Jp1 - Jp2;
      normalizeWithJac(y, J);
      arr a = simplex1[1]-simplex1[0];  a/=length(a);
      J -= a*(~a*J);
      J += a*(~a*crossProduct(Jx1, y));
    }
    if(simplexType(1, 2)) {
      y = p1 - p2;
      J = Jp1 - Jp2;
      normalizeWithJac(y, J);
      arr b = simplex2[1]-simplex2[0];  b/=length(b);
      J -= b*(~b*J);
      J += b*(~b*crossProduct(Jx2, y));
    }
    if(simplexType(1, 1)) {
      y = p1 - p2;
      J = Jp1 - Jp2;
      normalizeWithJac(y, J);
    }
    checkNan(J);
  }
}

void PairCollision::kinVector(arr& y, arr& J,
                              const arr &Jp1, const arr &Jp2,
                              const arr &Jx1, const arr &Jx2) {
  y = p1 - p2;
  if(!!J) {
    J = Jp1 - Jp2;
    if(simplexType(1, 3)) {
      J = normal*(~normal*J);
      J += crossProduct(Jx2, p1-p2);
    }
    if(simplexType(3, 1)) {
      J = normal*(~normal*J);
      J += crossProduct(Jx1, p1-p2);
    }
    if(simplexType(2, 2)) {
      J = normal*(~normal*J);
      arr a = simplex1[1]-simplex1[0];  a/=length(a);
      arr b = simplex2[1]-simplex2[0];  b/=length(b);
      double ab=scalarProduct(a,b);
      if(1.-ab*ab>1e-8) { //the edges are not colinear
        double nn = ::sqrt(1.-ab*ab);
        double sign = ::sign(scalarProduct(normal, crossProduct(b,a)));
        J += ((distance * sign/nn) * (eye(3,3) - normal*~normal)) * (skew(b) * crossProduct(Jx1, a) - skew(a) * crossProduct(Jx2, b));
      }
    }
    if(simplexType(2, 1)) {
      arr a = simplex1[1]-simplex1[0];  a/=length(a);
      J -= a*(~a*J);
      J += a*(~a*crossProduct(Jx1, p1-p2));
    }
    if(simplexType(1, 2)) {
      arr b = simplex2[1]-simplex2[0];  b/=length(b);
      J -= b*(~b*J);
      J += b*(~b*crossProduct(Jx2, p1-p2));
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
      J = J*fac + y*d_fac;
      checkNan(J);
    }
    y *= fac;
  }
}


void PairCollision::kinPointP1(arr& y, arr& J, const arr& Jp1, const arr& Jp2, const arr& Jx1, const arr& Jx2){
  y = p1;
  if(!!J) {
    J = Jp1;
    if(simplexType(3, 1)) {
      J = Jp2;
      J += normal*(~normal*(Jp1-Jp2));
      J += crossProduct(Jx1, p1-p2);
    }
    if(simplexType(2, 2)) {
      arr a = simplex1[1]-simplex1[0];  a/=length(a);
      arr b = simplex2[1]-simplex2[0];  b/=length(b);
      double ab=scalarProduct(a,b);

      J = Jp1;
      arr c = b*ab-a;
      double ac = scalarProduct(a,c);
      J += (1./ac) * a*(~c*(Jp2-Jp1));

      arr x = p1-p2;
      arr Jc = (b*~b-eye(3,3))* crossProduct(Jx1, a) + (ab*eye(3,3) + b*~a - 2.*a*~b)*crossProduct(Jx2, b);
      J += (1./ac) * scalarProduct(c,x) * (eye(3,3) - (1./ac)*a*~c) * crossProduct(Jx1, a);
      J -= (1./ac) * (a * ~x) * (eye(3,3) - (1./ac)*c*~a) * Jc;
    }
    if(simplexType(2, 1)) {
      arr a = simplex1[1]-simplex1[0];  a/=length(a);
      J += a*(~a*(Jp2-Jp1));
      J += a*(~a*crossProduct(Jx1, p1-p2));
    }
    checkNan(J);
  }
}

void PairCollision::kinPointP2(arr& y, arr& J, const arr& Jp1, const arr& Jp2, const arr& Jx1, const arr& Jx2){
  y = p2;
  if(!!J) {
    J = Jp2;
    if(simplexType(1, 3)) {
      J = Jp1;
      J += normal*(~normal*(Jp2-Jp1));
      J += crossProduct(Jx2, p2-p1);
    }
    if(simplexType(2, 2)) {
      arr a = simplex2[1]-simplex2[0];  a/=length(a);
      arr b = simplex1[1]-simplex1[0];  b/=length(b);
      double ab=scalarProduct(a,b);

      J = Jp2;
      arr c = b*ab-a;
      double ac = scalarProduct(a,c);
      J += (1./ac) * a*(~c*(Jp1-Jp2));

      arr x = p2-p1;
      arr Jc = (b*~b-eye(3,3))* crossProduct(Jx2, a) + (ab*eye(3,3) + b*~a - 2.*a*~b)*crossProduct(Jx1, b);
      J += (1./ac) * scalarProduct(c,x) * (eye(3,3) - (1./ac)*a*~c) * crossProduct(Jx2, a);
      J -= (1./ac) * (a * ~x) * (eye(3,3) - (1./ac)*c*~a) * Jc;
    }
    if(simplexType(1, 2)) {
      arr b = simplex2[1]-simplex2[0];  b/=length(b);
      J += b*(~b*(Jp1-Jp2));
      J += b*(~b*crossProduct(Jx2, p2-p1));
    }
    checkNan(J);
  }
}

void PairCollision::kinCenter(arr& y, arr& J, const arr& Jp1, const arr& Jp2, const arr& Jx1, const arr& Jx2){
  y = .5 * (p1 + p2 + (rad2-rad1)*normal);
  if(!!J){
    arr JP1, JP2, Jn, yy;
    kinPointP1(yy, JP1, Jp1, Jp2, Jx1, Jx2);
    kinPointP2(yy, JP2, Jp1, Jp2, Jx1, Jx2);
    kinNormal(yy, Jn, Jp1, Jp2, Jx1, Jx2);
    J = .5 * (JP1 + JP2 + (rad2-rad1)*Jn);
  }
}

void PairCollision::nearSupportAnalysis(double eps) {
  rai::Mesh M1(*mesh1); t1->applyOnPointArray(M1.V);
  rai::Mesh M2(*mesh2); t2->applyOnPointArray(M2.V);
  
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

double coll_1on2(arr &p2, arr& normal, const arr &pts1, const arr &pts2) {
  CHECK(pts1.nd==2 && pts1.d0==1 && pts1.d1==3, "I need a set of 1 pts1");
  CHECK(pts2.nd==2 && pts2.d0==2 && pts2.d1==3, "I need a set of 2 pts2");

  const arr& p1 = pts1[0];

  arr b = pts2[1]-pts2[0];  b/=length(b);

  p2 = pts2[0];
  p2 += scalarProduct(p1-p2, b)*b;

  normal = p1-p2;
  double d = length(normal);
  normal /= d;
  return d;
}

double coll_1on3(arr &p2, arr& normal, const arr &pts1, const arr &pts2) {
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
  return d;
}

double coll_2on2(arr &p1, arr& p2, arr& normal, const arr &pts1, const arr &pts2) {
  CHECK(pts1.nd==2 && pts1.d0==2 && pts1.d1==3, "I need a set of 2 pts1");
  CHECK(pts2.nd==2 && pts2.d0==2 && pts2.d1==3, "I need a set of 2 pts2");
  
  //compute normal
  arr a=pts1[1]-pts1[0], b=pts2[1]-pts2[0];
  normal = crossProduct(b, a);
  normal /= length(normal);
  
  //distance
  double d = scalarProduct(normal, pts2[0]-pts1[0]);
  
  //2nd plane
  arr n = crossProduct(normal, b);
  double t = scalarProduct(pts2[0]-pts1[0], n)/scalarProduct(a, n);//https://en.wikipedia.org/wiki/Line%E2%80%93plane_intersection
  
  p1 = pts1[0] + t*a;
  p2 = p1 + d*normal;
  
  return d;
}

double coll_2on3(arr &p1, arr& p2, arr& normal, const arr &pts1, const arr &pts2, const arr& center) {
  CHECK(pts1.nd==2 && pts1.d0==2 && pts1.d1==3, "I need a set of 2 pts1");
  //collide center on line to get p1:
  arr cen = center;
  cen.reshape(1,3);
  coll_1on2(p1, normal, cen, pts1);
//  p1 = .5*(pts1[0]+pts1[1]); //take center of line segment as single point
  p1.reshape(1,3);
  double d = coll_1on3(p2, normal, p1, pts2);
  p1.reshape(3);
  return d;
}

double coll_3on3(arr &p1, arr& p2, arr& normal, const arr &pts1, const arr &pts2, const arr& center) {
  CHECK(pts1.nd==2 && pts1.d0==3 && pts1.d1==3, "I need a set of 2 pts1");
  //collide center on tri1 to get p1:
  arr cen = center;
  cen.reshape(1,3);
  coll_1on3(p1, normal, cen, pts1);
//  p1 = (1./3.)*(pts1[0]+pts1[1]+pts1[2]); //take center of line tri as single point
  p1.reshape(1,3);
  double d = coll_1on3(p2, normal, p1, pts2);
  p1.reshape(3);
  return d;
}
