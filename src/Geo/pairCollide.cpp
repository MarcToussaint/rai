#include "pairCollide.h"

#ifdef MLR_extern_GJK
extern "C"{
#  include "GJK/gjk.h"
}
#endif

#include <Gui/opengl.h>
#include <ccd/ccd.h>
#include <ccd/quat.h>

PairCollision::PairCollision(mlr::Mesh &mesh1, mlr::Mesh &mesh2, mlr::Transformation &t1, mlr::Transformation &t2)
    : mesh1(mesh1), mesh2(mesh2), t1(t1), t2(t2) {

    double d2 = GJK_sqrDistance(); //mesh1, mesh2, t1, t2, p1, p2, e1, e2, pt1.x, pt2.x);

    if(d2>1e-10){
      distance = sqrt(d2);
    }else{
      //THIS IS COSTLY!! DO WITHIN THE SUPPORT FUNCTION!
      mlr::Mesh M1(mesh1); t1.applyOnPointArray(M1.V);
      mlr::Mesh M2(mesh2); t2.applyOnPointArray(M2.V);
      distance = - GJK_libccd_penetration(M1, M2);
    }

//    double rad=rad1+rad2;
//    if(rad>0.){
//        arr d = p1-p2;
//        double l = length(d);
//        CHECK_LE(fabs(fabs(distance)-l), 1e-10, "");
//        p1 -= (rad1/distance)*d;
//        for(uint i=0;i<simplex1.d0;i++) simplex1[i] -= (rad1/distance)*d;
//        p2 += (rad2/distance)*d;
//        for(uint i=0;i<simplex2.d0;i++) simplex2[i] += (rad2/distance)*d;

//        distance -= rad;

////        double fac = (l-rad)/l;
////        if(&J){
////        arr d_fac = (1.-(l-rad)/l)/l2 *(~v)*J;
////        J = J*fac + v*d_fac;
////      }
////      v *= fac;
//    }
}

void PairCollision::write(std::ostream &os) const{
    os <<"PairCollide INFO" <<endl;
    if(distance>0.){
        os <<"  distance=" <<distance <<endl;
    }else{
        os <<"  penetration=" <<distance <<endl;
    }
    os <<"  closest points: " <<p1 <<"  " <<p2 <<endl;
    os <<"  simplex #: " <<simplex1.d0 <<"  " <<simplex2.d0 <<endl;
    if(eig1.N || eig2.N) os <<"  EIG #: " <<eig1.d0<<'-' <<eig2.d0 <<endl;
}


void support_mesh(const void *_obj, const ccd_vec3_t *_dir, ccd_vec3_t *v){
    mlr::Mesh *m = (mlr::Mesh*)_obj;
    arr dir(_dir->v, 3, true);
    uint vertex = m->support(dir);
    memmove(v->v, &m->V(vertex, 0), 3*m->V.sizeT);
}

void center_mesh(const void *obj, ccd_vec3_t *center){
    mlr::Mesh *m = (mlr::Mesh*)obj;
    mlr::Vector c = m->getCenter();
    memmove(center->v, &c.x, 3*sizeof(double));
}

double PairCollision::GJK_libccd_penetration(const mlr::Mesh& m1,const mlr::Mesh& m2){
  ccd_t ccd;
  CCD_INIT(&ccd); // initialize ccd_t struct

  // set up ccd_t struct
  ccd.support1       = support_mesh; // support function for first object
  ccd.support2       = support_mesh; // support function for second object
  ccd.max_iterations = 100;   // maximal number of iterations
  ccd.epa_tolerance  = 1e-4;  // maximal tolerance fro EPA part
  ccd.center1       = center_mesh; // support function for first object
  ccd.center2       = center_mesh; // support function for second object

  ccd_real_t depth;
  ccd_vec3_t _dir, _pos;
  ccd_vec3_t simplex[8];

  //    S.m1.translate(randn(3));
  //    int intersect = ccdGJKIntersect(&S.m1, &S.m2, &ccd, &v1, &v2);
//  int non_intersect = ccdGJKPenetration(&m1, &m2, &ccd, &depth, &_dir, &_pos);
  //    int intersect = ccdMPRIntersect(&S.m1, &S.m2, &ccd);
  int ret = ccdMPRPenetration(&m1, &m2, &ccd, &depth, &_dir, &_pos, simplex);

  if(ret<0) return 0.; //no intersection
  //res == 1  // Touching contact on portal's v1.
  //res == 2  // Origin lies on v0-v1 segment.
  //res == 0  // penetration

  simplex1.resize(0,3);
  simplex2.resize(0,3);
  arr c1=m1.getMean();
  arr c2=m2.getMean();
  arr s(3);

  //grab simplex points
  bool append;
  for(uint i=0;i<4;i++){
      s = arr(simplex[0+i].v, 3);
      append=true;
      if(sqrDistance(s, c1)<1e-10) append=false;
      for(uint i=0;i<simplex1.d0;i++) if(sqrDistance(s, simplex1[i])<1e-10){ append=false; break; }
      if(append) simplex1.append(s);

      s = arr(simplex[4+i].v, 3);
      append=true;
      if(sqrDistance(s, c2)<1e-10) append=false;
      for(uint i=0;i<simplex2.d0;i++) if(sqrDistance(s, simplex2[i])<1e-10){ append=false; break; }
      if(append) simplex2.append(s);
  }

  //compute witness points
  double d=0.;
  if(simplexType(1, 3)){
      p1 = simplex1[0];
      d=coll_1on3(p2, normal, simplex1, simplex2);
  }
  if(simplexType(3, 1)){
      p2 = simplex2[0];
      d=coll_1on3(p1, normal, simplex2, simplex1);
  }
  if(simplexType(2, 2)){
      d=coll_2on2(p1, p2, normal, simplex1, simplex2);
  }
  if(simplexType(2, 3)) LOG(-1) <<"SIMPLEX TYPES 2 & 3 - NIY";
  if(simplexType(3, 2)) LOG(-1) <<"SIMPLEX TYPES 3 & 2 - NIY";

  return fabs(d);
}

double PairCollision::GJK_sqrDistance(){
  // convert meshes to 'Object_structures'
  Object_structure m1,m2;
  mlr::Array<double*> Vhelp1, Vhelp2;
  m1.numpoints = mesh1.V.d0;  m1.vertices = mesh1.V.getCarray(Vhelp1);  m1.rings=NULL; //TODO: rings would make it faster
  m2.numpoints = mesh2.V.d0;  m2.vertices = mesh2.V.getCarray(Vhelp2);  m2.rings=NULL;

  // convert transformations to affine matrices
  arr T1,T2;
  mlr::Array<double*> Thelp1, Thelp2;
  if(&t1){  T1=t1.getAffineMatrix();  T1.getCarray(Thelp1);  }
  if(&t2){  T2=t2.getAffineMatrix();  T2.getCarray(Thelp2);  }

  // call GJK
  simplex_point simplex;
  p1.resize(3).setZero();
  p2.resize(3).setZero();
  double d2 = gjk_distance(&m1, Thelp1.p, &m2, Thelp2.p, p1.p, p2.p, &simplex, 0);

  normal = p1-p2;
  normal /= length(normal);

  //grab simplex points
  simplex1.resize(0,3);
  simplex2.resize(0,3);
  if(simplex.npts>=1){
      simplex1.append(arr(simplex.coords1[0], 3));
      simplex2.append(arr(simplex.coords2[0], 3));
  }
  if(simplex.npts>=2){
      if(simplex.simplex1[1]!=simplex.simplex1[0]) simplex1.append(arr(simplex.coords1[1], 3));
      if(simplex.simplex2[1]!=simplex.simplex2[0]) simplex2.append(arr(simplex.coords2[1], 3));
  }
  if(simplex.npts>=3){
      if(simplex.simplex1[2]!=simplex.simplex1[0] && simplex.simplex1[2]!=simplex.simplex1[1]) simplex1.append(arr(simplex.coords1[2], 3));
      if(simplex.simplex2[2]!=simplex.simplex2[0] && simplex.simplex2[2]!=simplex.simplex2[1]) simplex2.append(arr(simplex.coords2[2], 3));
  }

  return d2;
}


void PairCollision::glDraw(OpenGL &){
    glColor(0., 1., 0., 1.);
    glDrawDiamond(p1(0), p1(1), p1(2), .01, .01, .01);
    for(uint i=0;i<simplex1.d0;i++){
        arr v = simplex1[i];
        glDrawDiamond(v(0), v(1), v(2), .02, .02, .02);
    }
    glColor(0., 0., 1., 1.);
    glDrawDiamond(p2(0), p2(1), p2(2), .01, .01, .01);
    for(uint i=0;i<simplex2.d0;i++){
        arr v = simplex2[i];
        glDrawDiamond(v(0), v(1), v(2), .02, .02, .02);
    }
    glColor(1., 0., 0., 1.);
    glLineWidth(5.f);
    glDrawProxy(p1, p2, .05);
    glLineWidth(2.f);
    glLoadIdentity();
}

void PairCollision::kinVector(arr& y, arr& J,
                             const arr &Jp1, const arr &Jp2,
                             const arr &Jx1, const arr &Jx2){
    y = p1 - p2;
    if(!&J) return;
    J = Jp1 - Jp2;
    if(simplexType(1, 3) || simplexType(3, 1)){
      J = normal*(~normal*J);
      arr Jv;
      if(simplex1.d0==1) Jv = crossProduct(Jx2, p1-p2); //K.kinematicsVec(vec, Jv, &s2.frame, s2.frame.X.rot/(p1-p2));
      if(simplex2.d0==1) Jv = crossProduct(Jx1, p1-p2); //K.kinematicsVec(vec, Jv, &s1.frame, s1.frame.X.rot/(p1-p2));
      J += Jv;
    }
    if(simplexType(2, 2)){
      arr Jv, a, b;
      J = normal*(~normal*J);

      //get edges
      a = simplex1[1]-simplex1[0]; //edge
      a /= length(a);
      b = simplex2[1]-simplex2[0]; //edge
      b /= length(b);
      double ab=scalarProduct(a,b);

      Jv = crossProduct(Jx1, a);      //K.kinematicsVec(vec, Jv, &s1.frame, s1.frame.X.rot/e1);
      J += (a-b*ab) * (1./(1.-ab*ab)) * (~(p1-p2)*(b*~b -eye(3,3))) * Jv;

      Jv = crossProduct(Jx2, b);      //K.kinematicsVec(vec, Jv, &s2.frame, s2.frame.X.rot/e2);
      J += (b-a*ab) * (1./(1.-ab*ab)) * (~(p1-p2)*(a*~a -eye(3,3))) * Jv;
    }
    if(simplexType(1, 2) || simplexType(2, 1)){
      arr vec, Jv, n;
      if(simplex1.d0==2) { n = simplex1[1]-simplex1[0];  n /= length(n); }
      if(simplex2.d0==2) { n = simplex2[1]-simplex2[0];  n /= length(n); }
      J = J - n*(~n*J);
      if(simplex1.d0==2) Jv = crossProduct(Jx1, p1-p2);  //K.kinematicsVec(vec, Jv, &s1.frame, s1.frame.X.rot/(p1-p2));
      if(simplex2.d0==2) Jv = crossProduct(Jx2, p1-p2);  //K.kinematicsVec(vec, Jv, &s2.frame, s2.frame.X.rot/(p1-p2));
      J += n*(~n*Jv);
    }
}

void PairCollision::kinDistance(arr &y, arr &J,
                                const arr &Jp1, const arr &Jp2,
                                const arr &Jx1, const arr &Jx2,
                                double rad1, double rad2){
    NIY;
//    kinVector(y, J, Jp1, Jp2, Jx1, Jx2);

            //    double rad=rad1+rad2;
            //    if(rad>0.){
            //        arr d = p1-p2;
            //        double l = length(d);
            //        CHECK_LE(fabs(fabs(distance)-l), 1e-10, "");
            //        p1 -= (rad1/distance)*d;
            //        for(uint i=0;i<simplex1.d0;i++) simplex1[i] -= (rad1/distance)*d;
            //        p2 += (rad2/distance)*d;
            //        for(uint i=0;i<simplex2.d0;i++) simplex2[i] += (rad2/distance)*d;

            //        distance -= rad;

            ////        double fac = (l-rad)/l;
            ////        if(&J){
            ////        arr d_fac = (1.-(l-rad)/l)/l2 *(~v)*J;
            ////        J = J*fac + v*d_fac;
            ////      }
            ////      v *= fac;
            //    }

}

void PairCollision::marginAnalysis(double margin){
    mlr::Mesh M1(mesh1); t1.applyOnPointArray(M1.V);
    mlr::Mesh M2(mesh2); t2.applyOnPointArray(M2.V);

    uintA pts1, pts2;
    M1.supportMargin(pts1, -normal, margin);
    M2.supportMargin(pts2, normal, margin);
//    cout <<"margin analysis: #1=" <<pts1.N <<"  #2=" <<pts2.N <<endl;

    simplex1.resize(0,3);
    for(uint i:pts1) simplex1.append(M1.V[i]);

    simplex2.resize(0,3);
    for(uint i:pts2) simplex2.append(M2.V[i]);

    //eigen value analysis
    m1 = mean(simplex1);
    m2 = mean(simplex2);
    arr var1 = covar(simplex1);
    arr var2 = covar(simplex2);

    arr sig1, sig2, vec1, vec2;
    lapack_EigenDecomp(var1, sig1, vec1);
    lapack_EigenDecomp(var2, sig2, vec2);

    eig1.resize(0,3);
    eig2.resize(0,3);
    for(uint i=0;i<3;i++){
        if(sig1(i)>1e-8) eig1.append(sqrt(sig1(i)) * vec1[i]);
        if(sig2(i)>1e-8) eig2.append(sqrt(sig2(i)) * vec2[i]);
    }

    cout <<"EIG1: " <<eig1 <<endl;
    cout <<"EIG2: " <<eig2 <<endl;
}

double coll_1on3(arr &pInTri, arr& normal, const arr &pts1, const arr &pts2){
  CHECK(pts1.nd==2 && pts1.d0==1 && pts1.d1==3, "I need a set of 1 pts1");
  CHECK(pts2.nd==2 && pts2.d0==3 && pts2.d1==3, "I need a set of 3 pts2");

  //choose pts1-vertex as origin
  arr tri = pts2;
  for(uint i=0;i<tri.d0;i++) tri[i] -= pts1[0];

  //compute normal of tri (plane eq first three parameters)
  arr a=tri[1]-tri[0], b=tri[2]-tri[0];
  normal = crossProduct(a, b);
  normal /= length(normal);

  //find plane eq offset parameter
  double d = scalarProduct(normal, tri[0]);

  pInTri = d*normal + pts1[0];
  return d;
}

double coll_2on2(arr &p1, arr& p2, arr& normal, const arr &pts1, const arr &pts2){
  CHECK(pts1.nd==2 && pts1.d0==2 && pts1.d1==3, "I need a set of 2 pts1");
  CHECK(pts2.nd==2 && pts2.d0==2 && pts2.d1==3, "I need a set of 2 pts2");

  //compute normal
  arr a=pts1[1]-pts1[0], b=pts2[1]-pts2[0];
  normal = crossProduct(a, b);
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
