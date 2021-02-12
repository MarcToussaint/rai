/*  ------------------------------------------------------------------
    Copyright (c) 2011-2020 Marc Toussaint
    email: toussaint@tu-berlin.de

    This code is distributed under the MIT License.
    Please see <root-path>/LICENSE for details.
    --------------------------------------------------------------  */

#include "geoOptim.h"
#include "../Optim/constrained.h"
#include "../Geo/qhull.h"
#include "../Gui/opengl.h"
#include "../Algo/ann.h"
#include "../Geo/pairCollision.h"
#include "../Geo/analyticShapes.h"

void fitSSBox(arr& x, double& f, double& g, const arr& X, int verbose) {
  struct fitSSBoxProblem : MathematicalProgram {
    const arr& X;
    fitSSBoxProblem(const arr& X):X(X) {}
    virtual void getFeatureTypes(ObjectiveTypeA& tt) { tt.resize(5+X.d0); tt=OT_ineq; tt(0) = OT_f; }
    void evaluate(arr& phi, arr& J, const arr& x) {
      phi.resize(5+X.d0);
      if(!!J) {  J.resize(5+X.d0, 11); J.setZero(); }

      //-- the scalar objective
      double a=x(0), b=x(1), c=x(2), r=x(3); //these are box-wall-coordinates --- not WIDTH!
      phi(0) = a*b*c + 2.*r*(a*b + a*c +b*c) + 4./3.*r*r*r;
      if(!!J) {
        J(0, 0) = b*c + 2.*r*(b+c);
        J(0, 1) = a*c + 2.*r*(a+c);
        J(0, 2) = a*b + 2.*r*(a+b);
        J(0, 3) = 2.*(a*b + a*c +b*c) + 4.*r*r;
      }

      //-- positive
      double w=100.;
      phi(1) = -w*(a-.001);
      phi(2) = -w*(b-.001);
      phi(3) = -w*(c-.001);
      phi(4) = -w*(r-.001);
      if(!!J) {
        J(1, 0) = -w;
        J(2, 1) = -w;
        J(3, 2) = -w;
        J(4, 3) = -w;
      }

      //-- all constraints
      for(uint i=0; i<X.d0; i++) {
        arr y, Jy;
        y = X[i];
        y.append(x);
        phi(i+5) = DistanceFunction_SSBox(Jy, NoArr, y);
        //      Jy({3,5})() *= -1.;
        if(!!J) J[i+5] = Jy({3, -1});
      }
    }
    virtual void getFHessian(arr& H, const arr& x) {
      double a=x(0), b=x(1), c=x(2), r=x(3); //these are box-wall-coordinates --- not WIDTH!
      H.resize(4, 4);
      H(0, 1) = H(1, 0) = c + 2.*r;
      H(0, 2) = H(2, 0) = b + 2.*r;
      H(0, 3) = H(3, 0) = 2.*(b+c);

      H(1, 2) = H(2, 1) = a + 2.*r;
      H(1, 3) = H(3, 1) = 2.*(a+c);

      H(2, 3) = H(3, 2) = 2.*(a+b);

      H(3, 3) = 8.*r;
    }

  } F(X);

  //initialization
  x.resize(11);
  rai::Quaternion rot;
  rot.setRandom();
  arr tX = X * rot.getArr(); //rotate points (with rot^{-1})
  arr ma = max(tX, 0), mi = min(tX, 0); //get coordinate-wise min and max
  x({0, 2})() = (ma-mi)/2.;  //sizes
  x(3) = 1.; //sum(ma-mi)/6.;  //radius
  x({4, 6})() = rot.getArr() * (mi+.5*(ma-mi)); //center (rotated back)
  x({7, 10})() = conv_quat2arr(rot);
  rndGauss(x({7, 10})(), .1, true);
  x({7, 10})() /= length(x({7, 10})());

  if(verbose>1) {
    checkJacobianCP(F, x, 1e-4);
    checkHessianCP(F, x, 1e-4);
  }

  OptConstrained opt(x, NoArr, F, OPT(
                       stopTolerance = 1e-4,
                       stopFTolerance = 1e-3,
                       damping=1,
                       maxStep=-1,
                       constrainedMethod = augmentedLag,
                       aulaMuInc = 1.1
                     ));
  opt.run();

  if(verbose>1) {
    checkJacobianCP(F, x, 1e-4);
    checkHessianCP(F, x, 1e-4);
  }

  f = opt.L.get_costs();
  g = opt.L.get_sumOfGviolations();
}

void computeOptimalSSBox(rai::Mesh& mesh, arr& x_ret, rai::Transformation& t_ret, const arr& X, uint trials, int verbose) {
  if(!X.N) { mesh.clear(); return; }

  arr x, x_best;
  double f, g, f_best, g_best;
  fitSSBox(x_best, f_best, g_best, X, verbose);
  for(uint k=1; k<trials; k++) {
    fitSSBox(x, f, g, X, verbose);
    if(g<g_best-1e-4 ||
        (g<1e-4 && f<f_best)) { x_best=x; f_best=f; g_best=g; }
  }

  x = x_best;

  //convert box wall coordinates to box width (incl radius)
  x(0) = 2.*(x(0)+x(3));
  x(1) = 2.*(x(1)+x(3));
  x(2) = 2.*(x(2)+x(3));

  if(x_ret!=NoArr)
    x_ret=x;

  if(verbose>2) {
    cout <<"x=" <<x;
    cout <<"\nf = " <<f_best <<"\ng-violations = " <<g_best <<endl;
  }

  rai::Transformation t;
  t.setZero();
  t.pos.set(x({4, 6}));
  t.rot.set(x({7, -1}));
  t.rot.normalize();
  mesh.setSSBox(x(0), x(1), x(2), x(3));
  t.applyOnPointArray(mesh.V);

  if(t_ret!=NoTransformation)
    t_ret = t;
}

void minimalConvexCore(arr& core, const arr& points, double radius, int verbose) {
  struct convexCoreProblem : MathematicalProgram {
    const arr& X;
    const uintA& T;
    double radius;

    OpenGL gl;
    rai::Mesh m0, m1;
    convexCoreProblem(const arr& X, const uintA& T, double radius) : X(X), T(T), radius(radius) {
      m0.V = X;
      m0.makeConvexHull();
      m0.C = {.5, .3, .3, 1.};
      m1.C = {.3, .3, .8, .2};
      gl.add(glStandardLight);
      gl.add(m0);
      gl.add(m1);
    }
    virtual void getFeatureTypes(ObjectiveTypeA& tt) { tt = consts<ObjectiveType>(OT_ineq, X.d0+1); tt(0) = OT_f; }
    void evaluate(arr& phi, arr& J, const arr& x) {
      uint n = X.d0;
      arr _x = x.ref().reshape(-1, 3);
      //n inequalities on distances
      //single accumulated cost

      phi.resize(n+1).setZero();
      if(!!J) J.resize(n+1, x.N).setZero();

      //-- accumulated cost
      double cost = 0.;
      arr Jcost = zeros(x.N);
      for(uint i=0; i<T.d0; i++) {
        int a=T(i, 0), b=T(i, 1), c=T(i, 2);
        {
          arr d = x[a]-x[b];
          double l = length(d);
          cost += l;
          if(l>1e-6) {
            Jcost({3*a, 3*a+2}) += d/l;
            Jcost({3*b, 3*b+2}) += -d/l;
            //            if(!!H){
            //              for(uint k=0;k<3;k++) for(uint l=0;l<3;l++){
            //                H(3*a+k,3*a+l) += d(k)*d(l)/(l*l);
            //              }
            //            }
          }
        }
        {
          arr d = x[c]-x[b];
          double l = length(d);
          cost += l;
          if(l>1e-6) {
            Jcost({3*c, 3*c+2}) += d/l;
            Jcost({3*b, 3*b+2}) += -d/l;
          }
        }
        {
          arr d = x[a]-x[c];
          double l = length(d);
          cost += l;
          if(l>1e-6) {
            Jcost({3*a, 3*a+2}) += d/l;
            Jcost({3*c, 3*c+2}) += -d/l;
          }
        }
      }

      double alpha = 1e-2;
      phi(0) = alpha*cost;
      if(!!J) J[0] = alpha*Jcost;

      //-- radius inequalities
      for(uint i=0; i<n; i++) {
        arr d = X[i] - _x[i];
        double l = length(d);
        phi(i+1) = l - radius;
        if(l>1e-6) {
          if(!!J) J(i+1, {3*i, 3*i+2}) += -d/l;
        }
      }

      gl.dataLock.lock(RAI_HERE);
      m1.setSSCvx(_x, radius);
      gl.dataLock.unlock();
      gl.update();

    }
  };

  uintA T;
  arr pts_hull = getHull(points, T);

  if(!core) {
    core = pts_hull;
  } else {
    core = getHull(core, T);
  }

  convexCoreProblem P(pts_hull, T, radius);

  arr x = core;

  if(verbose>1) {
    checkJacobianCP(P, x, 1e-4);
    checkHessianCP(P, x, 1e-4);
  }

  OptConstrained opt(x, NoArr, P, OPT(
                       stopTolerance = 1e-4,
                       stopFTolerance = 1e-3,
                       damping=1.,
                       maxStep=.1,
                       constrainedMethod = augmentedLag,
                       aulaMuInc = 1.1,
                       verbose = 3
                     ));
  opt.run();

  if(verbose>0) {
    LOG(0) <<" f: " <<opt.L.get_costs() <<" g: " <<opt.L.get_sumOfGviolations();

    P.gl.watch();
  }

  if(verbose>1) {
    checkJacobianCP(P, x, 1e-4);
    checkHessianCP(P, x, 1e-4);
  }
}

struct MeshEdge {
  uint i, j;
  double d;
  void write(ostream& os) const { os <<i <<'-' <<j <<": " <<d; }
};
stdOutPipe(MeshEdge)

bool shorter(const MeshEdge& a, const MeshEdge& b) {
  return a.d<b.d;
}

struct MeshCluster {
  arr center;
  double radius;
  uintA points;
};

void RitterAlgorithm(arr& center, double& radius, const arr& pts) {
  arr Pts = ~pts;
  int minx=Pts[0].argmin(), maxx=Pts[0].argmax(),
      miny=Pts[1].argmin(), maxy=Pts[1].argmax(),
      minz=Pts[2].argmin(), maxz=Pts[2].argmax();
  double dist2x = sumOfSqr(pts[minx] - pts[maxx]);
  double dist2y = sumOfSqr(pts[miny] - pts[maxy]);
  double dist2z = sumOfSqr(pts[minz] - pts[maxz]);

  int min = minx;
  int max = maxx;
  if(dist2y > dist2x && dist2y > dist2z) {
    min = miny;
    max = maxy;
  } else if(dist2z > dist2x && dist2z > dist2y) {
    min = minz;
    max = maxz;
  }

  center = .5 * (pts[min] + pts[max]);
  radius = 0.;

  for(uint i=0; i<pts.d0; i++) {
    double r = length(pts[i]-center);
    if(r>radius) radius = r;
  }
}

void minimalConvexCore2(arr& core, const arr& org_pts, double max_radius, int verbose) {
  arr pts = getHull(org_pts);

  rai::Array<MeshCluster> clusters;

  //-- initialize
  clusters.resize(pts.d0);
  for(uint i=0; i<clusters.N; i++) {
    clusters(i).center = pts[i];
    clusters(i).radius = 0.;
    clusters(i).points = {i};
  }

  bool changes=false;
  //-- merge
  for(uint i=0;; i++) {
    if(i>=clusters.N) {
      if(!changes) break;
      i = 0;
      changes=false;
    }
    if(clusters.N<=1) break;
    MeshCluster& c = clusters(i);

    double min_d=0.;
    int min_j=-1;
    for(uint j=0; j<clusters.N; j++) {
      if(j!=i) {
        double d=length(clusters(j).center-c.center);
        if(min_j<0 || d<min_d) { min_d=d; min_j=j; }
      }
    }

    //union
    uintA merge_points = c.points;
    merge_points.setAppend(clusters(min_j).points);
    arr merge_pts(merge_points.N, 3);
    for(uint j=0; j<merge_points.N; j++) merge_pts[j] = pts[merge_points(j)];

    arr merge_center;
    double merge_radius;
    RitterAlgorithm(merge_center, merge_radius, merge_pts);
    if(merge_radius<max_radius) { //do it! merge!
      LOG(1) <<"merging clusters! #clusters: " <<clusters.N;
      c.center = merge_center;
      c.radius = merge_radius;
      c.points = merge_points;

      clusters.remove(min_j);
      changes = true;
    }
  }

  core.resize(clusters.N, 3);
  for(uint i=0; i<core.d0; i++) core[i] = clusters(i).center;
}

void minimalConvexCore3(arr& core, const arr& org_pts, double max_radius, int verbose) {
  arr pts = getHull(org_pts);

  uint k=20;
  arr centers(k, 3);
  uintA labels(pts.d0);
  HALT("obsolete");
  //kmeans(centers.p, pts.p, labels.p, 3, pts.d0, k, 100, 3);

  core = centers;
}

struct LinearProgram : MathematicalProgram {
  arr c;
  arr G, g;

  LinearProgram(const arr& _c, const arr& _G, const arr& _g) : c(_c), G(_G), g(_g) {
    CHECK_EQ(c.N, G.d1, "");
    CHECK_EQ(g.N, G.d0, "");
  }

  uint dim_x() { return c.N; }

  virtual void getFeatureTypes(ObjectiveTypeA& ot) { ot.resize(1+G.d0); ot = OT_ineq; ot(0) = OT_f; }
  virtual void evaluate(arr& phi, arr& J, const arr& x) {
    phi.resize(1+G.d0);
    if(!!J) J.resize(phi.N, x.N).setZero();

    phi(0) = scalarProduct(c, x);
    if(!!J) J[0] = c;

    phi.setVectorBlock(G*x+g, 1);
    if(!!J) J.setMatrixBlock(G, 1, 0);
  }
};

double sphereReduceConvex(rai::Mesh& M, double radius, int verbose) {
  //-- construct H-polytope (normals and offsets)
  M.makeConvexHull();
  arr V_orig = M.V;
  M.computeNormals();
  uint nIneq = M.Tn.d0;
  arr G(nIneq, 3), g(nIneq);
  for(uint i=0; i<nIneq; i++) {
    arr n = M.Tn[i];
    arr p = M.V[M.T(i, 0)];
    G[i] = n;
    g(i) = -scalarProduct(n, p)+radius;
  }

  //-- Define LP
  for(uint i=0; i<M.V.d0; i++) {
    arr x = M.V[i];
    arr c = -M.Vn[i];
    LinearProgram LP(c, G, g);
    OptConstrained opt(x, NoArr, LP, OPT(stopTolerance=1e-4, stopGTolerance=1e-4));
    opt.run();
  }

  double r = radius;
  for(uint i=0; i<M.V.d0; i++) {
    double l = length(M.V[i] - V_orig[i]);
    if(l>r) r=l;
  }
  M.fuseNearVertices(1e-3);
  M.makeConvexHull();

  cout <<"result radius:" <<r <<endl;

  return r;
}

struct FitSphereProblem : MathematicalProgram {
  const arr& X;
  FitSphereProblem(const arr& X):X(X) {}
  virtual void getFeatureTypes(ObjectiveTypeA& tt) { tt.resize(1+X.d0); tt=OT_ineq;   tt(0) = OT_f; }
  void evaluate(arr& phi, arr& J, const arr& x) {
    CHECK_EQ(x.N, 4, "");  //x,y,z,radius
    phi.resize(1+X.d0);
    if(!!J) {  J.resize(1+X.d0, 4); J.setZero(); }

    //-- the radius objective
    phi(0) = x(3);
    if(!!J)  J(0, 3) = 1.;

    //-- all constraints
    arr c = x({0, 2});
    double r = x(3);
    for(uint i=0; i<X.d0; i++) {
      arr d = c - X[i];
      double dlen = length(d);
      phi(1+i) = dlen - r;
      if(!!J) {
        J(1+i, {0, 2}) = d / dlen;
        J(1+i, 3) = -1.;
      }
    }
  }
};

struct FitCapsuleProblem : MathematicalProgram {
  const arr& X;
  FitCapsuleProblem(const arr& X):X(X) {}
  virtual void getFeatureTypes(ObjectiveTypeA& tt) { tt.resize(2+X.d0); tt=OT_ineq; tt(0) = OT_f; }
  void evaluate(arr& phi, arr& J, const arr& x) {
    CHECK_EQ(x.N, 7, "");  //x,y,z, x,y,z, radius
    phi.resize(2+X.d0);
    if(!!J) {  J.resize(2+X.d0, 7); J.setZero(); }

    //-- the radius objective
    phi(0) = 4.*x(6);
    if(!!J)  J(0, 6) = 4.;

    //-- the capsule length objective
    arr a = x({0, 2});
    arr b = x({3, 5});
    double l = length(a-b);
    phi(0) += l;
    if(!!J) {
      J(0, {0, 2}) += (a-b)/l;
      J(0, {3, 5}) += (b-a)/l;
    }

    //-- all constraints
    double scale = 1e1;
    arr pts2 = x({0, 5});
    pts2.reshape(2, 3);
    double r = x(6);
    for(uint i=0; i<X.d0; i++) {
      double d, s;
      arr p2, normal;
      d = coll_1on2(p2, normal, s, X[i].reshape(1, 3), pts2);
      if(d>1e-8) {
        normal *= -scale;
        checkNan(normal);
        phi(2+i) = scale*(d - r);
        if(!!J) {
          if(s<=0.) {
            J(2+i, {0, 2}) = normal;
          } else if(s>=1.) {
            J(2+i, {3, 5}) = normal;
          } else {
            J(2+i, {0, 2}) = (1.-s)*normal;
            J(2+i, {3, 5}) = s*normal;
          }
          J(2+i, 6) = -scale;
        }
      }
    }
    checkNan(J);
  }
  virtual void getFHessian(arr& H, const arr& x) {
    arr a = x({0, 2});
    arr b = x({3, 5});
    double l = length(a-b);

    arr B(3, 3);
    B.setId();
    B *= 1./l;
    B -= ((a-b)^(a-b)) / (l*l*l);
    arr A;
    A.setBlockMatrix(B, -B, -B, B);
    H.setMatrixBlock(A, 0, 0);
    checkNan(H);
  }//zero

};

void optimalSphere(arr& core, uint num, const arr& org_pts, double& radius, int verbose) {
  arr pts = getHull(org_pts);

  LOG(1) <<"merging with radius " <<radius;

  //initialization
  arr x;
  if(num==1) {
    RitterAlgorithm(x, radius, pts);
  } else if(num==2) {
    x.resize(2, 3);
    x[0] = pts[rnd(pts.d0)];
    x[1] = pts[rnd(pts.d0)];
    radius = .1;
  }
  x.append(radius);

  //problem
  ptr<MathematicalProgram> F;
  if(num==1) F = make_shared<FitSphereProblem>(pts);
  else if(num==2)  F = make_shared<FitCapsuleProblem>(pts);

  if(verbose>1) {
    checkJacobianCP(*F, x, 1e-4);
    checkHessianCP(*F, x, 1e-4);
  }

#if 1
  OptConstrained opt(x, NoArr, *F, OPT(
                       stopTolerance = 1e-4,
                       stopFTolerance = 1e-3,
                       damping=1,
                       maxStep=-1,
                       constrainedMethod = augmentedLag,
                       aulaMuInc = 1.1
                     ));
#else
  OptPrimalDual opt(x, NoArr, *F, OPT(
                      stopTolerance = 1e-5,
                      stopFTolerance = 1e-5,
                      damping=1e-0,
                      maxStep=-1,
                      muLBInit=1e1));
#endif

  opt.run();

  if(verbose>1) {
    checkJacobianCP(*F, x, 1e-4);
    checkHessianCP(*F, x, 1e-4);
  }

  core = x({0, x.N-2});
  core.reshape(-1, 3);
  radius = x.last();

  double f = opt.L.get_costs();
  double g = opt.L.get_sumOfGviolations();
  cout <<"core:" <<core <<" radius:"<<radius <<endl;
  cout <<"cost:" <<f <<" ineq:"<<g <<endl;
}
