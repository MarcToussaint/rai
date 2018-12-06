/*  ------------------------------------------------------------------
    Copyright (c) 2017 Marc Toussaint
    email: marc.toussaint@informatik.uni-stuttgart.de

    This code is distributed under the MIT License.
    Please see <root-path>/LICENSE for details.
    --------------------------------------------------------------  */

#include <Optim/constrained.h>

#include "geoOptim.h"

void fitSSBox(arr& x, double& f, double& g, const arr& X, int verbose) {
  struct fitSSBoxProblem : ConstrainedProblem {
    const arr& X;
    fitSSBoxProblem(const arr& X):X(X) {}
    void phi(arr& phi, arr& J, arr& H, ObjectiveTypeA& tt, const arr& x, arr& lambda) {
      phi.resize(5+X.d0);
      if(!!tt) { tt.resize(5+X.d0); tt=OT_ineq; }
      if(!!J) {  J.resize(5+X.d0,11); J.setZero(); }
      if(!!H) {  H.resize(11,11); H.setZero(); }
      
      //-- the scalar objective
      double a=x(0), b=x(1), c=x(2), r=x(3); //these are box-wall-coordinates --- not WIDTH!
      phi(0) = a*b*c + 2.*r*(a*b + a*c +b*c) + 4./3.*r*r*r;
      if(!!tt) tt(0) = OT_f;
      if(!!J) {
        J(0,0) = b*c + 2.*r*(b+c);
        J(0,1) = a*c + 2.*r*(a+c);
        J(0,2) = a*b + 2.*r*(a+b);
        J(0,3) = 2.*(a*b + a*c +b*c) + 4.*r*r;
      }
      if(!!H) {
        H(0,1) = H(1,0) = c + 2.*r;
        H(0,2) = H(2,0) = b + 2.*r;
        H(0,3) = H(3,0) = 2.*(b+c);
        
        H(1,2) = H(2,1) = a + 2.*r;
        H(1,3) = H(3,1) = 2.*(a+c);
        
        H(2,3) = H(3,2) = 2.*(a+b);
        
        H(3,3) = 8.*r;
      }
      
      //-- positive
      double w=100.;
      phi(1) = -w*(a-.001);
      phi(2) = -w*(b-.001);
      phi(3) = -w*(c-.001);
      phi(4) = -w*(r-.001);
      if(!!J) {
        J(1,0) = -w;
        J(2,1) = -w;
        J(3,2) = -w;
        J(4,3) = -w;
      }
      
      //-- all constraints
      for(uint i=0; i<X.d0; i++) {
        arr y, Jy;
        y = X[i];
        y.append(x);
        phi(i+5) = DistanceFunction_SSBox(Jy, NoArr, y);
        //      Jy({3,5})() *= -1.;
        if(!!J) J[i+5] = Jy({3,-1});
      }
    }
  } F(X);
  
  //initialization
  x.resize(11);
  rai::Quaternion rot;
  rot.setRandom();
  arr tX = X * rot.getArr(); //rotate points (with rot^{-1})
  arr ma = max(tX,0), mi = min(tX,0);  //get coordinate-wise min and max
  x({0,2})() = (ma-mi)/2.;   //sizes
  x(3) = 1.; //sum(ma-mi)/6.;  //radius
  x({4,6})() = rot.getArr() * (mi+.5*(ma-mi)); //center (rotated back)
  x({7,10})() = conv_quat2arr(rot);
  rndGauss(x({7,10})(), .1, true);
  x({7,10})() /= length(x({7,10})());
  
  if(verbose>1) {
    checkJacobianCP(F, x, 1e-4);
    checkHessianCP(F, x, 1e-4);
  }
  
  OptConstrained opt(x, NoArr, F, -1, OPT(
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
  
  arr x,x_best;
  double f,g, f_best, g_best;
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
  t.pos.set(x({4,6}));
  t.rot.set(x({7,-1}));
  t.rot.normalize();
  mesh.setSSBox(x(0), x(1), x(2), x(3));
  t.applyOnPointArray(mesh.V);
  
  if(t_ret!=NoTransformation)
    t_ret = t;
}
