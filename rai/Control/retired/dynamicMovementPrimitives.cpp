/*  ------------------------------------------------------------------
    Copyright (c) 2011-2020 Marc Toussaint
    email: toussaint@tu-berlin.de

    This code is distributed under the MIT License.
    Please see <root-path>/LICENSE for details.
    --------------------------------------------------------------  */

#include "dynamicMovementPrimitives.h"
#include "../Plot/plot.h"
#include "../KOMO/komo.h"

DynamicMovementPrimitives::DynamicMovementPrimitives(arr& y_ref_, uint nBase_, double dt_, double lambda_) {
  y_ref = y_ref_;
  dt = dt_;
  nBase = nBase_;

  lambda = lambda_;

  T = y_ref.d0*dt;
  tau = 0.5/T;

  dimY = y_ref.d1;

  alphax = 25./3.;
  alphay = 25.;
  betay = alphay/4.;

  X = 1.;
  Xd = 0.;

  Y = y_ref[0];
  Yd = (y_ref[1]-y_ref[0])/dt;
  Ydd = Y*0.;

  y_bk.append(~Y);
  yd_bk.append(~Yd);

  y0 = Y;
  goal = y_ref[y_ref.d0-1];
  amp = goal-y0;

  // init forcing function weights
  C = arr(nBase);
  H = arr(nBase);

  uint i;
  for(i=0; i<nBase; i++) {
    C(i) = exp(-alphax*i*0.5/(nBase-1));
  }

  for(i=0; i<nBase-1; i++) {
    H(i) = 0.5 / (0.65*(C(i+1) - C(i))*(C(i+1) - C(i)));
  }
  H(nBase-1) = H(nBase-2);
}

DynamicMovementPrimitives::~DynamicMovementPrimitives() {

}

void DynamicMovementPrimitives::changeGoal(const arr& goal_) {
  goal = goal_;
}

void DynamicMovementPrimitives::trainDMP() {
  uint i, j;
  PHI = zeros(y_ref.d0, nBase);
  double x_=1.;
  for(i = 0; i<y_ref.d0; i++) {

    for(j = 0; j<nBase; j++) {
      PHI(i, j) = exp(-H(j)*(x_-C(j))*(x_-C(j)));
    }

    double cs = sum(PHI.row(i));

    for(j = 0; j<nBase; j++) {
      PHI(i, j) = PHI(i, j)*(x_/cs);
    }

    x_ = x_ - alphax*x_*tau*dt;
  }

  arr FT;
  arr trajd = y_ref*0.;
  arr trajdd = y_ref*0.;

  getVel(trajd, y_ref, dt);
  getAcc(trajdd, y_ref, dt);

  for(i = 0; i<dimY; i++) {
    FT = (trajdd.col(i)/(tau*tau) - alphay*(betay*(goal(i)-y_ref.col(i)) -trajd.col(i)/tau))/amp(i);
    weights.append(~(inverse(~PHI*PHI + eye(PHI.d1)*lambda)*(~PHI)*FT));
  }
  weights=~weights;

}

void DynamicMovementPrimitives::iterate() {

  uint i;
  arr psi;
  double f;
  for(i =0; i< dimY; i++) {
    psi = exp(-H%(X-C)%(X-C));
    f = sum(~weights.col(i)*(psi)*X)/sum(psi);

    Ydd(i) = (alphay*(betay*(goal(i)-Y(i))-(Yd(i)/tau)) + (amp(i)*f))*tau*tau;
    Yd(i) = Yd(i) + Ydd(i)*dt;
    Y(i) = Y(i) + Yd(i)*dt;
  }

  Xd = -alphax*X*tau;
  X = X + Xd*dt;

  y_bk.append(~Y);
  yd_bk.append(~Yd);
  x_bk.append(X);
}

void DynamicMovementPrimitives::reset() {
  X = 1.;
  Xd = 0.;
  Y = y_ref[0];
  Yd = (y_ref[1]-y_ref[0])/dt;
  Ydd = Y*0.;

  yd_bk.clear();
  y_bk.clear();
  x_bk.clear();
}

void DynamicMovementPrimitives::plotDMP() {
  write(LIST<arr>(x_bk), "data/x_bk.dat");
  write(LIST<arr>(y_ref), "data/y_ref.dat");
  write(LIST<arr>(y_bk), "data/y_bk.dat");
  write(LIST<arr>(C), "data/C.dat");
  write(LIST<arr>(H), "data/H.dat");
  write(LIST<arr>(weights), "data/weights.dat");
}

void DynamicMovementPrimitives::printDMP() {
  std::cout <<"tau : " << tau << std::endl;
  std::cout <<"T : " << T << std::endl;
  std::cout <<"dt : " << dt << std::endl;
  std::cout <<"nBase : " << nBase << std::endl;
  std::cout <<"dimY : " << dimY << std::endl;
  std::cout <<"amp : " << amp << std::endl;
  std::cout <<"y0 : " << y0 << std::endl;
  std::cout <<"goal : " << goal << std::endl;
  std::cout <<"alphax : " << alphax << std::endl;
  std::cout <<"alphay : " << alphay << std::endl;
  std::cout <<"betay : " << betay << std::endl;
  std::cout <<"Y : " << Y << std::endl;
  //    std::cout <<"C : " << C << std::endl;
  //    std::cout <<"H : " << H << std::endl;
  std::cout <<"weights : " << weights << std::endl;
}
