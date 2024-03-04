/*  ------------------------------------------------------------------
    Copyright (c) 2011-2024 Marc Toussaint
    email: toussaint@tu-berlin.de

    This code is distributed under the MIT License.
    Please see <root-path>/LICENSE for details.
    --------------------------------------------------------------  */

#include "F_LeapCost.h"

CubicSplineLeapCost::CubicSplineLeapCost(const uintA& _selectedFrames) {
  setFrameIDs(_selectedFrames);
}

uint CubicSplineLeapCost::dim_phi2(const FrameL& F) {
  uint d = F_qItself().dim_phi2(F);
  return 2*d;
}

void CubicSplineLeapCost::phi2(arr& y, arr& J, const FrameL& F) {
  CHECK_EQ(order, 2, "");
//  CHECK_EQ(F.nd, 2, "");
  CHECK_EQ(F.d0, 3, "");
  arr x0 = F_qItself().eval(F({1, 1}));
  arr xT = F_qItself().eval(F({2, 2}));
  arr V = F_qItself().setOrder(1).eval(F({0, 1}));

  arr tau = F_qTime().eval({F(2, 0, 0)}); //first frame in last slice
  double Tau = tau.scalar();

  arr D;
  D = (xT-x0) - (.5*Tau)*V;
  D.J() = (xT.J()-x0.J()) - (.5*Tau)*V.J() - .5 * V * tau.J();

  double s12 = sqrt(12.);

  arr tilD;
  tilD = (s12 * pow(Tau, -1.5)) * D;
  tilD.J() = (s12 * pow(Tau, -1.5)) * D.J() + (s12 * (-1.5) * pow(Tau, -2.5)) * D * tau.J();

  arr tilV;
  tilV = pow(Tau, -0.5) * V;
  tilV.J() = pow(Tau, -0.5) * V.J() + ((-0.5) * pow(Tau, -1.5)) * V * tau.J();

  y.setBlockVector(tilD, tilV);
  if(!!J) {
    J.setBlockMatrix(tilD.J(), tilV.J());
  }
}

/* previous test
void CubicSplineLeapCost::phi2_org(arr& y, arr& J, const FrameL& F) {
    CHECK_EQ(order, 2, "");
    CHECK_EQ(F.nd, 2, "");
    CHECK_EQ(F.d0, 3, "");
    Value x0 = F_qItself().eval(F[1].reshape(1,-1));
    Value xT = F_qItself().eval(F[2].reshape(1,-1));
    Value V = F_qItself().setOrder(1).eval(F({0,1}));
    Value D;
    D = xT-x0;
    D.J() = xT.J()-x0.J();

    Value tau = F_qTime().eval({F(2,0)}); //first frame in last slice

    double D2 = sumOfSqr(D);
    double V2 = sumOfSqr(V);
    double DV = scalarProduct(D, V);
    double T = tau.scalar();
    double T2 = T*T;
    double T3 = T2*T;
    double T4 = T3*T;

    double cost = (4.*T2*V2-12.*DV*T+12.*D2)/T3;
    double dcost_dT = -(4.*T2*V2-24.*DV*T+36.*D2)/T4;
    arr dcost_dD = -(12.*T*V-24.*D)/T3;
    arr dcost_dV = (8.*T*V-12.*D)/T2;

    y.resize(1) = cost;
    if(!!J){
        F(2,0)->C.jacobian_zero(J, 1);
        J += dcost_dT * tau.J();
        J += dcost_dD * D.J();
        J += dcost_dV * V.J();
    }
}
*/
