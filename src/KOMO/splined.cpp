/*  ------------------------------------------------------------------
    Copyright (c) 2011-2024 Marc Toussaint
    email: toussaint@tu-berlin.de

    This code is distributed under the MIT License.
    Please see <root-path>/LICENSE for details.
    --------------------------------------------------------------  */

#include "splined.h"

SplinedKOMO::SplinedKOMO(uint degree, uint numCtrlPoints, KOMO& _komo)
  : komo(_komo) {

  komo_nlp = komo.nlp();
  x0 = komo.getConfiguration_qOrg(-1);

  arr pts(numCtrlPoints+1, x0.N);
  pts[0] = x0;
  for(uint i=0; i<pts.d0; i++) {
    pts[i] = komo.getConfiguration_qOrg(komo.T*(double(i)/double(pts.d0-1))-1);
  }
  S.set(degree, pts, grid(1, 0., komo.tau*komo.T, numCtrlPoints).reshape(-1));

  //setup the NLP signature
  dimension = numCtrlPoints*x0.N;
//  bounds_lo = timing.bounds_lo;
//  bounds_up = timing.bounds_up;
  featureTypes = (komo_nlp->featureTypes);
  featureNames = (komo.featureNames);

//  if(useTorqueLimits){
//    ObjectiveTypeA add;
//    //add.resize(2*(komo.T)*8) = OT_ineq;
//    add.resize(2*pieceSubSamples*timing.waypoints.N) = OT_ineq;
//    featureTypes.append(add);
//    for(uint j=0;j<2*pieceSubSamples*timing.waypoints.d0;j++){
//      for(uint i=0;i<timing.waypoints.d1;i++) featureNames.append(STRING("torqueLimit_"<<i));
//    }
//  }
  CHECK_EQ(featureNames.N, featureTypes.N, "");

  C.copy(komo.world, true);
  C.pruneInactiveJoints();
  C.optimizeTree(true);
  C.sortFrames();
  LOG(0) <<"DOFS:" <<C.getJointNames();
  LOG(0) <<"vel limits:" <<C.getTorqueLimits(C.activeDofs, 2);
  LOG(0) <<"acc limits:" <<C.getTorqueLimits(C.activeDofs, 3);
  LOG(0) <<"torque limits:" <<C.getTorqueLimits(C.activeDofs, 4);
}

void SplinedKOMO::evaluate(arr& phi, arr& J, const arr& x) {
  arr pts = x;
  pts.prepend(x0);
  S.setPoints(pts);

  arr timeGrid = range(S.knotTimes(0), S.knotTimes(-1), komo.T);
  timeGrid.popFirst();
  arr x_fine(timeGrid.N, x0.N);
  arr x_fineJ, Jpoints, tmp, tail;
  x_fineJ.sparse().resize(timeGrid.N * x0.N, dimension, 0);
//  x_fineJ.resize(uintA{timeGrid.N, x0.N, S.ctrlPoints.d0, x0.N}).setZero();
  for(uint i=0; i<timeGrid.N; i++) {
    x_fine[i] = S.eval2(timeGrid(i), 0, Jpoints);
    tmp = Jpoints.sub(0, -1, +1+(S.degree/2), -1-(S.degree/2), 0, -1); //clip the Jacobian w.r.t. head/tail ctrl points AND first const point
    for(int k=1; k<=int(S.degree/2); k++) { //tricky: the tail ctrlPoints actually all contribute to the last Jpoints (as setPoints overwrites all tail ctrlPoints with same end point)
      for(uint i=0; i<x0.N; i++) tmp(i, -1, i) += Jpoints(i, -k, i);
    }
    tmp.reshape(x0.N, dimension);
    x_fineJ.sparse().add(tmp, i * x0.N);
  }
  //arr Jpos = T.getPosJacobian(S, timeGrid);

  CHECK_EQ(x_fine.N, komo_nlp->dimension, "wrong dof dimensions")

  arr phi1, J1;
  komo_nlp->evaluate(phi1, J1, x_fine);
  //phi1 *= 0.;
  //J1 *= 0.;

  J1 = J1 * x_fineJ;

  phi = phi1;
  J = J1;

//  phi.clear();
//  uint d0=phi.N;
//  phi.append(phi1);
//  J.sparse().resize(phi.N, dimension, 0);
//  J.sparse().add(J1, d0, 0);

//  if(useTorqueLimits){
//    //get M, F in all time slices
//    arr Mi,M;
//    M.sparse().resize(x_fine.d0*8, x_fine.d0*8, 0);
//    arr F(x_fine.d0, 8);
//    for(uint i=0;i<x_fine.d0;i++){
//      C.setJointState(x_fine[i]);
//      C.equationOfMotion(Mi, F[i].noconst(), v_fine[i], true);
//      M.sparse().add(Mi, i*8,i*8);
//    }
//    F.reshape(x_fine.d0*8);

//    //get torque limits
//    arr torqueLimits = C.getTorqueLimits(C.activeDofs, 4);
//    torqueLimits = repmat(torqueLimits, x_fine.d0, 1).reshape(-1);

//    //compute u = M * a
//    x_fine.reshape(-1);
//    a_fine.reshape(-1);

//    //u_fine = a_fine; //for gradient checking
//    u_fine = M*a_fine + F;

//    //add limit objectives
//    double scale = 1e-1;
//    d0=phi.N;
//    phi.append(scale*(u_fine - torqueLimits));
//    J.sparse().reshape(phi.N, J.d1);
//    J.sparse().add(scale*u_fine.J(),d0,0);

//    d0=phi.N;
//    phi.append(-scale*(u_fine + torqueLimits));
//    J.sparse().reshape(phi.N, J.d1);
//    J.sparse().add(-scale*u_fine.J(),d0,0);

//    a_fine.reshape(-1, 8);
//    u_fine.reshape(-1, 8);
//  }
}

arr SplinedKOMO::getInitializationSample(const arr& previousOptima) {
  arr ways = S.getPoints();
  ways.delRows(0);
  return ways;
}
