/*  ------------------------------------------------------------------
    Copyright (c) 2011-2020 Marc Toussaint
    email: toussaint@tu-berlin.de

    This code is distributed under the MIT License.
    Please see <root-path>/LICENSE for details.
    --------------------------------------------------------------  */

#if 0
void MotionProblemFunction::phi_t(arr& phi, arr& J, uint t, const arr& x_bar, const arr& z, const arr& J_z) {
  uint T=get_T(), n=dim_x(), k=get_k();

  //assert some dimensions
  CHECK_EQ(x_bar.d0, k+1, "");
  CHECK_EQ(x_bar.d1, n, "");
  CHECK_LE(t, T, "");

  double tau=MP.tau;
  double tau2=tau*tau, tau3=tau2*tau;

  //-- transition costs
  arr h = sqrt(MP.H_rate_diag)*sqrt(tau);
  if(k==1)  phi = (x_bar[1]-x_bar[0])/tau; //penalize velocity
  if(k==2)  phi = (x_bar[2]-2.*x_bar[1]+x_bar[0])/tau2; //penalize acceleration
  if(k==3)  phi = (x_bar[3]-3.*x_bar[2]+3.*x_bar[1]-x_bar[0])/tau3; //penalize jerk
  phi = h % phi;

  if(!!J) {
    J.resize(phi.N, k+1, n);
    J.setZero();
    for(uint i=0; i<n; i++) {
      if(k==1) { J(i, 1, i) = 1.;  J(i, 0, i) = -1.; }
      if(k==2) { J(i, 2, i) = 1.;  J(i, 1, i) = -2.;  J(i, 0, i) = 1.; }
      if(k==3) { J(i, 3, i) = 1.;  J(i, 2, i) = -3.;  J(i, 1, i) = +3.;  J(i, 0, i) = -1.; }
    }
    if(k==1) J/=tau;
    if(k==2) J/=tau2;
    if(k==3) J/=tau3;
    J.reshape(phi.N, (k+1)*n);
    for(uint i=0; i<n; i++) J[i]() *= h(i);
  }

  if(!!J) CHECK_EQ(J.d0, phi.N, "");

  //-- task cost (which are taken w.r.t. x_bar[k])
  arr _phi, J_x, J_v;
  if(k>0) MP.setState(x_bar[k], (x_bar[k]-x_bar[k-1])/tau);
  else    MP.setState(x_bar[k], NoArr); //don't set velocities
  MP.getTaskCosts(_phi, J_x:NoArr), (!!J?J_v, t);
  phi.append(_phi);
  if(!!J && _phi.N) {
    arr Japp(_phi.N, (k+1)*n);
    Japp.setZero();
    Japp.setMatrixBlock(J_x + (1./tau)*J_v, 0,  k*n);    //w.r.t. x_bar[k]
    Japp.setMatrixBlock((-1./tau)*J_v, 0, (k-1)*n);      //w.r.t. x_bar[k-1]
    J.append(Japp);
  }

  if(!!J) CHECK_EQ(J.d0, phi.N, "");

  //store in CostMatrix
  if(!MP.phiMatrix.N) MP.phiMatrix.resize(get_T()+1);
  MP.phiMatrix(t) = phi;
}

#else
