/*  ------------------------------------------------------------------
    Copyright (c) 2011-2020 Marc Toussaint
    email: toussaint@tu-berlin.de

    This code is distributed under the MIT License.
    Please see <root-path>/LICENSE for details.
    --------------------------------------------------------------  */

uint ParticleAroundWalls::dim_phi(uint t) {
  uint T=get_T();
  if(t==T/2 || t==T/4 || t==3*T/4 || t==T) return 2*dim_x(t);
  return dim_x(t);
}

uint ParticleAroundWalls::dim_g(uint t) {
  uint T=get_T();
  if(t==T/2 || t==T/4 || t==3*T/4 || t==T) return dim_x(t);
  return 0;
}

void ParticleAroundWalls::phi_t(arr& phi, arr& J, ObjectiveTypeA& tt, uint t) {
  uint T=get_T(), n=dim_x(t), k=get_k();

  //-- construct x_bar
  arr x_bar;
  if(t>=k) {
    x_bar.referToRange(x, t-k, t);
  } else { //x_bar includes the prefix
    x_bar.resize(k+1, n);
    for(int i=t-k; i<=(int)t; i++) x_bar[i-t+k]() = (i<0)? zeros(n) : x[i];
  }

  //-- assert some dimensions
  CHECK_EQ(x_bar.d0, k+1, "");
  CHECK_EQ(x_bar.d1, n, "");
  CHECK_LE(t, T, "");

  //-- transition costs: append to phi
  if(k==1)  phi = x_bar[1]-x_bar[0]; //penalize velocity
  if(k==2)  phi = x_bar[2]-2.*x_bar[1]+x_bar[0]; //penalize acceleration
  if(k==3)  phi = x_bar[3]-3.*x_bar[2]+3.*x_bar[1]-x_bar[0]; //penalize jerk
  if(!!tt) tt = consts(OT_sos, n);

  //-- wall constraints: append to phi
  //Note: here we append to phi ONLY in certain time slices ->
  //the dimensionality of phi may very with time slices; see dim_phi(uint t)
  for(uint i=0; i<n; i++) { //add barrier costs to each dimension
    if(t==T/4)   phi.append((i+1.-x_bar(k, i))); //``greater than i+1''
    if(t==T/2)   phi.append((x_bar(k, i)+i+1.)); //``lower than -i-1''
    if(t==3*T/4) phi.append((i+1.-x_bar(k, i))); //``greater than i+1''
    if(t==T)     phi.append((x_bar(k, i)+i+1.)); //``lower than -i-1''
  }
  if(!!tt && (t==T/4 || t==T/2 || t==3*T/4 || t==T)) tt.append(OT_ineq, n);

  uint m=phi.N;
  CHECK_EQ(m, dim_phi(t), "");
  if(!!tt) CHECK_EQ(m, tt.N, "");

  if(!!J) { //we also need to return the Jacobian
    J.resize(m, k+1, n).setZero();

    //-- transition costs
    for(uint i=0; i<n; i++) {
      if(k==1) { J(i, 1, i) = 1.;  J(i, 0, i) = -1.; }
      if(k==2) { J(i, 2, i) = 1.;  J(i, 1, i) = -2.;  J(i, 0, i) = 1.; }
      if(k==3) { J(i, 3, i) = 1.;  J(i, 2, i) = -3.;  J(i, 1, i) = +3.;  J(i, 0, i) = -1.; }
    }

    //-- wall constraints
    for(uint i=0; i<n; i++) {
      if(t==T/4)   J(n+i, k, i) = -1.;
      if(t==T/2)   J(n+i, k, i) = +1.;
      if(t==3*T/4) J(n+i, k, i) = -1.;
      if(t==T)     J(n+i, k, i) = +1.;
    }
  }

  J.reshape(m, (k+1)*n);
  if(!!J && t<k) J.delColumns(0, (k-t)*n);
}
