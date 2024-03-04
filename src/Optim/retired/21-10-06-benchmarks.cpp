/*  ------------------------------------------------------------------
    Copyright (c) 2011-2024 Marc Toussaint
    email: toussaint@tu-berlin.de

    This code is distributed under the MIT License.
    Please see <root-path>/LICENSE for details.
    --------------------------------------------------------------  */

//===========================================================================

void ParticleAroundWalls2::getStructure(uintA& variableDimensions, uintA& featureTimes, ObjectiveTypeA& featureTypes) {
  variableDimensions = consts<uint>(n, T);

  if(!!featureTimes) featureTimes.clear();
  if(!!featureTypes) featureTypes.clear();
  for(uint t=0; t<T; t++) {
    if(!!featureTimes) featureTimes.append(consts<uint>(t, n));
    if(!!featureTypes) featureTypes.append(consts(OT_sos, n));
    if(t==T/4 || t==T/2 || t==3*T/4 || t==T-1) {
      if(!!featureTimes) featureTimes.append(consts<uint>(t, n));
      if(!!featureTypes) featureTypes.append(consts(OT_ineq, n));
    }
  }
}

void ParticleAroundWalls2::phi(arr& phi, arrA& J, arrA& H, uintA& featureTimes, ObjectiveTypeA& tt, const arr& x) {

  uint M=x.N + 4*3;
  phi.resize(M);
  if(!!J) J.resize(M);
  if(!!tt) tt.resize(M);

  uint m=0;
  for(uint t=0; t<T; t++) {
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

    //-- transition costs
    for(uint i=0; i<n; i++) {
      if(k==1) {
        phi(m) = x_bar(1, i)-x_bar(0, i); //penalize velocity
        if(!!J) { J(m).resize(k+1, n).setZero(); J(m)(1, i) = 1.;  J(m)(0, i) = -1.; }
      }
      if(k==2) {
        phi(m) = x_bar(2, i)-2.*x_bar(1, i)+x_bar(0, i); //penalize acceleration
        if(!!J) { J(m).resize(k+1, n).setZero(); J(m)(2, i) = 1.;  J(m)(1, i) = -2.;  J(m)(0, i) = 1.; }
      }
      if(k==3) {
        phi(m) = x_bar(3, i)-3.*x_bar(2, i)+3.*x_bar(1, i)-x_bar(0, i); //penalize jerk
        if(!!J) { J(m).resize(k+1, n).setZero(); J(m)(3, i) = 1.;  J(m)(2, i) = -3.;  J(m)(1, i) = +3.;  J(m)(0, i) = -1.; }
      }
      if(!!J && t<k) J(m) = J(m).sub(k-t, -1, 0, -1); //cut the prefix Jacobians
      if(!!tt) tt(m) = OT_sos;
      m++;
    }

    //-- wall constraints
    if(t==T/4 || t==T/2 || t==3*T/4 || t==T-1) {
      for(uint i=0; i<n; i++) { //add barrier costs to each dimension
        if(t==T/4) {
          phi(m) = (i+1.-x_bar(k, i)); //``greater than i+1''
          if(!!J) { J(m).resize(k+1, n).setZero(); J(m)(k, i) = -1.; }
        }
        if(t==T/2) {
          phi(m) = (x_bar(k, i)+i+1.); //``lower than -i-1''
          if(!!J) { J(m).resize(k+1, n).setZero(); J(m)(k, i) = +1.; }
        }
        if(t==3*T/4) {
          phi(m) = (i+1.-x_bar(k, i)); //``greater than i+1''
          if(!!J) { J(m).resize(k+1, n).setZero(); J(m)(k, i) = -1.; }
        }
        if(t==T-1) {
          phi(m) = (x_bar(k, i)+i+1.); //``lower than -i-1''
          if(!!J) { J(m).resize(k+1, n).setZero(); J(m)(k, i) = +1.; }
        }
        if(!!tt) tt(m) = OT_ineq;
        m++;
      }
    }
  }
  CHECK_EQ(m, M, "");
}
