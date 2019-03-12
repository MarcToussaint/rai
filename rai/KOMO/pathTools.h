#include <Core/array.h>

arr getVelocities(const arr& q, double tau) {
  arr v;
  v.resizeAs(q);
  for(uint t=1; t<q.d0-1; t++) {
    v[t] = (q[t+1]-q[t-1])/(2.*tau);
  }
  v[0] = (q[1] - q[0])/tau;
  v[T] = (q[T] - q[T-1])/tau;
  return v;
}

void getAccelerations(const arr& q, double tau) {
  arr a;
  a.resizeAs(q);
  for(uint t=1; t<q.d0-1; t++)  a[t] = (q[t+1] + q[t-1] - 2.*q[t])/(tau*tau);
  a[0] = a[1]/2.;
  a[T] = a[T-1]/2.;
  return a;
}

arr sineProfile(const arr& q0, const arr& qT,uint T) {
  arr q(T+1,q0.N);
  for(uint t=0; t<=T; t++) q[t] = q0 + .5 * (1.-cos(RAI_PI*t/T)) * (qT-q0);
  return q;
}

arr reverseTrajectory(const arr& q) {
  uint T=q.d0-1;
  arr r(T+1, q.d1);
  for(uint t=0; t<=T; t++) r[T-t] = q[t];
  return r;
}

