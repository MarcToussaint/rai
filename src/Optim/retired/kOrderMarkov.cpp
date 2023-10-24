/*  ------------------------------------------------------------------
    Copyright (c) 2011-2020 Marc Toussaint
    email: toussaint@tu-berlin.de

    This code is distributed under the MIT License.
    Please see <root-path>/LICENSE for details.
    --------------------------------------------------------------  */

#include "kOrderMarkov.h"

#define TT T //(T+1)
#define tlT (t<T) //(t<=T)

void conv_KOrderMarkovFunction_ConstrainedProblem(KOrderMarkovFunction& f, arr& phi, arr& J, arr& H, ObjectiveTypeA& tt, const arr& x) {
#if 1
  //set state
  f.set_x(x);

  uint T=f.get_T();
  uint k=f.get_k();
  uint dim_phi=0;
  for(uint t=0; tlT; t++) dim_phi += f.dim_phi(t);
  uint dim_xmax = 0;
  for(uint t=0; tlT; t++) { uint d=f.dim_x(t); if(d>dim_xmax) dim_xmax=d; }

  //resizing things:
  phi.resize(dim_phi).setZero();
  RowShifted* Jaux=nullptr;
  if(!!J) {
    Jaux = makeRowShifted(J, dim_phi, (k+1)*dim_xmax, x.N);
    J.setZero();
  }
  if(!!tt) tt.resize(dim_phi).setZero();

  //loop over time t
  uint Jshift=0;
  uint M=0;
  for(uint t=0; tlT; t++) {
    uint dimxbar = 0;
    for(int s=(int)t-k; s<=(int)t; s++) if(s>=0) dimxbar += f.dim_x(s);

    //query
    arr phi_t, J_t;
    ObjectiveTypeA tt_t;
    f.phi_t(phi_t, J_t, tt_t, t);
    //    CHECK_EQ(phi_t.N, f.dim_phi(t), "");
    if(!phi_t.N) continue;
    phi.setVectorBlock(phi_t, M);
    if(!!tt) tt.setVectorBlock(tt_t, M);
    if(!!J) {
      CHECK(J_t.nd==2 && J_t.d0==phi_t.N && J_t.d1==dimxbar, "");
      if(t>=k) {
        J.setMatrixBlock(J_t, M, 0);
        for(uint i=0; i<phi_t.N; i++) Jaux->rowShift(M+i) = Jshift;
        Jshift += f.dim_x(t-k);
      } else { //cut away the Jacobian w.r.t. the prefix
//        J_t.delColumns(0,(k-t)*n); //nothing to cut
        J.setMatrixBlock(J_t, M, 0);
        for(uint i=0; i<phi_t.N; i++) Jaux->rowShift(M+i) = 0;
      }
    }
    M += phi_t.N;
  }

  CHECK_EQ(M, dim_phi, "");
  if(!!J) {
    Jaux->reshift();
    Jaux->computeColPatches(true);
  }

  if(!!H) H.clear();
#else

  //probing dimensionality
  uint T=f.get_T();
  uint k=f.get_k();
  uint n=f.dim_x();
  arr x_pre=f.get_prefix();
  arr x_post=f.get_postfix();
  arr x;
  x.referTo(_x);
  x.reshape(T+1-x_post.d0, n);
  uint dim_phi=0;
  for(uint t=0; tlT; t++) dim_phi+=f.dim_phi(t);
  CHECK(x.nd==2 && x.d1==n && x.d0==T+1-x_post.d0, "");
  CHECK(x_pre.nd==2 && x_pre.d1==n && x_pre.d0==k, "prefix is of wrong dim");

  //resizing things:
  phi.resize(dim_phi).setZero();
  RowShifted* Jaux;
  if(!!J) {
    Jaux = makeRowShifted(J, dim_phi, (k+1)*n, _x.N);
    J.setZero();
  }
  if(!!tt) tt.resize(dim_phi).setZero();

  //loop over time t
  uint M=0;
  for(uint t=0; tlT; t++) {
    uint dimphi_t = f.dim_phi(t);
//    uint dimg_t   = f.dim_g(t);
//    uint dimh_t   = f.dim_h(t);
//    uint dimf_t   = dimphi_t - dimg_t - dimh_t;
    if(!dimphi_t) continue;

    //construct x_bar
    arr x_bar;
    if(t>=k) {
      if(t>=x.d0) { //x_bar includes the postfix
        x_bar.resize(k+1, n);
        for(int i=t-k; i<=(int)t; i++) x_bar[i-t+k]() = (i>=(int)x.d0)? x_post[i-x.d0] : x[i];
      } else {
        x_bar.referToRange(x, t-k, t);
      }
    } else { //x_bar includes the prefix
      x_bar.resize(k+1, n);
      for(int i=t-k; i<=(int)t; i++) x_bar[i-t+k]() = (i<0)? x_pre[k+i] : x[i];
    }

    //query
    arr phi_t, J_t;
    ObjectiveTypeA tt_t;
    f.phi_t(phi_t, J_t, tt_t, t, x_bar);
    CHECK_EQ(phi_t.N, dimphi_t, "");
    phi.setVectorBlock(phi_t, M);
    if(!!tt) tt.setVectorBlock(tt_t, M);

    //if the jacobian is returned
    if(!!J) {
      if(J_t.nd==3) J_t.reshape(J_t.d0, J_t.d1*J_t.d2);
      //insert J_t into the large J at index M
      CHECK(J_t.d0==dimphi_t&& J_t.d1==(k+1)*n, "");
      if(t>=k) {
        J.setMatrixBlock(J_t, M, 0);
        for(uint i=0; i<dimphi_t; i++) Jaux->rowShift(M+i) = (t-k)*n;
      } else { //cut away the Jacobian w.r.t. the prefix
        J_t.delColumns(0, (k-t)*n);
        J.setMatrixBlock(J_t, M, 0);
        for(uint i=0; i<dimphi_t; i++) Jaux->rowShift(M+i) = 0;
      }
    }

    M += dimphi_t;
  }

  CHECK_EQ(M, dim_phi, "");
  if(!!J) {
    Jaux->computeColPatches(true);
  }

  if(!!H) H.clear();
#endif
}
