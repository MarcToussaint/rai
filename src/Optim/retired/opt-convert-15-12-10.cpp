/*  ------------------------------------------------------------------
    Copyright (c) 2011-2020 Marc Toussaint
    email: toussaint@tu-berlin.de

    This code is distributed under the MIT License.
    Please see <root-path>/LICENSE for details.
    --------------------------------------------------------------  */

#include "convert.h"

//the Convert is essentially only a ``garbage collector'', creating all the necessary conversion objects and then deleting them on destruction
Convert::Convert(const ScalarFunction& p):kom(nullptr), cstyle_fs(nullptr), cstyle_fv(nullptr), data(nullptr) { sf=p; }
Convert::Convert(const VectorFunction& p):kom(nullptr), cstyle_fs(nullptr), cstyle_fv(nullptr), data(nullptr) { vf=p; }
//Convert::Convert(QuadraticFunction& p){ sf=&p; }
//Convert::Convert(VectorChainFunction& p) { vcf=&p; }
//Convert::Convert(QuadraticChainFunction& p) { qcf=&p; }
Convert::Convert(KOrderMarkovFunction& p):kom(&p), cstyle_fs(nullptr), cstyle_fv(nullptr), data(nullptr) { }
Convert::Convert(double(*fs)(arr*, const arr&, void*), void* data):kom(nullptr), cstyle_fs(fs), cstyle_fv(nullptr), data(data) {  }
Convert::Convert(void (*fv)(arr&, arr*, const arr&, void*), void* data):kom(nullptr), cstyle_fs(nullptr), cstyle_fv(fv), data(data) {  }

#ifndef libRoboticsCourse
//Convert::Convert(ControlledSystem& p) { cs=&p; }
#endif

Convert::~Convert() {
}

//===========================================================================
//
// casting methods
//

Convert::operator ScalarFunction() {
  if(!sf) {
    if(cstyle_fs) sf = convert_cstylefs_ScalarFunction(cstyle_fs, data);
    else {
      if(!vf) vf = this->operator VectorFunction();
      if(vf)  sf = convert_VectorFunction_ScalarFunction(vf);
    }
  }
  if(!sf) HALT("");
  return sf;
}

Convert::operator VectorFunction() {
  if(!vf) {
    if(cstyle_fv)
      vf = convert_cstylefv_VectorFunction(cstyle_fv, data);
    else {
      if(kom) vf = convert_KOrderMarkovFunction_VectorFunction(*kom);
    }
  }
  if(!vf) HALT("");
  return vf;
}

Convert::operator ConstrainedProblem() {
  if(!cpm) {
    if(kom) cpm = convert_KOrderMarkovFunction_ConstrainedProblem(*kom);
  }
  if(!cpm) HALT("");
  return cpm;
}

Convert::operator KOrderMarkovFunction& () {
  if(!kom) {
// #ifndef libRoboticsCourse
//     if(cs) kom = new sConvert::ControlledSystem_2OrderMarkovFunction(*cs);
// #endif
  }
  if(!kom) HALT("");
  return *kom;
}

//===========================================================================
//
// actual convertion routines
//

ScalarFunction convert_cstylefs_ScalarFunction(double(*fs)(arr*, const arr&, void*), void* data) {
  return [&fs, data](arr& g, arr& H, const arr& x) -> double {
    if(!!H) NIY;
    return fs(&g, x, data);
  };
}

VectorFunction convert_cstylefv_VectorFunction(void (*fv)(arr&, arr*, const arr&, void*), void* data) {
  return [&fv, data](arr& y, arr& J, const arr& x) -> void {
    fv(y, &J, x, data);
  };
}

ScalarFunction convert_VectorFunction_ScalarFunction(const VectorFunction& f) {
  return [&f](arr& g, arr& H, const arr& x) -> double {
    arr y, J;
    f(y, (!!g?J:NoArr), x);
    //  if(J.special==arr::RowShiftedST) J = unpack(J);
    if(!!g) { g = comp_At_x(J, y); g *= 2.; }
    if(!!H) { H = comp_At_A(J); H *= 2.; }
    return sumOfSqr(y);
  };
}

void conv_KOrderMarkovFunction_ConstrainedProblem(KOrderMarkovFunction& f, arr& phi, arr& J, arr& H, ObjectiveTypeA& tt, const arr& _x) {
  //probing dimensionality
  uint T=f.get_T();
  uint k=f.get_k();
  uint n=f.dim_x();
  uint dim_z=f.dim_z();
  arr x_pre=f.get_prefix();
  arr x_post=f.get_postfix();
  arr x, z;
  if(dim_z) { //split _x into (x,z)
    x.referTo(_x);
    x.reshape((T+1-x_post.d0)*n + dim_z);
    z.referToRange(x, -(int)dim_z, -1);
    x.referToRange(_x, 0, -(int)dim_z-1);
    x.reshape(T+1-x_post.d0, n);
  } else { //there is no z -> x = _x
    x.referTo(_x);
    x.reshape(T+1-x_post.d0, n);
  }
  uint dim_phi=0;
  for(uint t=0; t<=T; t++) dim_phi+=f.dim_phi(t);
  CHECK(x.nd==2 && x.d1==n && x.d0==T+1-x_post.d0, "");
  CHECK(x_pre.nd==2 && x_pre.d1==n && x_pre.d0==k, "prefix is of wrong dim");

  //resizing things:
  phi.resize(dim_phi).setZero();
  RowShifted* Jaux, *Jzaux;
  arr* Jz;
  if(!!J) {
    Jaux = makeRowShifted(J, dim_phi, (k+1)*n, _x.N);
    J.setZero();
    if(dim_z) {
      Jz = new arr(dim_phi, dim_z);
      Jzaux = makeRowShifted(*Jz, dim_phi, dim_z, _x.N);
      Jz->setZero();
      Jaux->nextInSum = Jz; //this is crucial: the returned J contains a quite hidden link to Jz
    }
  }
  if(!!tt) tt.resize(dim_phi).setZero();

  //loop over time t
  uint M=0;
  for(uint t=0; t<=T; t++) {
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
        if(!dim_z) x_bar.referToRange(x, t-k, t);
        else x_bar = x.sub(t-k, t, 0, -1); //need to copy as we will augment
      }
    } else { //x_bar includes the prefix
      x_bar.resize(k+1, n);
      for(int i=t-k; i<=(int)t; i++) x_bar[i-t+k]() = (i<0)? x_pre[k+i] : x[i];
    }
    if(dim_z) { //append the constant variable to x_bar
      x_bar.insColumns(x_bar.d1, dim_z);
      for(uint i=0; i<=k; i++) x_bar[i]({-dim_z, -1})=z;
    }

    //query
    arr phi_t, J_t, Jz_t;
    ObjectiveTypeA tt_t;
    f.phi_t(phi_t, J_t, tt_t, t, x_bar);
    CHECK_EQ(phi_t.N, dimphi_t, "");
    phi.setVectorBlock(phi_t, M);
    if(!!tt) tt.setVectorBlock(tt_t, M);

    //if the jacobian is returned
    if(!!J) {
      if(J_t.nd==3) J_t.reshape(J_t.d0, J_t.d1*J_t.d2);
      //in case of a z: decompose J_t -> (J_t_xbar, Jz_t) //TODO: inefficient
      if(dim_z) {
        Jz_t.resize(J_t.d0, dim_z).setZero();
        J_t.reshape(J_t.d0, (k+1)*(n+dim_z));
        for(uint i=0; i<=k; i++) {
          J_t.setMatrixBlock(J_t.sub(0, -1, i*(n+dim_z), i*(n+dim_z)+n-1), 0, i*n);
          Jz_t += J_t.sub(0, -1, i*(n+dim_z)+n, i*(n+dim_z)+n+dim_z-1); //we add up the Jacobians
        }
        J_t.delColumns((k+1)*n, (k+1)*dim_z);
      }
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
      //insert Jz_t into the large Jz at index M
      if(dim_z) {
        CHECK(!Jz_t.N || (Jz_t.d0==dimphi_t&& Jz_t.d1==dim_z), "");
        Jz->setMatrixBlock(Jz_t, M, 0);
        for(uint i=0; i<dimphi_t; i++) Jzaux->rowShift(M+i) = x.N;
      }
    }

    M += dimphi_t;
  }

  CHECK_EQ(M, dim_phi, "");
  if(!!J) {
    Jaux->computeColPatches(true);
    if(dim_z) Jzaux->computeColPatches(false);
  }

  if(!!H) {
    H.clear();
  }
}

ConstrainedProblem convert_KOrderMarkovFunction_ConstrainedProblem(KOrderMarkovFunction& f) {
  return [&f](arr& phi, arr& J, arr& H, ObjectiveTypeA& tt, const arr& x) -> void {
    conv_KOrderMarkovFunction_ConstrainedProblem(f, phi, J, H, tt, x);
  };
}

void conv_KOrderMarkovFunction_VectorFunction(KOrderMarkovFunction& f, arr& phi, arr& J, const arr& _x) {
#if 0 //non-packed Jacobian
  //probing dimensionality
  uint T=f->get_T();
  uint k=f->get_k();
  uint n=f->get_n();
  uint M=0;
  for(uint t=0; t<=T-k; t++) M+=f->get_m(t);
  CHECK(x.nd==2 && x.d1==n && x.d0==(T+1), "");
  //resizing things:
  phi.resize(M);   phi.setZero();
  if(!!J) { J.resize(M, x.N); J.setZero(); }
  M=0;
  uint m_t;
  for(uint t=0; t<=T-k; t++) {
    m_t = f->get_m(t);
    arr phi_t, J_t;
    f->phi_t(phi_t, J_t, t, x({t, t+k}));
    CHECK_EQ(phi_t.N, m_t, "");
    phi.setVectorBlock(phi_t, M);
    if(!!J) {
      if(J_t.nd==3) J_t.reshape(J_t.d0, J_t.d1*J_t.d2);
      CHECK(J_t.d0==m_t && J_t.d1==(k+1)*n, "");
      J.setMatrixBlock(J_t, M, t*n);
    }
    M += m_t;
  }
#else
  //probing dimensionality
  uint T=f.get_T();
  uint k=f.get_k();
  uint n=f.dim_x();
  uint dim_z=f.dim_z();
  uint dim_Phi=0;
  arr x_pre=f.get_prefix();
  arr x_post=f.get_postfix();
  arr x, z;
  if(dim_z) {
    x.referTo(_x);
    x.reshape((T+1-x_post.d0)*n + dim_z);
    z.referToRange(x, -(int)dim_z, -1);
    x.referToRange(_x, 0, -(int)dim_z-1);
    x.reshape(T+1-x_post.d0, n);
  } else {
    x.referTo(_x);
    x.reshape(T+1-x_post.d0, n);
  }
  for(uint t=0; t<=T; t++) dim_Phi+=f.dim_phi(t);
  CHECK(x.nd==2 && x.d1==n && x.d0==T+1-x_post.d0, "");
  CHECK(x_pre.nd==2 && x_pre.d1==n && x_pre.d0==k, "prefix is of wrong dim");

  //resizing things:
  phi.resize(dim_Phi);   phi.setZero();
  RowShifted* Jaux, *Jzaux;
  arr* Jz;
  if(!!J) {
    Jaux = makeRowShifted(J, dim_Phi, (k+1)*n, _x.N);
    J.setZero();
    if(dim_z) {
      Jz = new arr(dim_Phi, dim_z);
      Jz->setZero();
      Jaux->nextInSum = Jz;
      Jzaux = makeRowShifted(*Jz, dim_Phi, dim_z, _x.N);
    }
  }

  //loop over time t
  uint M=0;
  for(uint t=0; t<=T; t++) {
    uint dimf_t = f.dim_phi(t);
    if(!dimf_t) continue;

    //construct x_bar
    arr x_bar;
    if(t>=k) {
      if(t>=x.d0) { //x_bar includes the postfix
        x_bar.resize(k+1, n);
        for(int i=t-k; i<=(int)t; i++) x_bar[i-t+k]() = (i>=(int)x.d0)? x_post[i-x.d0] : x[i];
      } else {
        if(!dim_z) x_bar.referToRange(x, t-k, t);
        else x_bar = x.sub(t-k, t, 0, -1); //need to copy as we will augment
      }
    } else { //x_bar includes the prefix
      x_bar.resize(k+1, n);
      for(int i=t-k; i<=(int)t; i++) x_bar[i-t+k]() = (i<0)? x_pre[k+i] : x[i];
    }
    if(dim_z) { //append the constant variable to x_bar
      x_bar.insColumns(x_bar.d1, dim_z);
      for(uint i=0; i<=k; i++) x_bar[i]({-dim_z, -1})=z;
    }

    //query
    arr f_t, J_t, Jz_t;
    f.phi_t(f_t, J_t, NoObjectiveTypeA, t, x_bar);
    CHECK_EQ(f_t.N, dimf_t, "");
    phi.setVectorBlock(f_t, M);
    if(!!J) {
      if(J_t.nd==3) J_t.reshape(J_t.d0, J_t.d1*J_t.d2);
      if(dim_z) { //decompose J_t//TODO: inefficient
        Jz_t.resize(J_t.d0, dim_z).setZero();
        J_t.reshape(J_t.d0, (k+1)*(n+dim_z));
        for(uint i=0; i<=k; i++) {
          J_t.setMatrixBlock(J_t.sub(0, -1, i*(n+dim_z), i*(n+dim_z)+n-1), 0, i*n);
          Jz_t += J_t.sub(0, -1, i*(n+dim_z)+n, i*(n+dim_z)+n+dim_z-1); //we add up the Jacobians
        }
        J_t.delColumns((k+1)*n, (k+1)*dim_z);
      }
      CHECK(J_t.d0==dimf_t&& J_t.d1==(k+1)*n, "");
      if(t>=k) {
        J.setMatrixBlock(J_t, M, 0);
        for(uint i=0; i<dimf_t; i++) Jaux->rowShift(M+i) = (t-k)*n;
      } else { //cut away the Jacobian w.r.t. the prefix
        J_t.delColumns(0, (k-t)*n);
        J.setMatrixBlock(J_t, M, 0);
        for(uint i=0; i<dimf_t; i++) Jaux->rowShift(M+i) = 0;
      }
      if(dim_z) {
        CHECK(!Jz_t.N || (Jz_t.d0==dimf_t&& Jz_t.d1==dim_z), "");
        Jz->setMatrixBlock(Jz_t, M, 0);
        for(uint i=0; i<dimf_t; i++) Jzaux->rowShift(M+i) = x.N;
      }
    }
    M += dimf_t;
  }

  CHECK_EQ(M, dim_Phi, "");
  if(!!J) {
    Jaux->computeColPatches(true);
    if(dim_z) Jzaux->computeColPatches(false);
  }
#endif
}

VectorFunction convert_KOrderMarkovFunction_VectorFunction(KOrderMarkovFunction& f) {
  return [&f](arr& y, arr& J, const arr& x) -> void {
    conv_KOrderMarkovFunction_VectorFunction(f, y, J, x);
  };
}

//collect the constraints from a KOrderMarkovFunction
double conv_KOrderMarkovFunction_ConstrainedProblem(KOrderMarkovFunction& f, arr& df, arr& Hf, arr& g, arr& Jg, arr& h, arr& Jh, const arr& x) {
#if 0 //old way
  //probing dimensionality
  uint T=f->get_T();
  uint k=f->get_k();
  uint n=f->dim_x();
  arr x_pre=f->get_prefix();
  arr x_post=f->get_postfix();

  CHECK(x.nd==2 && x.d1==n && x.d0==(T+1)-x_post.d0, "");
  CHECK(x_pre.nd==2 && x_pre.d1==n && x_pre.d0==k, "prefix is of wrong dim");
  CHECK(!x_post.N || (x_post.nd==2 && x_post.d1==n), "postfix is of wrong dim");

  //resizing things:
  uint meta_phid = dim_phi();
  uint meta_gd = dim_g();
  uint meta_yd = meta_phid - meta_gd;

  bool getJ = (&df || &Hf || &Jg);

  arr meta_y, meta_Jy;
  RowShifted* Jy_aux, *Jg_aux;
  meta_y.resize(meta_yd);
  if(!!g) g.resize(meta_gd);
  if(getJ) { Jy_aux = makeRowShifted(meta_Jy, meta_yd, (k+1)*n, x.N); meta_Jy.setZero(); }
  if(!!Jg) { Jg_aux = makeRowShifted(Jg, meta_gd, (k+1)*n, x.N); Jg.setZero(); }

  uint y_count=0;
  uint g_count=0;

  for(uint t=0; t<=T; t++) {
    uint phid = f->dim_phi(t);
    uint dimg_t   = f->dim_g(t);
    uint m_t   = phid-dimg_t;
    if(!phid) continue;
    arr x_bar, phi_t, J_t, f_t, Jf_t, g_t, Jg_t;

    //construct x_bar
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

    //query the phi
    f->phi_t(phi_t, (getJ?J_t:NoArr), t, x_bar);
    if(getJ) if(J_t.nd==3) J_t.reshape(J_t.d0, J_t.d1*J_t.d2);
    CHECK_EQ(phi_t.N, phid, "");
    if(getJ) CHECK(J_t.d0==phid && J_t.d1==(k+1)*n, "");

    //insert in meta_y
    f_t.referToRange(phi_t, 0, m_t-1);
    CHECK_EQ(f_t.N, m_t, "");
    meta_y.setVectorBlock(f_t, y_count);
    if(getJ) {
      Jf_t.referToRange(J_t, 0, m_t-1);
      if(t>=k) {
        meta_Jy.setMatrixBlock(Jf_t, y_count, 0);
        for(uint i=0; i<Jf_t.d0; i++) Jy_aux->rowShift(y_count+i) = (t-k)*n;
      } else { //cut away the Jacobian w.r.t. the prefix
        Jf_t.dereference();
        Jf_t.delColumns(0, (k-t)*n);
        meta_Jy.setMatrixBlock(Jf_t, y_count, 0);
        for(uint i=0; i<Jf_t.d0; i++) Jy_aux->rowShift(y_count+i) = 0;
      }
    }
    y_count += m_t;

    //insert in meta_g
    if(dimg_t) {
      g_t.referToRange(phi_t, m_t, -1);
      CHECK_EQ(g_t.N, dimg_t, "");
      if(!!g) g.setVectorBlock(g_t, g_count);
      if(!!Jg) {
        Jg_t.referToRange(J_t, m_t, -1);
        if(t>=k) {
          Jg.setMatrixBlock(Jg_t, g_count, 0);
          for(uint i=0; i<Jg_t.d0; i++) Jg_aux->rowShift(g_count+i) = (t-k)*n;
        } else { //cut away the Jacobian w.r.t. the prefix
          Jg_t.dereference();
          Jg_t.delColumns(0, (k-t)*n);
          Jg.setMatrixBlock(Jg_t, g_count, 0);
          for(uint i=0; i<Jg_t.d0; i++) Jg_aux->rowShift(g_count+i) = 0;
        }
      }
      g_count += dimg_t;
    }
  }
  CHECK_EQ(y_count, meta_y.N, "");
  if(!!g) CHECK_EQ(g_count, g.N, "");
  if(getJ) Jy_aux->computeColPatches(true);
  if(!!Jg) Jg_aux->computeColPatches(true);
  //if(!!J) J=Jaux->unpack();

  //finally, compute the scalar function
  if(!!df) { df = comp_At_x(meta_Jy, meta_y); df *= 2.; }
  if(!!Hf) { Hf = comp_At_A(meta_Jy); Hf *= 2.; }
  return sumOfSqr(meta_y);
#else

  /* Basically: first get (phi, J, Jz) as VectorFunction; then loop through all elements of the vector phi
   * and 'sort' it either into a cost or a constraint. This sorting has to be done for both, the vector
   * and its Jacobian, to give (y, Jy, Jyz) for the costs, and (g, Jg, Jgz) for the constraints
   * The 'z' Jacobians complicate only a bit: they are perfectly parallel to the ordinary 'x'-Jacobians
   * -> everything done with the 'x'-Jacobians is also done for the 'z'-Jacobians */

//  sConvert::KOrderMarkovFunction_VectorFunction F(*f);
  arr phi, J;
  bool getJ = (&df) || (&Hf) || (&Jg) || (&Jh);
  conv_KOrderMarkovFunction_VectorFunction(f, phi, (getJ?J:NoArr), x);
  RowShifted* J_aux = (RowShifted*)J.aux;

  //resizing things:
  uint T=f.get_T();
  uint dimphi = 0;  for(uint t=0; t<=T; t++) dimphi += f.dim_phi(t);
  uint dimg = 0;    for(uint t=0; t<=T; t++) dimg += f.dim_g(t);
  uint dimh = 0;    for(uint t=0; t<=T; t++) dimh += f.dim_h(t);
  uint dimy = dimphi - dimg - dimh;
  CHECK_EQ(phi.N, dimphi, "");

  arr y, Jy;
  RowShifted* Jy_aux, *Jg_aux, *Jh_aux;
  y.resize(dimy);
  if(!!g) g.resize(dimg);
  if(!!h) h.resize(dimh);
  if(getJ) Jy_aux = makeRowShifted(Jy, dimy, J.d1, J_aux->real_d1);
  if(!!Jg)  Jg_aux = makeRowShifted(Jg, dimg, J.d1, J_aux->real_d1);
  if(!!Jh)  Jh_aux = makeRowShifted(Jh, dimh, J.d1, J_aux->real_d1);

  //if there is a z
  uint dimz = f.dim_z();
  arr* Jz, *Jyz, *Jgz, *Jhz;
  RowShifted* Jz_aux, *Jyz_aux, *Jgz_aux, *Jhz_aux;
  if(dimz && getJ) {
    Jz = J_aux->nextInSum;
    Jz_aux = (RowShifted*)Jz->aux;
    { Jyz = new arr(dimy, dimz);  Jy_aux->nextInSum = Jyz;  Jyz_aux = makeRowShifted(*Jyz, dimy, Jz->d1, Jz_aux->real_d1); }
    if(!!Jg) { Jgz = new arr(dimg, dimz);  Jg_aux->nextInSum = Jgz;  Jgz_aux = makeRowShifted(*Jgz, dimg, Jz->d1, Jz_aux->real_d1); }
    if(!!Jh) { Jhz = new arr(dimh, dimz);  Jh_aux->nextInSum = Jhz;  Jhz_aux = makeRowShifted(*Jhz, dimh, Jz->d1, Jz_aux->real_d1); }
  }

  //loop over time t
  uint M=0, y_count=0, g_count=0, h_count=0;
  for(uint t=0; t<=T; t++) {
    uint dimphi_t = f.dim_phi(t);
    uint dimg_t   = f.dim_g(t);
    uint dimh_t   = f.dim_h(t);
    uint dimf_t   = dimphi_t - dimg_t - dimh_t;

    //split up: push cost terms into y
    if(dimf_t) {
      y.setVectorBlock(phi({M, M+dimf_t-1}), y_count);
      if(getJ) {
        Jy.setMatrixBlock(J({M, M+dimf_t-1}), y_count, 0);
        for(uint i=0; i<dimf_t; i++) Jy_aux->rowShift(y_count+i) = J_aux->rowShift(M+i);
        if(dimz) {
          Jyz->setMatrixBlock(Jz->operator()({M, M+dimf_t-1}), y_count, 0);
          for(uint i=0; i<dimf_t; i++) Jyz_aux->rowShift(y_count+i) = Jz_aux->rowShift(M+i);
        }
      }
      M += dimf_t;
      y_count += dimf_t;
    }
    //split up: push inequality terms into g
    if(!!g && dimg_t) g.setVectorBlock(phi({M, M+dimg_t-1}), g_count);
    if(!!Jg && dimg_t) {
      Jg.setMatrixBlock(J({M, M+dimg_t-1}), g_count, 0);
      for(uint i=0; i<dimg_t; i++) Jg_aux->rowShift(g_count+i) = J_aux->rowShift(M+i);
      if(dimz) {
        Jgz->setMatrixBlock(Jz->operator()({M, M+dimg_t-1}), g_count, 0);
        for(uint i=0; i<dimg_t; i++) Jgz_aux->rowShift(g_count+i) = Jz_aux->rowShift(M+i);
      }
    }
    M += dimg_t;
    g_count += dimg_t;

    //split up: push equality terms into h
    if(!!h && dimh_t) h.setVectorBlock(phi({M, M+dimh_t-1}), h_count);
    if(!!Jh && dimh_t) {
      Jh.setMatrixBlock(J({M, M+dimh_t-1}), h_count, 0);
      for(uint i=0; i<dimh_t; i++) Jh_aux->rowShift(h_count+i) = J_aux->rowShift(M+i);
      if(dimz) {
        Jhz->setMatrixBlock(Jz->operator()({M, M+dimh_t-1}), h_count, 0);
        for(uint i=0; i<dimh_t; i++) Jhz_aux->rowShift(h_count+i) = Jz_aux->rowShift(M+i);
      }
    }
    M += dimh_t;
    h_count += dimh_t;
  }
  CHECK_EQ(M, dimphi, "");
  CHECK_EQ(y_count, dimy, "");
  if(!!g) CHECK_EQ(g_count, dimg, "");
  if(!!h) CHECK_EQ(h_count, dimh, "");
  if(getJ) Jy_aux->computeColPatches(true);
  if(!!Jg) Jg_aux->computeColPatches(true);
  if(!!Jh) Jh_aux->computeColPatches(true);

  //finally, compute the scalar function
  if(!!df) { df = comp_At_x(Jy, y); df *= 2.; }
  if(!!Hf) { Hf = comp_At_A(Jy); Hf *= 2.; }
  return sumOfSqr(y);
#endif
}

//===========================================================================

RUN_ON_INIT_BEGIN()
rai::Array<ObjectiveType>::memMove=true;
RUN_ON_INIT_END()

