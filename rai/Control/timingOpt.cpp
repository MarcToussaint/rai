#include "timingOpt.h"

TimingProblem::TimingProblem(const arr& _flags, const arr& _tangents, const arr& _x0, const arr& _v0, double _alpha, const arr& v_init, const arr& tau_init, bool _optTau)
  : flags(_flags),
    tangents(_tangents),
    x0(_x0),
    v0(_v0),
    alpha(_alpha),
    optTau(_optTau),
    v(v_init),
    tau(tau_init){

  CHECK_EQ(flags.nd, 2, "");
  uint K = flags.d0;
  uint d = flags.d1;

  if(tangents.N){
    CHECK_EQ(tangents.nd, 2, "");
    CHECK_EQ(tangents.d0, K-1, "");
    CHECK_EQ(tangents.d1, d, "");
  }

  if(!v.N){
    if(tangents.N) v.resize(K-1).setZero();
    else v.resize(K-1, d).setZero();
  }

  if(!tau.N) tau.resize(K) = 1.;
  CHECK_EQ(tau.N, K, "");

  //init dim and bounds
  dimension = v.N;
  if(optTau){
    dimension += tau.N;
    bounds_lo.resize(dimension) = 0.;
    bounds_up.resize(dimension) = -1.;
    for(uint k=0;k<tau.N;k++){
      bounds_lo(k) = 1e-3;
      bounds_up(k) = 1e1;
    }
  }

  //-- init feature types
  featureTypes.resize(1 + flags.d0*2*flags.d1);

  //all objectives are sos...
  featureTypes = OT_sos;

  //..except for the total time cost
  featureTypes(0) = OT_f;
}

void TimingProblem::evaluate(arr& phi, arr& J, const arr& x){
  CHECK_EQ(x.N, dimension, "");

  uint K = flags.d0;
  uint d = flags.d1;
  uint vIdx=0;
  if(optTau){
    tau = x.sub(0, K-1).reshape(K);
    vIdx=K;
  }
  if(K>1){
    if(tangents.N) v = x.sub(vIdx, -1).reshape(K-1);
    else v = x.sub(vIdx, -1).reshape(K-1, d);
  }else{
    if(tangents.N) v.resize(0, 1);
    else v.resize(0,d);
  }

  phi.resize(featureTypes.N).setZero();
  rai::SparseMatrix* JS=0;
  if(!!J){
    JS = &J.sparse();
    JS->resize(phi.N, dimension, 0);
  }
  uint m=0;

  //total time cost
  phi(0) = alpha*sum(tau);
  if(!!J){
    if(optTau){
      for(uint i=0;i<tau.N;i++) J.elem(0, i) = alpha;
    }
  }
  m += 1;

  //leaps
  for(uint k=0;k<K;k++){
    arr _x0 = x0;
    arr _v0 = v0;
    if(k){
      _x0 = flags[k-1];
      if(tangents.N){
        _v0 = v(k-1) * tangents[k-1];
        rai::SparseMatrix& J = _v0.J().sparse().resize(d, dimension, d);
        for(uint i=0;i<d;i++) J.entry(i, vIdx+(k-1), i) = tangents(k-1,i);
      }else{
        _v0 = v[k-1];
        rai::SparseMatrix& J = _v0.J().sparse().resize(d, dimension, d);
        for(uint i=0;i<d;i++) J.entry(i, vIdx+(k-1)*d+i, i) = 1.;
      }
    }
    arr _x1 = flags[k];
    arr _v1 = zeros(d);
    if(k<K-1){
      if(tangents.N){
        _v1 = v(k) * tangents[k];
        rai::SparseMatrix& J = _v1.J().sparse().resize(d, dimension, d);
        for(uint i=0;i<d;i++) J.entry(i, vIdx+k, i) = tangents(k,i);
      }else{
        _v1 = v[k];
        rai::SparseMatrix& J = _v1.J().sparse().resize(d, dimension, d);
        for(uint i=0;i<d;i++) J.entry(i, vIdx+k*d+i, i) = 1.;
      }
    }

    arr tauJ;
    if(optTau){
      tauJ.sparse().resize(1, dimension, 1).entry(0, k, 0) = 1;
    }

    arr y = CubicSplineLeapCost(_x0, _v0, _x1, _v1, tau(k), tauJ);
    phi.setVectorBlock(y.noJ(), m);
    if(!!J) J.sparse().add(y.J(), m, 0);

    m += 2*d;
  }

  CHECK_EQ(m, phi.N, "");
}

arr TimingProblem::getInitializationSample(const arr& previousOptima){
  arr x;
  if(optTau){
    x = (tau, v);
  }else{
    x = v;
  }
  //    rndGauss(x, .1, true);
  return x.reshape(-1);
}
