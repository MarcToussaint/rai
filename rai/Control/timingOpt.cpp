#include "timingOpt.h"

TimingProblem::TimingProblem(const arr& _waypoints, const arr& _tangents,
                             const arr& _x0, const arr& _v0,
                             double _timeCost, double _ctrlCost,
                             bool _optTau, bool _optLastVel,
                             const arr& v_init, const arr& tau_init,
                             double _maxVel, double _maxAcc, double _maxJer)
  : waypoints(_waypoints),
    tangents(_tangents),
    x0(_x0),
    v0(_v0),
    timeCost(_timeCost),
    ctrlCost(_ctrlCost),
    optTau(_optTau),
    optLastVel(_optLastVel),
    maxVel(_maxVel),
    maxAcc(_maxAcc),
    maxJer(_maxJer),
    v(v_init),
    tau(tau_init){

  CHECK_EQ(waypoints.nd, 2, "");
  uint K = waypoints.d0;
  uint d = waypoints.d1;

  uint vN=K;
  if(!optLastVel) vN -=1;

  if(tangents.N){
    CHECK_EQ(tangents.nd, 2, "");
    CHECK_EQ(tangents.d0, vN, "");
    CHECK_EQ(tangents.d1, d, "");
  }

  if(!v.N){
    if(tangents.N) v.resize(vN).setZero();
    else v.resize(vN, d).setZero();
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
  uint m=1; //timeCost
  if(ctrlCost>0.) m += K*2*d; //control costs
  if(maxAcc>0.) m += K*4*d;
  featureTypes.resize(m);

  m=0;
  featureTypes(m) = OT_f;
  m++;
  for(uint k=0;k<K;k++){
    if(ctrlCost>0.){
      featureTypes({m,m+2*d-1}) = OT_sos; //control costs
      m += 2*d;
    }
    if(maxAcc>0.){
      featureTypes({m,m+4*d-1}) = OT_ineq; //maxAcc
      m += 4*d;
    }
  }
}

void TimingProblem::evaluate(arr& phi, arr& J, const arr& x){
  CHECK_EQ(x.N, dimension, "");

  //-- read the decision variables tau,v from x
  uint K = waypoints.d0;
  uint d = waypoints.d1;
  uint vIdx=0;
  if(optTau){
    tau = x.sub(0, K-1).reshape(K);
    vIdx=K;
  }
  if(K>1){
    if(tangents.N) v = x.sub(vIdx, -1).reshape(v.N);
    else v = x.sub(vIdx, -1).reshape(v.d0, v.d1);
  }else{
    if(tangents.N) v.resize(0, 1);
    else v.resize(0,d);
  }

  //-- initialize phi and sparse Jacobian
  phi.resize(featureTypes.N).setZero();
  rai::SparseMatrix* JS=0;
  if(!!J){
    JS = &J.sparse();
    JS->resize(phi.N, dimension, 0);
  }
  uint m=0;

  // 1) total time cost
  if(timeCost>0.){
    phi(m) = timeCost*sum(tau);
    if(!!J){
      if(optTau){
        for(uint i=0;i<tau.N;i++) J.elem(m, i) = timeCost;
      }
    }
    m += 1;
  }

  // loop through segments for other objectives
  for(uint k=0;k<K;k++){
    //get x0,v0,x1,v1 with jacobians
    arr _x0 = x0;
    arr _v0 = v0;
    if(k){
      _x0 = waypoints[k-1];
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
    arr _x1 = waypoints[k];
    arr _v1 = zeros(d);
    if(k<K-1 || optLastVel){
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

    //get tau with jacobian
    arr tauJ;
    if(optTau){
      tauJ.sparse().resize(1, dimension, 1).entry(0, k, 0) = 1;
    }

    //- 2) control costs (sum of sqr-acc
    if(ctrlCost>0.){
      arr y = rai::CubicSplineLeapCost(_x0, _v0, _x1, _v1, tau(k), tauJ);
      y *= ctrlCost;
      phi.setVectorBlock(y.noJ(), m);
      if(!!J) J.sparse().add(y.J(), m, 0);
      m += y.N;
    }

    // 3) acc limits
    if(maxAcc>0.){
      arr y = rai::CubicSplineMaxAcc(_x0, _v0, _x1, _v1, tau(k), tauJ);
      y -= maxAcc;
      y *= 1e1;
      phi.setVectorBlock(y.noJ(), m);
      if(!!J) J.sparse().add(y.J(), m, 0);
      m += y.N;
    }
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

void TimingProblem::getVels(arr& vel){
  if(tangents.N) vel = v%tangents;
  else vel = v;
  if(!optLastVel) vel.append(zeros(waypoints.d1));
  vel.reshape(waypoints.d0, waypoints.d1);
}
