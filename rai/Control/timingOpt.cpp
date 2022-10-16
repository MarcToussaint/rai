#include "timingOpt.h"

TimingProblem::TimingProblem(const arr& _waypoints, const arr& _tangents,
                             const arr& _x0, const arr& _v0,
                             double _timeCost, double _ctrlCost,
                             bool _optTau, bool _optLastVel,
                             const arr& v_init, const arr& tau_init,
                             double _maxVel, double _maxAcc, double _maxJer,
                             uint _refine, bool _accCont)
  : waypoints(_waypoints),
    tangents(_tangents),
    x0(_x0),
    v0(_v0),
    timeCost(_timeCost),
    ctrlCost(_ctrlCost),
    optTau(_optTau),
    optLastVel(_optLastVel),
    v(v_init),
    tau(tau_init),
    refine(_refine),
    accCont(_accCont){

  CHECK_EQ(waypoints.nd, 2, "");
  uint K = waypoints.d0;
  uint d = waypoints.d1;

  if(refine>0){
    waypoints.resize(_waypoints.d0*2, _waypoints.d1);
    for(uint k=0;k<K;k++){
      if(!k) waypoints[2*k] = .5*(x0+_waypoints[k]);
      else waypoints[2*k] = .5*(_waypoints[k-1]+_waypoints[k]);
      waypoints[2*k+1] = _waypoints[k];
    }
    K = waypoints.d0;
    if(tau.N){
      tau.resizeCopy(K);
      for(uint k=K;k--;) tau(k) = tau(k/2);
    }
  }

  uint vN=K; //number of vels to be optimized
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
  dimension = 0;
  if(optTau) dimension += tau.N;
  dimension += v.N;
  if(refine) dimension += waypoints.d0/2*d;

  bounds_lo.resize(dimension) = 0.;
  bounds_up.resize(dimension) = -1.; //means deactivated
  if(optTau){
    for(uint k=0;k<tau.N;k++){
      bounds_lo(k) = .01;
      bounds_up(k) = 10.;
    }
  }

  if(_maxVel>0.) maxVel=consts(_maxVel, d);
  if(_maxAcc>0.) maxAcc=consts(_maxAcc, d);
  if(_maxJer>0.) maxJer=consts(_maxJer, d);

  //-- init feature types
  uint m=1; //timeCost
  if(ctrlCost>0.) m += K*2*d; //control costs
  if(maxVel.N) m += K*4*d;
  if(maxAcc.N) m += K*4*d;
  if(maxJer.N) m += K*2*d;
  if(accCont) m += (K+1)*d; //acc continuity
  if(tauBarrier) m += K;
  featureTypes.resize(m);

  m=0;
  featureTypes(m) = OT_f;
  m++;
  for(uint k=0;k<K;k++){
    if(ctrlCost>0.){
      featureTypes({m,m+2*d-1}) = OT_sos; //control costs
      m += 2*d;
    }
    if(maxVel.N){
      featureTypes({m,m+4*d-1}) = OT_ineq; //maxVel
      m += 4*d;
    }
    if(maxAcc.N){
      featureTypes({m,m+4*d-1}) = OT_ineq; //maxAcc
      m += 4*d;
    }
    if(maxJer.N){
      featureTypes({m,m+2*d-1}) = OT_ineq; //maxJer
      m += 2*d;
    }
    if(accCont){
      if(k==0){
        featureTypes({m,m+d-1}) = OT_eq; //acc continuity
        m += d;
      }
      featureTypes({m,m+d-1}) = OT_eq; //acc continuity
      m += d;
    }
    if(tauBarrier){
      featureTypes(m) = OT_ineqB; //tau barrier
      m += 1;
    }
  }
  CHECK_EQ(m, featureTypes.N, "");
}

void TimingProblem::evaluate(arr& phi, arr& J, const arr& x){
  CHECK_EQ(x.N, dimension, "");

  //-- read the decision variables tau, v, refine-waypoints from x
  uint K = waypoints.d0;
  uint d = waypoints.d1;
  uint vIdx=0;
  if(optTau){
    tau = x.sub(0, K-1).reshape(tau.N);
    vIdx=K;
  }
  if(v.N){
    if(tangents.N) v = x.sub(vIdx, -1).reshape(v.N);
    else v = x.sub(vIdx, vIdx+v.d0*v.d1-1).reshape(v.d0, v.d1);
  }else{
    if(tangents.N) v.resize(0, 1);
    else v.resize(0,d);
  }
  if(refine){
    arr xway = x({vIdx+v.d0*v.d1,-1});
    xway.reshape(K/2, d);
    for(uint k=0;k<K;k+=2){
      waypoints[k] = xway[k/2];
    }
  }

  //-- initialize phi and sparse Jacobian
  phi.resize(featureTypes.N).setZero();
  if(!!J) J.sparse().resize(phi.N, dimension, 0);
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
    //get x0,v0,x1,v1,tau with (trivial) jacobians
    arr _x0 = xJ((int)k-1);
    arr _v0 = vJ((int)k-1);
    arr _x1 = xJ(k);
    arr _v1 = vJ(k);
    arr tauJ = Jtau(k);

    //- 2) control costs (sum of sqr-acc)
    if(ctrlCost>0.){
      arr y = rai::CubicSplineLeapCost(_x0, _v0, _x1, _v1, tau(k), tauJ);
      y *= ctrlCost;
      phi.setVectorBlock(y.noJ(), m);
      if(!!J) J.sparse().add(y.J(), m, 0);
      m += y.N;
    }

    // 4) vel limits
    if(maxVel.N){
      arr y = rai::CubicSplineMaxVel(_x0, _v0, _x1, _v1, tau(k), tauJ);
      for(uint i=0;i<y.N;i++) { y.elem(i) -= maxVel.elem(i%maxVel.N); }
      y *= 30.;
      phi.setVectorBlock(y.noJ(), m);
      if(!!J) J.sparse().add(y.J(), m, 0);
      m += y.N;
    }

    // 5) acc limits
    if(maxAcc.N){
      arr y = rai::CubicSplineMaxAcc(_x0, _v0, _x1, _v1, tau(k), tauJ);
      for(uint i=0;i<y.N;i++) { y.elem(i) -= maxAcc.elem(i%maxAcc.N); }
      y *= 3.;
      phi.setVectorBlock(y.noJ(), m);
      if(!!J) J.sparse().add(y.J(), m, 0);
      m += y.N;
    }

    // 6) jer limits
    if(maxJer.N){
      arr y = rai::CubicSplineMaxJer(_x0, _v0, _x1, _v1, tau(k), tauJ);
      for(uint i=0;i<y.N;i++) { y.elem(i) -= maxJer.elem(i%maxJer.N); } //y -= maxJer;
      y *= 1.;
      phi.setVectorBlock(y.noJ(), m);
      if(!!J) J.sparse().add(y.J(), m, 0);
      m += y.N;
    }

    // 7) jer limits -> continuous accel!
    if(accCont){
      if(k==0){
        arr y;
        y = rai::CubicSplineAcc0(_x0, _v0, _x1, _v1, tau(k), tauJ);
        y *= 10.;
        phi.setVectorBlock(y.noJ(), m);
        if(!!J) J.sparse().add(y.J(), m, 0);
        m += y.N;
      }

      arr y;
      if(k==K-1){
        y = rai::CubicSplineAcc1(_x0, _v0, _x1, _v1, tau(k), tauJ);
        y*=10.;
      }else{
        y = rai::CubicSplineAcc1(_x0, _v0, _x1, _v1, tau(k), tauJ)
          - rai::CubicSplineAcc0(_x1, _v1, xJ(k+1), vJ(k+1), tau(k+1), Jtau(k+1));
      }
//      y *= 1.;
      phi.setVectorBlock(y.noJ(), m);
      if(!!J) J.sparse().add(y.J(), m, 0);
      m += y.N;
    }

    if(tauBarrier){
      phi(m) = -1.1 * tau(k); //tau barrier
      if(!!J) J.sparse().add(-1.1 * tauJ, m, 0);
      m += 1;
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
  if(refine) for(uint k=0;k<waypoints.d0;k+=2){
    x.append(waypoints[k]);
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

void TimingProblem::getTaus(arr& taus){
  taus = tau;
}

arr TimingProblem::xJ(int k){
  if(k==-1) return x0;
  if(!refine || !((k+1)%2)) return waypoints[k].copy();

  uint K = waypoints.d0;
  uint d = waypoints.d1;
  uint xIdx = (optTau?K:0) + v.d0*d;

  arr xk = waypoints[k].copy();
  rai::SparseMatrix& J = xk.J().sparse().resize(d, dimension, d);
  for(uint i=0;i<d;i++) J.entry(i, xIdx+(k/2)*d+i, i) = 1.;
  return xk;
}

arr TimingProblem::vJ(int k){
  uint K = waypoints.d0;
  uint d = waypoints.d1;
  uint vIdx = (optTau?K:0);

  if(k==-1) return v0;
  if(k==(int)K-1 && !optLastVel) return zeros(d);

  arr vk = v[k].copy();
  rai::SparseMatrix& J = vk.J().sparse().resize(d, dimension, d);
  for(uint i=0;i<d;i++) J.entry(i, vIdx+k*d+i, i) = 1.;
  return vk;
}

arr TimingProblem::Jtau(int k){
  CHECK(k>=0, "");
  arr tauJ;
  if(optTau){
    tauJ.sparse().resize(1, dimension, 1).entry(0, k, 0) = 1;
  }
  return tauJ;
}
