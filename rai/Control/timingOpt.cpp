#include "timingOpt.h"
#include <Core/util.h>

TimingProblem::TimingProblem(const arr& _waypoints, const arr& _tangents,
                             const arr& _x0, const arr& _v0,
                             double _timeCost, double _ctrlCost,
                             bool _optTau, bool _optLastVel,
                             const arr& v_init, const arr& tau_init,
                             double _maxVel, double _maxAcc, double _maxJer,
                             const uintA& _wayFree, bool _accCont, double _timeCost2)
  : waypoints(_waypoints),
    x0(_x0),
    v0(_v0),
    timeCost(_timeCost),
    timeCost2(_timeCost2),
    ctrlCost(_ctrlCost),
    optTau(_optTau),
    optLastVel(_optLastVel),
    accCont(_accCont),
    wayFree(_wayFree),
    v(v_init),
    tau(tau_init){

  CHECK_EQ(waypoints.nd, 2, "");
  uint K = waypoints.d0;
  uint d = waypoints.d1;

  uint vN=K; //number of vels to be optimized
  if(!optLastVel) vN -=1;

  if(!v.N) v.resize(vN, d).setZero();

  if(!tau.N) tau.resize(K) = 1.;
  CHECK_EQ(tau.N, K, "");

  if(_maxVel>0.) maxVel=consts(_maxVel, d);
  if(_maxAcc>0.) maxAcc=consts(_maxAcc, d);
  if(_maxJer>0.) maxJer=consts(_maxJer, d);

  //== define NLP signature

  //-- init dim and bounds
  dimension = 0;
  if(optTau) dimension += tau.N;
  dimension += v.N;
  if(wayFree.N) dimension += wayFree.N*d;

  bounds_lo.resize(dimension) = 0.;
  bounds_up.resize(dimension) = -1.; //means deactivated
  if(optTau){
    for(uint k=0;k<tau.N;k++){
      bounds_lo(k) = .01;
      bounds_up(k) = 10.;
    }
  }

  //-- init feature types
  uint m=1; //timeCost
  if(timeCost2>0.) m += K;
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
    if(timeCost2>0.){
      featureTypes({m,m}) = OT_sos; //control costs
      m += 1;
    }
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
  //tau decision variables
  if(optTau){
    tau = x({0, K-1}).reshape(tau.N);
    vIdx = K;
  }
  //vel decision variables
  v = x({vIdx, vIdx+v.d0*v.d1-1}).reshape(v.d0, v.d1);
  //free waypoint decision variables
  if(wayFree.N){
    arr xway = x({vIdx+v.d0*v.d1,-1});
    xway.reshape(wayFree.N, d);
    for(uint i=0;i<wayFree.N;i++){
      waypoints[wayFree(i)] = xway[i];
    }
  }

  //-- initialize phi and sparse Jacobian
  phi.resize(featureTypes.N).setZero();
  if(!!J) J.sparse().resize(phi.N, dimension, 0);
  uint m=0;

  // 0) total time cost
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

    // 1) sqr time costs (sum of sqr-acc)
    if(timeCost2>0.){
      phi.setVectorBlock(timeCost2*arr{tau(k)}, m);
      if(!!J) J.sparse().add(timeCost2*tauJ, m, 0);
      m += 1;
    }

    // 2) control costs (sum of sqr-acc)
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
//      y *= 1.;
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

  {
    //to animate optimization
//    report(cout, 3, 0);
//    rai::wait(.1);
  }
}

arr TimingProblem::getInitializationSample(const arr& previousOptima){
  arr x;
  if(optTau) x = (tau, v);
  else x = v;
  if(wayFree.N) for(uint i=0;i<wayFree.N;i++){
    x.append(waypoints[wayFree(i)]);
  }
  //    rndGauss(x, .1, true);
  return x.reshape(-1);
}

void TimingProblem::report(std::ostream& fil, int verbose, const char* msg){

  arr path = waypoints;       path.prepend(x0);
  arr vels = v;               vels.prepend(v0);  vels.append(zeros(vels.d1));
  arr times = integral(tau);  times.prepend(0.);

  if(verbose>0){
    LOG(0) <<"TAUS: " <<tau <<"\nTIMES: " <<times <<"\nTOTAL: " <<times.last() <<endl;
    {
      //write
      fil <<", " <<times(-1);
      if(verbose>1){
        fil <<"waypointTimes:" <<times <<endl;
        fil <<"waypoints:" <<path <<endl;
        fil <<"waypointVels:" <<vels <<endl;
      }
      //      arr T;
      //      for(double t=0;t<=times(-1);t+=.002) T.append(t);
      //      fil <<"fine500HzPath: " <<S.eval(T) <<endl;
    }
  }

  if(verbose>2){
    rai::CubicSpline S;
    S.set(path, vels, times);

    arr timeGrid = range(S.times.first(), S.times.last(), 100);
    arr x = S.eval(timeGrid);
    arr xd = S.eval(timeGrid, 1);
    arr xdd = S.eval(timeGrid, 2);
    arr xddd = S.eval(timeGrid, 3);

    if(maxVel.N) for(uint i=0;i<xd.d0;i++) xd[i] /= maxVel;
    if(maxAcc.N) for(uint i=0;i<xdd.d0;i++) xdd[i] /= maxAcc;
    if(maxJer.N) for(uint i=0;i<xddd.d0;i++) xddd[i] /= maxJer;

    if(verbose>2){
      if(x.d1>1){
        arr vM = max(xd,1);
        arr aM = max(xdd,1);
        arr jM = max(xddd,1);
        arr vm = min(xd,1);
        arr am = min(xdd,1);
        arr jm = min(xddd,1);
        rai::catCol({timeGrid, vM, vm, aM, am, jM, jm}).modRaw().write( FILE("z.dat") );
        gnuplot("plot [:][-1.1:1.1] 'z.dat' us 1:2 t 'vmax' ls 1, '' us 1:3 t 'vmin' ls 1, '' us 1:4 t 'amax' ls 2, '' us 1:5 t 'amin' ls 2, '' us 1:6 t 'jmax' ls 3, '' us 1:7 t 'jmin' ls 3");
      }else{
        rai::catCol({timeGrid, x, xd, xdd, xddd}).reshape(-1,5).modRaw(). write( FILE("z.dat") );
        gnuplot("plot [:][-1.1:1.1] 'z.dat' us 1:2 t 'x', ''us 1:3 t 'v', '' us 1:4 t 'a', '' us 1:5 t 'j'");
      }
    }
  }
}

void TimingProblem::smartInitVels(){
  for(uint k=0;k<v.d0;k++){
    if(!k) v[0] = (waypoints[1]-x0)/(tau(0)+tau(1));
    else v[k] = (waypoints[k+1]-waypoints[k-1])/(tau(k)+tau(k+1));
  }
}

void TimingProblem::getVels(arr& vel){
  vel = v;
  if(!optLastVel) vel.append(zeros(waypoints.d1));
  vel.reshape(waypoints.d0, waypoints.d1);
}

arr TimingProblem::xJ(int k){
  if(k==-1) return x0;

  uint K = waypoints.d0;
  uint d = waypoints.d1;
  uint xIdx = (optTau?K:0) + v.d0*d;
  arr xk = waypoints[k].copy();

  for(uint j=0;j<wayFree.N;j++){
    if((int)wayFree(j)==k){
      rai::SparseMatrix& J = xk.J().sparse().resize(d, dimension, d);
      for(uint i=0;i<d;i++) J.entry(i, xIdx+j*d+i, i) = 1.;
      break;
    }
  }

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
