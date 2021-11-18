#include <KOMO/komo.h>
#include <Optim/MP_Solver.h>
#include <KOMO/pathTools.h>

#include <thread>

//===========================================================================

arr getPath(){
  rai::Configuration C("arm.g");
  cout <<"configuration space dim=" <<C.getJointStateDimension() <<endl;
  
  KOMO komo;
  komo.setModel(C);
  komo.setTiming(1., 100, 5., 2);
  komo.add_qControlObjective({}, 2, 1.);
  komo.addQuaternionNorms({}, 1., false);

  //-- set a time optim objective
//  komo.addObjective({}, make_shared<TM_Time>(), OT_sos, {1e2}, {}, 1); //smooth time evolution
//  komo.addObjective({}, make_shared<TM_Time>(), OT_sos, {1e1}, {komo.tau}, 0); //prior on timing

  komo.addObjective({1.}, FS_positionDiff, {"endeff", "target"}, OT_eq, {1e2});
  komo.addObjective({1.}, FS_qItself, {}, OT_eq, {1e2}, {}, 1);
  komo.addObjective({}, FS_accumulatedCollisions, {}, OT_eq, {1e0});

  komo.reportProblem();

  komo.optimize();

  komo.view(false, "result");
//  while(komo.view_play(true));

  return komo.getPath_qOrg();
}

//===========================================================================

struct TimeOpt : MathematicalProgram {
  const arr path;
  const uint prefix;
  const uint n;
  const uint d;
  const double maxVel;
  const double maxAcc;
  const double maxJer;
  const double smooth;
  bool animate=true;
  arr delta;
  arr tau;

  TimeOpt(const arr& _path, uint _prefix, double _maxVel, double _maxAcc, double _maxJer, double _smooth)
    : path(_path),
      prefix(_prefix),
      n(path.d0-_prefix),
      d(path.d1),
      maxVel(_maxVel),
      maxAcc(_maxAcc),
      maxJer(_maxJer),
      smooth(_smooth) {

    //-- delta path is differences of 'path'
    CHECK_EQ(path.nd, 2, "");
    delta.resizeAs(path).setZero();
    for(uint t=1;t<path.d0;t++) delta[t] = path[t]-path[t-1];

    //init dim and bounds
    dimension=n;
    bounds_lo.resize(n) = 1e-3;
    bounds_up.resize(n) = 1.;

    //-- init feature types
    uint dimPhi = 1;
    if(smooth>0) dimPhi+=n-2;
    if(maxVel>0) dimPhi+=2*n*d;
    if(maxAcc>0) dimPhi+=2*n*d;
    if(maxJer>0) dimPhi+=2*n*d;
    featureTypes.resize(dimPhi);

    //all objectives are inequalities...
    featureTypes = OT_ineq;

    //..except for the total time cost
    featureTypes(0) = OT_f;

    //..and the smoothness prior
    if(smooth>0) featureTypes({1,n}) = OT_sos;
  }

  //-- essential methods that need overload
  virtual void evaluate(arr& phi, arr& J, const arr& x){
    tau = x;
    CHECK_EQ(tau.N, n, "");

    phi.resize(featureTypes.N).setZero();
    if(!!J) J.sparse().resize(phi.N, n, 0);
    uint m=0;

    //total time cost
    double a = 1e1;
    phi(0) = a*sum(tau);
    if(!!J){
      for(uint i=0;i<n;i++) J.elem(0,i) += a;
    }
    m += 1;

    //smoothness prior
    if(smooth>0){
      for(uint t=0;t<n-2;t++){
//        phi(m) = smooth*(tau(t+3) - 3.*tau(t+2) + 3.*tau(t+1) - tau(t));
        phi(m) = smooth*(tau(t+2) - 2.*tau(t+1) + tau(t));
        if(!!J){
          J.elem(m, t) += smooth;
          J.elem(m, t+1) -= 2.*smooth;
          J.elem(m, t+2) += smooth;
//          J(m, t+3) += smooth;
        }
        m++;
      }
    }

    //velocity inequalities
    if(maxVel>0) for(uint s=0;s<2;s++){
      double sign = (s?1.:-1.);
      for(uint t=0;t<n;t++) for(uint i=0;i<d;i++){
        phi(m) = sign*delta(prefix+t, i);
        phi(m) -= tau(t) * maxVel;
        if(!!J) J.elem(m, t) = -maxVel;
        m++;
      }
    }

    //acceleration inequalities
    if(maxAcc>0) for(uint s=0;s<2;s++){
      double sign = (s?1.:-1.);
      for(uint t=0;t<n;t++) for(uint i=0;i<d;i++){
        double tau0 = (t>=0 ? tau(t) : 1e-2);
        double tau1 = (t>0 ? tau(t-1) : 1e-2);
        phi(m) = sign*(delta(prefix+t, i)/tau0 - delta(prefix+t-1, i)/tau1);
        phi(m) -= 0.5*(tau0+tau1)*maxAcc;
        if(!!J){
          J.elem(m, t) -= sign*delta(prefix+t, i)/rai::sqr(tau0);
          if(t>0) J.elem(m, t-1) += sign*delta(prefix+t-1, i)/rai::sqr(tau1);
          J.elem(m, t) -= 0.5*maxAcc;
          if(t>0) J.elem(m, t-1) -= 0.5*maxAcc;
        }
        m++;
      }
    }

    //jerk inequalities
    if(maxJer>0) for(uint s=0;s<2;s++){
      double sign = (s?1.:-1.);
      for(uint t=0;t<n;t++) for(uint i=0;i<d;i++){
        double tau0 = (t>=0 ? tau(t) : 1e-2);
        double tau1 = (t>0 ? tau(t-1) : 1e-2);
        double tau2 = (t>1 ? tau(t-2) : 1e-2);
        phi(m) = sign*(delta(prefix+t, i)/tau0 - delta(prefix+t-1, i)/tau1)/(tau0+tau1);
        phi(m) -= sign*(delta(prefix+t-1, i)/tau1 - delta(prefix+t-2, i)/tau2)/(tau1+tau2);
        phi(m) -= 0.5*tau1*maxJer;
        if(!!J){
          J.elem(m, t) -= sign*delta(prefix+t, i)/rai::sqr(tau0*(tau0+tau1))*(2.*tau0+tau1);
          if(t>0) J.elem(m, t-1) -= sign*delta(prefix+t, i)/rai::sqr(tau0*(tau0+tau1))*tau0;
          J.elem(m, t) += sign*delta(prefix+t-1, i)/rai::sqr(tau1*(tau0+tau1))*tau1;
          if(t>0) J.elem(m, t-1) += sign*delta(prefix+t-1, i)/rai::sqr(tau1*(tau0+tau1))*(2.*tau1+tau0);

          if(t>0) J.elem(m, t-1) += sign*delta(prefix+t-1, i)/rai::sqr(tau1*(tau1+tau2))*(2.*tau1+tau2);
          if(t>1) J.elem(m, t-2) += sign*delta(prefix+t-1, i)/rai::sqr(tau1*(tau1+tau2))*tau1;
          if(t>0) J.elem(m, t-1) -= sign*delta(prefix+t-2, i)/rai::sqr(tau2*(tau1+tau2))*tau2;
          if(t>1) J.elem(m, t-2) -= sign*delta(prefix+t-2, i)/rai::sqr(tau2*(tau1+tau2))*(2.*tau2+tau1);

          if(t>0) J.elem(m, t-1) -= 0.5*maxJer;
        }
        m++;
      }
    }

    CHECK_EQ(m, phi.N, "");

    if(animate){
      arr time = integral(tau);
      arr path = integral(delta);
      path.remove(0, prefix);

      catCol(~~time,~~path).writeRaw( FILE("z.dat") );
      gnuplot("plot [0:5] 'z.dat' us 1:2 w lp");
    }

  }

  virtual arr  getInitializationSample(const arr& previousOptima= {}){
    arr tau(dimension);
    tau = 1e-0;
    return tau;
  }

  void plotSolution(){
    arr ttau = (ones(prefix), tau);
    arr time(path.d0);
    time.setZero();
    for(uint t=prefix;t<time.N;t++) time(t) = time(t-1) + ttau(t);
    arr v = max(getVel(path,ttau),1) / maxVel;
    arr a = max(getAcc(path,ttau),1) / maxAcc;
    arr j = max(getJerk(path,ttau),1) / maxJer;
    arr vi = min(getVel(path,ttau),1) / maxVel;
    arr ai = min(getAcc(path,ttau),1) / maxAcc;
    arr ji = min(getJerk(path,ttau),1) / maxJer;
    catCol(LIST(~~time, ~~v, ~~a, ~~j, ~~vi, ~~ai, ~~ji)).reshape(-1,7).writeRaw( FILE("z.dat") );
    gnuplot("plot [:][-1.1:1.1] 'z.dat' us 1:2 t 'v', '' us 1:3 t 'a', '' us 1:4 t 'j', '' us 1:5 t 'vmin', '' us 1:6 t 'amin', '' us 1:7 t 'jmin'");
  }
};

//===========================================================================

void timeOpt(){
  arr path = getPath();
  path = path.cols(1,7);
  uint k=2;
  arr x0=path[0];
  for(uint i=0;i<k;i++){ path.prepend(x0); }

  TimeOpt mp(path, k, 1., 4., 30., 1e1);
  mp.animate=false;
//  checkJacobianCP(mp, tau.sub(k,-1), 1e-6);
//  rai::wait();

  rai::Enum<MP_SolverID> sid (rai::getParameter<rai::String>("solver"));
  MP_Solver solver;
  solver
      .setProblem(mp.ptr())
      .setSolver(sid)
//      .setTracing(true, true,true, false)
      .setVerbose(rai::getParameter<int>("opt/verbose"));
  auto ret = solver.solve();
  cout <<*ret <<endl;

  checkJacobianCP(mp, ret->x, 1e-6);
  arr tau(path.d0);
  tau=.01;
  tau({k,-1}) = mp.tau;

  cout <<tau <<endl;

  mp.plotSolution();
  rai::wait();

  MP_Viewer(mp.ptr(), solver.P).plotCostTrace();
  rai::wait();
//  cout <<solver.getTrace_x() <<solver.getTrace_costs() <<endl;

}

//===========================================================================

int main(int argc,char** argv){
  rai::initCmdLine(argc,argv);

//  rnd.clockSeed();

  timeOpt();

  return 0;
}

