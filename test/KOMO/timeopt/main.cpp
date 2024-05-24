#include <KOMO/komo.h>
#include <Optim/NLP_Solver.h>
#include <KOMO/pathTools.h>

#include <thread>

//===========================================================================

void createPath(){
  rai::Configuration C("arm.g");
  cout <<"configuration space dim=" <<C.getJointStateDimension() <<endl;
  
  KOMO komo;
  komo.setConfig(C);
  komo.setTiming(1., 100, 1., 2);
  komo.addControlObjective({}, 2, 1.);
  komo.addQuaternionNorms({}, 1., false);

  //-- set a time optim objective
//  komo.addObjective({}, make_shared<TM_Time>(), OT_sos, {1e2}, {}, 1); //smooth time evolution
//  komo.addObjective({}, make_shared<TM_Time>(), OT_sos, {1e1}, {komo.tau}, 0); //prior on timing

  komo.addObjective({1.}, FS_positionDiff, {"endeff", "target"}, OT_eq, {1e2});
  komo.addObjective({1.}, FS_qItself, {}, OT_eq, {1e2}, {}, 1);
  komo.addObjective({}, FS_accumulatedCollisions, {}, OT_eq, {1e0});

  cout <<komo.report(true, false) <<endl;

  auto ret = NLP_Solver()
      .setProblem(komo.nlp())
      .solve();
  cout <<*ret <<endl;
//  komo.optimize();

  komo.view(false, "result");
  while(komo.view_play(true));

  FILE("z.path") <<komo.getPath_qOrg();
}

//===========================================================================

struct TimeOpt : NLP {
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
  double prefixTau = 1e-1;
  uint jacSize=0;

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
    bounds.resize(2, n);
    bounds[0] = 1e-3;
    bounds[1] = 1.;

    //-- init feature types
    uint dimPhi = 1;
    jacSize += n;
    if(smooth>0){ dimPhi+=n-2; jacSize+=(n-2)*3; }
    if(maxVel>0){ dimPhi+=2*n*d; jacSize+=2*n*d; }
    if(maxAcc>0){ dimPhi+=2*n*d; jacSize+=2*n*d*4; }
    if(maxJer>0){ dimPhi+=2*n*d; jacSize+=2*n*d*9; }
    featureTypes.resize(dimPhi);

    //all objectives are inequalities...
    featureTypes = OT_ineqB;

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
    rai::SparseMatrix* JS=0;
    if(!!J){
      JS = &J.sparse();
      JS->resize(phi.N, n, jacSize);
    }
    uint m=0;
    uint z=0;

    //total time cost
    double a = 1e1;
    phi(0) = a*sum(tau);
    if(!!J){
      for(uint i=0;i<n;i++) JS->entry(0, i, z++) += a;
    }
    m += 1;

    //smoothness prior
    if(smooth>0){
      for(uint t=0;t<n-2;t++){
//        phi(m) = smooth*(tau(t+3) - 3.*tau(t+2) + 3.*tau(t+1) - tau(t));
        phi(m) = smooth*(tau(t+2) - 2.*tau(t+1) + tau(t));
        if(!!J){
          JS->entry(m, t, z++) += smooth;
          JS->entry(m, t+1, z++) -= 2.*smooth;
          JS->entry(m, t+2, z++) += smooth;
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
        if(!!J) JS->entry(m, t, z++) = -maxVel;
        m++;
      }
    }

    //acceleration inequalities
    if(maxAcc>0) for(uint s=0;s<2;s++){
      double sign = (s?1.:-1.);
      for(uint t=0;t<n;t++) for(uint i=0;i<d;i++){
        double tau0 = (t>=0 ? tau(t) : prefixTau);
        double tau1 = (t>0 ? tau(t-1) : prefixTau);
        phi(m) = sign*(delta(prefix+t, i)/tau0 - delta(prefix+t-1, i)/tau1);
        phi(m) -= 0.5*(tau0+tau1)*maxAcc;
        if(!!J){
          JS->entry(m, t, z++) -= sign*delta(prefix+t, i)/rai::sqr(tau0);
          if(t>0) JS->entry(m, t-1, z++) += sign*delta(prefix+t-1, i)/rai::sqr(tau1);
          JS->entry(m, t, z++) -= 0.5*maxAcc;
          if(t>0) JS->entry(m, t-1, z++) -= 0.5*maxAcc;
        }
        m++;
      }
    }

    //jerk inequalities
    if(maxJer>0) for(uint s=0;s<2;s++){
      double sign = (s?1.:-1.);
      for(uint t=0;t<n;t++) for(uint i=0;i<d;i++){
        double tau0 = (t>=0 ? tau(t) : prefixTau);
        double tau1 = (t>0 ? tau(t-1) : prefixTau);
        double tau2 = (t>1 ? tau(t-2) : prefixTau);
        phi(m) = sign*(delta(prefix+t, i)/tau0 - delta(prefix+t-1, i)/tau1)/(tau0+tau1);
        phi(m) -= sign*(delta(prefix+t-1, i)/tau1 - delta(prefix+t-2, i)/tau2)/(tau1+tau2);
        phi(m) -= 0.5*tau1*maxJer;
        if(!!J){
          JS->entry(m, t, z++) -= sign*delta(prefix+t, i)/rai::sqr(tau0*(tau0+tau1))*(2.*tau0+tau1);
          if(t>0) JS->entry(m, t-1, z++) -= sign*delta(prefix+t, i)/rai::sqr(tau0*(tau0+tau1))*tau0;
          JS->entry(m, t, z++) += sign*delta(prefix+t-1, i)/rai::sqr(tau1*(tau0+tau1))*tau1;
          if(t>0) JS->entry(m, t-1, z++) += sign*delta(prefix+t-1, i)/rai::sqr(tau1*(tau0+tau1))*(2.*tau1+tau0);

          if(t>0) JS->entry(m, t-1, z++) += sign*delta(prefix+t-1, i)/rai::sqr(tau1*(tau1+tau2))*(2.*tau1+tau2);
          if(t>1) JS->entry(m, t-2, z++) += sign*delta(prefix+t-1, i)/rai::sqr(tau1*(tau1+tau2))*tau1;
          if(t>0) JS->entry(m, t-1, z++) -= sign*delta(prefix+t-2, i)/rai::sqr(tau2*(tau1+tau2))*tau2;
          if(t>1) JS->entry(m, t-2, z++) -= sign*delta(prefix+t-2, i)/rai::sqr(tau2*(tau1+tau2))*(2.*tau2+tau1);

          if(t>0) JS->entry(m, t-1, z++) -= 0.5*maxJer;
        }
        m++;
      }
    }

    CHECK_EQ(m, phi.N, "");

    if(animate){
      arr time = integral(tau);
      arr path = integral(delta);
      path.remove(0, prefix);

      FILE("z.dat") <<catCol(~~time,~~path).modRaw();
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
    FILE("z.dat") <<rai::catCol(rai::Array<rai::Array<double>*>{&time, &v, &a, &j, &vi, &ai, &ji}).reshape(-1,7).modRaw();
    gnuplot("plot [:][-1.1:1.1] 'z.dat' us 1:2 t 'v', '' us 1:3 t 'a', '' us 1:4 t 'j', '' us 1:5 t 'vmin', '' us 1:6 t 'amin', '' us 1:7 t 'jmin'");
  }
};

//===========================================================================

void timeOpt(){
  arr path;
  FILE("z.path") >>path;
//  path = path.cols(0,7);
  uint k=2;
  arr x0=path[0];
  for(uint i=0;i<k;i++){ path.prepend(x0); }

  TimeOpt nlp(path, k, 2., 4., 30., 1e1);
  nlp.animate=false;
//  checkJacobianCP(nlp, tau.sub(k,-1), 1e-6);
//  rai::wait();

  rai::setParameter<double>("opt/maxStep", 1e-1);
  rai::setParameter<double>("opt/stopTolerance", 1e-6);
  rai::setParameter<double>("opt/damping", 1e-2);

  rai::Enum<NLP_SolverID> sid (rai::getParameter<rai::String>("solver"));
  NLP_Solver solver;
  solver
      .setProblem(nlp.ptr())
//      .setTracing(true, true,true, false)
      .setSolver(sid);
  auto ret = solver.solve();
  cout <<*ret <<endl;

//  checkJacobianCP(nlp, ret->x, 1e-6);
  arr tau(path.d0);
  tau=.01;
  tau({k,-1}) = nlp.tau;

  cout <<tau <<endl;

  nlp.plotSolution();
  rai::wait();

  NLP_Viewer(nlp.ptr(), solver.P).plotCostTrace();
  rai::wait();
//  cout <<solver.getTrace_x() <<solver.getTrace_costs() <<endl;

}

//===========================================================================

int main(int argc,char** argv){
  rai::initCmdLine(argc,argv);

//  rnd.clockSeed();

  createPath();
  timeOpt();

  return 0;
}

