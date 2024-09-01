#include <Control/timingOpt.h>

#include <Optim/NLP_Solver.h>
#include <Control/TimingMPC.h>
#include <Kin/frame.h>
#include <Kin/kin.h>
#include <Kin/viewer.h>
#include <Gui/opengl.h>

//===========================================================================

void plotSplineTiming(const rai::CubicSpline& S, const arr& maxVel={}, const arr& maxAcc={}, const arr& maxJer={}){
  arr time = range(S.times.first(), S.times.last(), 100);
  arr x = S.eval(time);
  arr xd = S.eval(time, 1);
  arr xdd = S.eval(time, 2);
  arr xddd = S.eval(time, 3);

  if(maxVel.N) for(uint i=0;i<xd.d0;i++) xd[i] /= maxVel;
  if(maxAcc.N) for(uint i=0;i<xdd.d0;i++) xdd[i] /= maxAcc;
  if(maxJer.N) for(uint i=0;i<xddd.d0;i++) xddd[i] /= maxJer;

  if(x.d1>1){
    arr vM = max(xd,1);
    arr aM = max(xdd,1);
    arr jM = max(xddd,1);
    arr vm = min(xd,1);
    arr am = min(xdd,1);
    arr jm = min(xddd,1);
    rai::catCol({time, vM, vm, aM, am, jM, jm}).modRaw().write( FILE("z.dat") );
    gnuplot("plot [:][-1.1:1.1] 'z.dat' us 1:2 t 'vmax' ls 1, '' us 1:3 t 'vmin' ls 1, '' us 1:4 t 'amax' ls 2, '' us 1:5 t 'amin' ls 2, '' us 1:6 t 'jmax' ls 3, '' us 1:7 t 'jmin' ls 3");
  }else{
    rai::catCol({time, x, xd, xdd, xddd}).reshape(-1,5).modRaw(). write( FILE("z.dat") );
    gnuplot("plot [:][-1.1:1.1] 'z.dat' us 1:2 t 'x', ''us 1:3 t 'v', '' us 1:4 t 'a', '' us 1:5 t 'j'");
  }
}

//===========================================================================

void refineWaypoints(const arr& x0, arr& waypoints, uintA& wayFree){
  uint K= waypoints.d0;
  waypoints.resizeCopy(K*2, waypoints.d1);
  for(uint k=K;k--;){
    waypoints[2*k+1] = waypoints[k];
    if(!k) waypoints[0] = .5*(x0+waypoints[0]);
    else waypoints[2*k] = .5*(waypoints[k-1]+waypoints[k]);
  }
  wayFree.resize(K);
  for(uint k=0;k<K;k++){ wayFree(k) = 2*k; }
}

//===========================================================================

void timeOpt(){
#if 0
  arr path(10,1);
  for(uint t=0;t<path.d0;t++){
    path.elem(t) = pow(0.1*(path.d0-t-1),1);
  }
#elif 0
  arr path = rand({5,3});
  cout <<path <<endl;
#elif 1
  arr path = rai::getParameter<arr>("path");
  path.reshape(-1,8);

#else
  arr path = {{5,1}, {0,.5,1.,.5,.0}};
#endif

  arr v0 = zeros(path.d1);
  arr x0 = path[0].copy();
  path.delRows(0,1);
#if 1
  path.prepend(x0);
//  path.prepend(x0);
//  path.append(path[-1]);
  path.append(path[-1]);
  uintA wayFree = {0,path.d0-2/*,1,path.d0-3*/};
//  uintA wayFree;
//  refineWaypoints(x0, path, wayFree);
//  wayFree.append(1);
//  wayFree.append(path.d0-3);
//  cout <<path.d0 <<wayFree <<endl;
#endif

//  x0 = 0.;
//  v0 = 0.;
  arr tau(path.d0);
  tau = .2;
  rndGauss(tau,.01, true);

  TimingProblem mp(path, {}, x0, v0, //waypoint, tangents, start
                   1e2, 1e1, //time ctrlCost
                   true, false, //optTau, optLastVel
                   {}, tau, //init
                   1., 1., 1.,  //max v a j
                   wayFree, true, 1e3); //refine
//  mp.tauBarrier=true;
  if(path.d1==8){
    mp.maxVel = rai::getParameter<arr>("max_vel");
    mp.maxAcc = rai::getParameter<arr>("max_acc");
    mp.maxJer = rai::getParameter<arr>("max_jrk");
  }
  mp.smartInitVels();

  rai::Enum<NLP_SolverID> sid (rai::getParameter<rai::String>("solver"));

  for(uint k=0;k<1;k++){
  {
    NLP_Solver solver;
    solver
        .setProblem(mp.ptr())
        .setSolver(sid);
    auto ret = solver.solve();
    mp.checkJacobian(ret->x, 1e-4);
    cout <<*ret <<endl;
  }

  mp.report(cout, 2, 0);
  LOG(0) <<"old ctrlCost:" <<mp.ctrlCost <<endl;
  rai::wait();
  mp.ctrlCost*=.5;
  }

  mp.report(cout, 3, 0);
  rai::wait();
}

//===========================================================================

void waypointHunting(){
  //-- create random waypoints
  uint K=5, d=3;
  arr waypoints(K,d);
  rndUniform(waypoints, -1., 1.);
  for(uint k=0;k<K;k++) waypoints(k,2) += 1.5;

  TimingMPC F(waypoints, 1e2, 1e-1);

  rai::Configuration C;
  for(uint k=0;k<K;k++){
    rai::Frame *f = C.addFrame(STRING("flag_"<<k));
    f->setShape(rai::ST_sphere, {.1});
    f->setPosition(F.waypoints[k]);
  }

  rai::Frame *bot = C.addFrame(STRING("ego"));
  bot->setShape(rai::ST_sphere, {.1}).setColor({.8, .6, .6});

  //-----------------------

  F.solve(zeros(3), zeros(3));

  //-----------------------

  rai::CubicSpline S;
  F.getCubicSpline(S, zeros(3), zeros(3));

  rai::Frame *spline = C.addFrame("spline");
  spline->setLines( S.eval(range(0., S.times.last(), 100)) );

//  C.gl().add(M);
  C.view(true);

  double del=.001, t=0.;
  uint FLAG = 0;
  for(uint step=0;;step++, t+=del){
    arr x0 = S.eval(t);
    arr v0 = S.eval(t,1);
    bot->setPosition(x0);
    if(maxDiff(x0,F.waypoints[F.phase]) < 1e-2){
      C.frames(FLAG)->setColor({.6,.8,.6});
      FLAG++;
      if(FLAG==K) break;
#if 1
      F.phase ++;
#else
      F.waypoints.delRows(0, 1);
      if(F.tangents.N) F.tangents.delRows(0, 1);
      if(F.vels.nd==1) F.vels.remove(0, 1);
      else F.vels.delRows(0, 1);
      F.tau.remove(0, 1);
#endif
    }

    if(!(step%10)) C.view();
    rai::wait(5.*del);

    //-- perturb waypoints
    if(rnd.uni()<10.*del){
      uint m=rnd(F.waypoints.d0);
      m=F.phase;
      F.waypoints[m] += .2*randn(F.waypoints.d1);
      C.frames(m)->setPosition(F.waypoints[m]);
    }

    //-- update:
    if(t>.1){
      F.solve(x0, v0);
      F.getCubicSpline(S, x0, v0);
      spline->setLines( S.eval(range(0., S.times.last(), 100)) );
      t = .0;
    }

  }
  C.view(true);

}

//===========================================================================

int main(int argc,char** argv){
  rai::initCmdLine(argc,argv);

//  rnd.seed(1);
  rnd.clockSeed();

  timeOpt();

  waypointHunting();

  return 0;
}

