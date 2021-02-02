#include <Kin/kin.h>
#include <Kin/kin_swift.h>
#include <Kin/kin_ode.h>
#include <Algo/spline.h>
#include <Algo/algos.h>
#include <Gui/opengl.h>
#include <Plot/plot.h>
#include <GL/gl.h>
#include <Optim/optimization.h>

//===========================================================================
//
// dynamics test
//

//namespace T2{
//  bool friction;
//  arr tau;
//  //static arr conswit;
//  //bool hasContact=false;
//  bool addContactsToDynamics=false;
//  rai::Configuration *G;
//}

//---------- test standard dynamic control
void TEST(Dynamics){
  rai::Configuration C("arm7.g");
//  rai::Configuration C("mypr2.g");
  C.optimizeTree(true);
  C.sortFrames();
  cout <<C <<endl;

  arr u;
  bool friction=false;
  VectorFunction diffEqn = [&C,&u,&friction](arr& y,arr&,const arr& x){
    C.setJointState(x[0]);
    if(!u.N) u.resize(x.d1).setZero();
    if(friction) u = -10. * x[1];
    /*if(T2::addContactsToDynamics){
        G.contactsToForces(100.,10.);
      }*/
    C.fwdDynamics(y, x[1], u);
  };
  
  uint t,T=720,n=C.getJointStateDimension();
  arr q,qd(n),qdd(n),qdd_(n);
  q = C.getJointState();
  qd.setZero();
  qdd.setZero();
  
  double dt=.01;

  ofstream z("z.dyn");
  rai::String text;
  C.watch();
//  for(rai::Body *b:G.bodies){ b->mass=1.; b->inertia.setZero(); }

  for(t=0;t<T;t++){
    if(false && t>=500){ //hold steady **THIS BREAKS! INV DYNAMICS ARE BROKE **
      qdd_ = -1. * qd;
      C.inverseDynamics(u, qd, qdd_);
      //tau.resize(n); tau.setZero();
      //G.clearForces();
      //G.gravityToForces();
      C.fwdDynamics(qdd, qd, u);
      CHECK_LE(maxDiff(qdd,qdd_,0), 1e-5, "dynamics and inverse dynamics inconsistent");
      //cout <<q <<qd <<qdd <<endl;
      cout <<"test dynamics: fwd-inv error =" <<maxDiff(qdd,qdd_,0) <<endl;
      q  += .5*dt*qd;
      qd +=    dt*qdd;
      q  += .5*dt*qd;
      C.setJointState(q);
      //cout <<q <<qd <<qdd <<endl;
      text.clear() <<"t=" <<t <<"  torque controlled damping (acc = - vel)\n(checking consistency of forward and inverse dynamics),  energy=" <<C.getEnergy(qd);
    }else{
      //cout <<q <<qd <<qdd <<' ' <<G.getEnergy() <<endl;
      arr x=cat(q,qd).reshape(2,q.N);
      rai::rk4_2ndOrder(x, x, diffEqn, dt);
      q=x[0]; qd=x[1];
      if(t>300){
        friction=true;
        text.clear() <<"t=" <<t <<"  friction swing using RK4,  energy=" <<C.getEnergy(qd);
      }else{
        friction=false;
        text.clear() <<"t=" <<t <<"  free swing using RK4,  energy=" <<C.getEnergy(qd);
      }
    }
    C.watch(false, text);
  }
}

// =============================================================================

int MAIN(int argc,char **argv){
  rai::initCmdLine(argc, argv);

  testDynamics();

  return 0;
}
