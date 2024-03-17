#include <Kin/kin.h>
#include <Kin/kin_ode.h>
#include <Algo/spline.h>
#include <Algo/rungeKutta.h>
#include <Gui/opengl.h>
#include <Gui/plot.h>
#include <GL/gl.h>

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

  arr u;
  bool friction=false;
  VectorFunction diffEqn = [&C,&u,&friction](const arr& x) -> arr{
    checkNan(x);
    C.setJointState(x[0]);
    if(!u.N) u.resize(x.d1).setZero();
    if(friction) u = -10. * x[1];
    /*if(T2::addContactsToDynamics){
        G.contactsToForces(100.,10.);
      }*/
    arr y;
    C.fwdDynamics(y, x[1], u);
    checkNan(y);
    return y;
  };
  
  uint t,T=500,n=C.getJointStateDimension();
  arr q,qd(n),qdd(n),qdd_(n);
  q = C.getJointState();
  qd.setZero();
  qdd.setZero();
  
  double dt=.01;

  ofstream z("z.dyn");
  rai::String text;
  C.view();
//  for(rai::Body *b:G.bodies){ b->mass=1.; b->inertia.setZero(); }

  for(t=0;t<T;t++){
    if(t>1000){ //hold steady **THIS BREAKS! INV DYNAMICS ARE BROKE **
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
      arr x = (q,qd).reshape(2,q.N);
      rai::rk4_2ndOrder(x, x, diffEqn, dt);
      q=x[0]; qd=x[1];
      if(t>500){
        friction=true;
        text.clear() <<"t=" <<t <<"  friction swing using RK4,  energy=" <<C.getEnergy(qd);
      }else{
        friction=false;
        text.clear() <<"t=" <<t <<"  free swing using RK4,  energy=" <<C.getEnergy(qd);
      }
    }
    C.view(false, text);
  }
}

// =============================================================================

int MAIN(int argc,char **argv){
  rai::initCmdLine(argc, argv);

  testDynamics();

  return 0;
}
