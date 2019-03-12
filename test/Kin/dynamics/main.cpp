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
//  rai::KinematicWorld *G;
//}

//---------- test standard dynamic control
void TEST(Dynamics){
//  rai::KinematicWorld G("arm7.g");
  rai::KinematicWorld G("mypr2.g");
  G.optimizeTree(true);
  G.sortFrames();
  cout <<G <<endl;

  arr u;
  bool friction=false;
  VectorFunction diffEqn = [&G,&u,&friction](arr& y,arr&,const arr& x){
    G.setJointState(x[0], x[1]);
    if(!u.N) u.resize(x.d1).setZero();
    if(friction) u = -10. * x[1];
    G.clearForces();
    G.gravityToForces();
    /*if(T2::addContactsToDynamics){
        G.contactsToForces(100.,10.);
      }*/
    G.fwdDynamics(y, x[1], u);
  };
  
  uint t,T=720,n=G.getJointStateDimension();
  arr q,qd,qdd(n),qdd_(n);
  G.getJointState(q, qd);
  qdd.setZero();
  
  double dt=.01;

  ofstream z("z.dyn");
  G.clearForces();
  G.watch();
//  for(rai::Body *b:G.bodies){ b->mass=1.; b->inertia.setZero(); }

  for(t=0;t<T;t++){
    if(t>=500){ //hold steady
      qdd_ = -1. * qd;
      G.inverseDynamics(u, qd, qdd_);
      //tau.resize(n); tau.setZero();
      //G.clearForces();
      //G.gravityToForces();
      G.fwdDynamics(qdd, qd, u);
      CHECK(maxDiff(qdd,qdd_,0)<1e-5,"dynamics and inverse dynamics inconsistent");
      //cout <<q <<qd <<qdd <<endl;
      cout <<"test dynamics: fwd-inv error =" <<maxDiff(qdd,qdd_,0) <<endl;
      q  += .5*dt*qd;
      qd +=    dt*qdd;
      q  += .5*dt*qd;
      G.setJointState(q,qd);
      //cout <<q <<qd <<qdd <<endl;
      G.gl().text.clear() <<"t=" <<t <<"  torque controlled damping (acc = - vel)\n(checking consistency of forward and inverse dynamics),  energy=" <<G.getEnergy();
    }else{
      //cout <<q <<qd <<qdd <<' ' <<G.getEnergy() <<endl;
      arr x=cat(q,qd).reshape(2,q.N);
      rai::rk4_2ndOrder(x, x, diffEqn, dt);
      q=x[0]; qd=x[1];
      if(t>300){
        friction=true;
        G.gl().text.clear() <<"t=" <<t <<"  friction swing using RK4,  energy=" <<G.getEnergy();
      }else{
        friction=false;
        G.gl().text.clear() <<"t=" <<t <<"  free swing using RK4,  energy=" <<G.getEnergy();
      }
    }
    G.watch(false);
  }
}

// =============================================================================

int MAIN(int argc,char **argv){
  rai::initCmdLine(argc, argv);

  testDynamics();

  return 0;
}
