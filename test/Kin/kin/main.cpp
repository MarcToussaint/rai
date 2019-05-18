#include <Kin/kin.h>
#include <Kin/frame.h>
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
// test laod save
//

void TEST(LoadSave){
  rai::KinematicWorld G;
  FILE("arm7.g") >>G;
  cout <<G <<endl;

  for(uint i=0;i<0;i++){
    G.watch(true);
//    rai::wait(.1)
  }
}

//===========================================================================
//
// Jacobian test
//

void testJacobianInFile(const char* filename, const char* shape){
  rai::KinematicWorld K(filename);

  rai::Frame *a=K.getFrameByName(shape);

  VectorFunction f = ( [&a, &K](arr& y, arr& J, const arr& x) -> void
  {
    K.setJointState(x);
    K.kinematicsPos(y, J, a, NoVector);
    if(!!J) cout <<"J=" <<J <<endl;
  } );

  checkJacobian(f, K.q, 1e-4);

//  rai::wait();
}

//===========================================================================
//
// Jacobian test
//

void TEST(Kinematics){

  struct MyFct : VectorFunction{
    enum Mode {Pos, Vec, Quat, RelPos, RelVec, RelRot} mode;
    rai::KinematicWorld& K;
    rai::Frame *b, *b2;
    rai::Vector &vec, &vec2;
    MyFct(Mode _mode, rai::KinematicWorld &_K,
          rai::Frame *_b, rai::Vector &_vec, rai::Frame *_b2, rai::Vector &_vec2)
      : mode(_mode), K(_K), b(_b), b2(_b2), vec(_vec), vec2(_vec2){
      VectorFunction::operator= ( [this](arr& y, arr& J, const arr& x) -> void{
        K.setJointState(x);
        switch(mode){
          case Pos:    K.kinematicsPos(y,J,b,vec); break;
          case Vec:    K.kinematicsVec(y,J,b,vec); break;
          case Quat:   K.kinematicsQuat(y,J,b); break;
          case RelPos: K.kinematicsRelPos(y,J,b,vec,b2,vec2); break;
          case RelVec: K.kinematicsRelVec(y,J,b,vec,b2); break;
          case RelRot: K.kinematicsRelRot(y,J,b,b2); break;
        }
        //if(!!J) cout <<"\nJ=" <<J <<endl;
      } );
    }
    VectorFunction& operator()(){ return *this; }
  };

//  rai::KinematicWorld G("arm7.g");
  rai::KinematicWorld G("kinematicTests.g");
//  rai::KinematicWorld G("../../../data/pr2_model/pr2_model.ors");
//  rai::KinematicWorld G("../../../projects/17-LGP-push/quatJacTest.g");
//  G.watch(true);

  for(uint k=0;k<10;k++){
    rai::Frame *b = G.frames.rndElem();
    rai::Frame *b2 = G.frames.rndElem();
    rai::Vector vec=0, vec2=0;
    vec.setRandom();
    vec2.setRandom();
    arr x(G.getJointStateDimension());
    rndUniform(x,-.5,.5,false);

    cout <<"kinematicsPos:   "; checkJacobian(MyFct(MyFct::Pos   , G, b, vec, b2, vec2)(), x, 1e-5);
    cout <<"kinematicsRelPos:"; checkJacobian(MyFct(MyFct::RelPos, G, b, vec, b2, vec2)(), x, 1e-5);
    cout <<"kinematicsVec:   "; checkJacobian(MyFct(MyFct::Vec   , G, b, vec, b2, vec2)(), x, 1e-5);
    cout <<"kinematicsRelVec:"; checkJacobian(MyFct(MyFct::RelVec, G, b, vec, b2, vec2)(), x, 1e-5);
    cout <<"kinematicsQuat:  "; checkJacobian(MyFct(MyFct::Quat  , G, b, vec, b2, vec2)(), x, 1e-5);
    cout <<"kinematicsRelRot:"; checkJacobian(MyFct(MyFct::RelRot, G, b, vec, b2, vec2)(), x, 1e-5);

    //checkJacobian(Convert(T1::f_hess, NULL), x, 1e-5);
  }
}

//===========================================================================
//
// Graph export test
//

void TEST(Graph){
  if(!rai::FileToken("../../../../rai-robotModels/pr2/pr2.g", false).exists()) return;
  
//  rai::KinematicWorld G("arm7.g");
//  rai::KinematicWorld K("kinematicTests.g");
  rai::KinematicWorld K("../../../../rai-robotModels/pr2/pr2.g");
//  rai::KinematicWorld G("../../../projects/17-LGP-push/quatJacTest.g");
//  G.watch(true);

  K.prefixNames();
  {
    Graph G = K.getGraph();
    G.displayDot();
    rai::wait(.5);
  }

  K.optimizeTree();
  {
    Graph G = K.getGraph();
    G.displayDot();
    rai::wait(.5);
  }

}

//===========================================================================
//
// Jacobian test
//

void TEST(QuaternionKinematics){
  rai::KinematicWorld G("kinematicTestQuat.g");
  G.orsDrawJoints=false;

  for(uint k=0;k<3;k++){
    rai::Quaternion target;
    target.setRandom();
    G.getFrameByName("ref")->Q.rot = target;
    G.getFrameByName("marker")->Q.rot = target;
    rai::Frame *endeff = G.getFrameByName("endeff");
    arr x;
    G.getJointState(x);
    for(uint t=0;t<100;t++){
      arr y,J;
      G.kinematicsQuat(y, J, endeff);  //get the new endeffector position
      arr Jinv = pseudoInverse(J, NoArr, 1e-4); //~J*inverse_SymPosDef(J*~J);
      if(scalarProduct(conv_quat2arr(target),y)<0.) target.flipSign();
      x += 0.05 * Jinv * (conv_quat2arr(target)-y);                  //simulate a time step (only kinematically)
      G.setJointState(x);
      G.watch(false, STRING("test quaternion task spaces -- time " <<t));
    }
  }
}

//===========================================================================
//
// copy operator test
//

void TEST(Copy){
  rai::KinematicWorld G1("kinematicTests.g");
  //rai::KinematicWorld G1("../../../data/pr2_model/pr2_model.ors");
  rai::KinematicWorld G2(G1);

  G1.checkConsistency();
  G2.checkConsistency();

  G1 >>FILE("z.1");
  G2 >>FILE("z.2");

  charA g1,g2;
  g1.readRaw(FILE("z.1"));
  g2.readRaw(FILE("z.2"));

  CHECK_EQ(g1, g2, "copy operator failed!")
  cout <<"** copy operator success" <<endl;
}

//===========================================================================
//
// Kinematic speed test
//

void TEST(KinematicSpeed){
#define NUM 100000
#if 1
//  rai::KinematicWorld K("kinematicTests.g");
  if(!rai::FileToken("../../../../rai-robotModels/pr2/pr2.g", false).exists()) return;
  rai::KinematicWorld K("../../../../rai-robotModels/pr2/pr2.g");
  K.optimizeTree();
  uint n=K.getJointStateDimension();
  arr x(n);
  rai::timerStart();
  for(uint k=0;k<NUM;k++){
    rndUniform(x,-.5,.5,false);
    K.setJointState(x);
//    G.watch();
//    rai::wait(.1);
  }
  cout <<"kinematics timing: "<< rai::timerRead() <<"sec" <<endl;
#endif

#if 0
  rai::Transformation t,s; t.setRandom(); s.setRandom();
  rai::timerStart();
  for(uint k=0;k<NUM;k++){
    t.appendTransformation(s);
  }
  cout <<"transformation appending: "<< rai::timerRead() <<"sec" <<endl;

  rai::Matrix A,B,Y; A.setRandom(); B.setRandom();
  rai::Vector a,b,y; a.setRandom(); b.setRandom();
  rai::timerStart();
  for(uint k=0;k<NUM;k++){
    Y=A*B;
    y=a+A*b;
    a=y;
    A=Y;
  }
  cout <<"matrix timing: "<< rai::timerRead() <<"sec" <<endl;
#endif
}

//===========================================================================
//
// SWIFT and contacts test
//

void TEST(Contacts){
  rai::KinematicWorld G("arm7.g");
  
  arr x,con,grad;
  uint t;

  G.swift().setCutoff(.5);

  VectorFunction f = [&G](arr& y, arr& J, const arr& x) -> void {
    G.setJointState(x);
    G.stepSwift();
    G.kinematicsProxyCost(y, (!!J?J:NoArr), .2);
  };

  x = G.q;
  for(t=0;t<100;t++){
    G.setJointState(x);
    G.stepSwift();

    G.reportProxies();

    G.kinematicsProxyCost(con, grad, .2);
    cout <<"contact meassure = " <<con(0) <<endl;
    //G.watch(true);
    G.watch(false, STRING("t=" <<t <<"  movement along negative contact gradient (using SWIFT to get contacts)"));
    //x += inverse(grad)*(-.1*c);
    x -= 3e-3*grad; //.1 * (invJ * grad);

    checkJacobian(f, x, 1e10);
  }
}

//===========================================================================

void TEST(Limits){
  rai::KinematicWorld G("arm7.g");

  arr limits = G.getLimits();
  VectorFunction F = [&G, &limits](arr& y, arr& J, const arr& x){
    G.setJointState(x);
    G.kinematicsLimitsCost(y,J,limits);
  };

  uint n=G.getJointStateDimension();
  arr x(n),y,J;
  for(uint k=0;k<10;k++){
    rndUniform(x,-2.,2.,false);
    checkJacobian(F,x,1e-4);
    for(uint t=0;t<10;t++){
      F(y,J,x);
      cout <<"y=" <<y <<"  " <<flush;
      x -= .1 * J.reshape(n);
      checkJacobian(F,x,1e-4);
      G.setJointState(x);
      G.watch();
    }
  }
}

//===========================================================================
//
// set state test
//

void generateSequence(arr &X, uint T, uint n){
  rnd.seed(0);
  arr P(10,n);

  //a random spline
  //a set of random via points with zero start and end:
  rndUniform(P,-1.,1.,false); P[0]=0.; P[P.d0-1]=0.;
  
  //convert into a smooth spline (1/0.03 points per via point):
  X = rai::Spline(T,P).eval();
}

void TEST(PlayStateSequence){
  rai::KinematicWorld G("arm7.g");
  uint n=G.getJointStateDimension();
  arr X;
  generateSequence(X, 200, n);
  arr v(X.d1); v=0.;
  for(uint t=0;t<X.d0;t++){
    G.setJointState(X[t](),v);
    G.watch(false, STRING("replay of a state sequence -- time " <<t));
  }
}

//===========================================================================
//
// ODE test
//

#ifdef RAI_ODE
void TEST(PlayTorqueSequenceInOde){
  rai::KinematicWorld G("arm7.g");
  G.ode();
  uint n=G.getJointStateDimension();
  arr F,Xt,Vt;
  generateSequence(F, 200, n);
  F *= .01;
  Xt.resizeAs(F); Vt.resizeAs(F);
  for(uint t=0;t<F.d0;t++){
    G.ode().addJointForce(F[t]());
    //G.clearJointErrors(); exportStateToOde(C,); //try doing this without clearing joint errors...!
    G.ode().step(0.03);
    G.ode().importStateFromOde();
    G.getJointState(Xt[t](),Vt[t]());
    G.gl().text.clear() <<"play a random torque sequence [using ODE] -- time " <<t;
    G.watch();
  }
}

void TEST(MeshShapesInOde){
  rai::KinematicWorld G("testOdeMesh.g");
  for(uint t=0;t<1000;t++){
    //G.clearJointErrors(); exportStateToOde(C,); //try doing this without clearing joint errors...!
    G.ode().step(0.03);
    G.ode().importStateFromOde();
    G.gl().timedupdate(.01);
  }
}
#endif

//===========================================================================
//
// standard IK test
//

void TEST(FollowRedundantSequence){  
  rai::KinematicWorld G("arm7.g");

  uint t,T,n=G.getJointStateDimension();
  arr x(n),y,J,invJ;
  x=.8;     //initialize with intermediate joint positions (non-singular positions)
  rai::Vector rel = G.getFrameByName("endeff")->Q.pos; //this frame describes the relative position of the endeffector wrt. 7th body

  //-- generate a random endeffector trajectory
  arr Z, Zt; //desired and true endeffector trajectories
  generateSequence(Z, 200, 3); //3D random sequence with limits [-1,1]
  Z *= .5;
  T=Z.d0;
  G.setJointState(x);
  rai::Frame *endeff = G.getFrameByName("arm7");
  G.kinematicsPos(y, NoArr, endeff, rel);
  for(t=0;t<T;t++) Z[t]() += y; //adjust coordinates to be inside the arm range
  plotLine(Z);
  G.glAdd(glDrawPlot,&plotModule);
  G.watch(false);
  //-- follow the trajectory kinematically
  for(t=0;t<T;t++){
    //Z[t] is the desired endeffector trajectory
    //x is the full joint state, z the endeffector position, J the Jacobian
    G.kinematicsPos(y, J, endeff, rel);  //get the new endeffector position
    invJ = ~J*inverse_SymPosDef(J*~J);
    x += invJ * (Z[t]-y);                  //simulate a time step (only kinematically)
    G.setJointState(x);
//    cout <<J * invJ <<endl <<x <<endl <<"tracking error = " <<maxDiff(Z[t],y) <<endl;
    G.watch(false, STRING("follow redundant trajectory -- time " <<t));
    //G.gl().timedupdate(.01);
  }
}

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
  rai::KinematicWorld G("arm7.g");
  G.optimizeTree();
  G.sortFrames();
  cout <<G <<endl;

  arr u;
  bool friction=false;
  VectorFunction diffEqn = [&G,&u,&friction](arr& y, arr&, const arr& x){
    checkNan(x);
    G.setJointState(x[0], x[1]);
    if(!u.N) u.resize(x.d1).setZero();
    if(friction) u = -1e-0 * x[1];
    checkNan(u);
    /*if(T2::addContactsToDynamics){
        G.contactsToForces(100.,10.);
      }*/
    G.fwdDynamics(y, x[1], u, true);
    checkNan(y);
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
      CHECK(maxDiff(qdd,qdd_,0)<1e-5,"dynamics and inverse dynamics inconsistent:\n" <<qdd <<'\n' <<qdd_);
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
      arr x=cat(q, qd).reshape(2, q.N);
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

/*void switchfunction(arr& s,const arr& x,const arr& v){
  G.setJointState(x,v);
  slGetProxies(C,ode);
  s.resize(G.bodies.N); s=.01;
  boolA c; c.resize(G.bodies.N);  c=false;
  uint i;
  int a,b;
  hasContact=false;
  for(i=0;i<G.proxies.N;i++) if(!G.proxies(i)->age){
    a=G.proxies(i)->a; b=G.proxies(i)->b;
    if(a>=0){ s(a) += G.proxies(i)->d; c(a) = true; }
    if(b>=0){ s(b) += G.proxies(i)->d; c(b) = true; }
    hasContact=true;
  }
  //for(i=0;i<s.N;i++) if(!c(i)) s(i)=.1;
}

bool checkContacts(const arr& s){
  uint i;
  for(i=0;i<s.N;i++) if(s(i)<0.) return true;
  return false;
}

//---------- test standard redundant control
void TEST(ContactDynamics){
  init();

  uint t,T=1000,n=G.getJointStateDimension();
  arr q,qd,qdd(n),qdd_(n),s;
  G.getJointState(q,qd);
  switchfunction(s,q,qd);

  double dt=.001;
  bool cross;

  ofstream z("z.dyn");
  G.clearForces();

  for(t=0;t<T;t++){
    if(!(t%1)){
      G.setJointState(q,qd);
      //G.zeroGaugeJoints();
      G.getJointState(q,qd);
    }
    z <<q <<qd <<qdd <<endl;
    conswit=s;
    if(false && checkContacts(s)){
      dt=.001;
      addContactsToDynamics=true;
      //G.watch(true);
    }else{
      dt=.01;
      addContactsToDynamics=false;
    }
    cross=rai::rk4dd_switch(q,qd,s,q,qd,s,ddf_joints,switchfunction,dt,1e-4);
    //G.reportProxies();
    cout <<"*** s = " <<s <<endl;
    G.gl().text.clear() <<"t=" <<t <<"  using RK4_switch,  energy=" <<G.getEnergy();
    //if(cross) G.watch(true);
    G.watch(false);
  }
}*/

//===========================================================================
//
// blender import test
//

#if 0
static void drawTrimesh(void* _mesh){
#if RAI_GL
  rai::Mesh *mesh=(rai::Mesh*)_mesh;
  glPushMatrix();
  mesh->glDraw();
  glPopMatrix();
#endif
}

void TEST(BlenderImport){
  rai::timerStart();
  rai::Mesh mesh;
  rai::KinematicWorld bl;
  readBlender("blender-export",mesh,bl);
  cout <<"loading time =" <<rai::timerRead() <<"sec" <<endl;
  OpenGL gl;
  G.glAdd(glStandardScene, NULL);
  G.glAdd(drawTrimesh,&mesh);
  G.watch(true, "mesh only");
  G.glAdd(rai::glDrawGraph,&bl);
  G.gl().text="testing blender import";
  animateConfiguration(bl,gl);
}
#endif

// =============================================================================
void TEST(InverseKinematics) {
  // we're testing some big steps / target positions, some of which are not
  // reachable to check if the IK handle it
  rai::KinematicWorld world("drawer.g");

  rai::Frame* drawer = world.getFrameByName("cabinet_drawer");
  rai::Frame* marker = world.getFrameByName("marker");
  arr destination = conv_vec2arr(marker->X.pos);

  cout << "destination: " << destination << endl;
  cout << "world state: " << world.q << endl;
  world.watch(true, STRING("press key to continue"));

  world.inverseKinematicsPos(*drawer, destination);
  cout << "destination: " << destination << endl;
  cout << "world state: " << world.q << endl;
  world.watch(true, STRING("press key to continue"));

  cout << "moving destination (can't be reached)" << endl;
  marker->X.pos.set(2., 1., 1);
  destination = conv_vec2arr(marker->X.pos);
  world.inverseKinematicsPos(*drawer, destination);
  cout << "destination: " << destination << endl;
  cout << "world state: " << world.q << endl;
  world.watch(true, STRING("press key to continue"));

  cout << "moving destination (can't be reached)" << endl;
  marker->X.pos.set(-2., 0.1, 1.2);
  destination = conv_vec2arr(marker->X.pos);
  world.inverseKinematicsPos(*drawer, destination);
  cout << "destination: " << destination << endl;
  cout << "world state: " << world.q << endl;
  world.watch(true, STRING("press key to continue"));
}

// =============================================================================

int MAIN(int argc,char **argv){
  rai::initCmdLine(argc, argv);

  testLoadSave();
  testCopy();
  testGraph();
  testPlayStateSequence();
  testKinematics();
  testQuaternionKinematics();
  testKinematicSpeed();
  testFollowRedundantSequence();
  testInverseKinematics();
  testDynamics();
  testContacts();
  testLimits();
#ifdef RAI_ODE
//  testMeshShapesInOde();
  testPlayTorqueSequenceInOde();
#endif
  //testBlenderImport();

  return 0;
}
