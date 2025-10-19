#include <Kin/kin.h>
#include <Kin/frame.h>
#include <Kin/viewer.h>
#include <Kin/i_Ode.h>
#include <Algo/spline.h>
#include <Algo/rungeKutta.h>
#include <Gui/opengl.h>
#include <Gui/plot.h>
#include <GL/gl.h>
#include <Kin/feature.h>
#include <Kin/dof_direction.h>

//===========================================================================
//
// test load save
//

void TEST(Mini){
  rai::Configuration C;
  C.addFrame("sphere")->setShape(rai::ST_sphere, {.1}).setPosition({0.,0.,.5});
  C.view(true);
}

//===========================================================================
//
// test load save
//

void TEST(LoadSave){
  rai::Configuration C;
  // C.addFile("../../../../rai-robotModels/panda/panda.g");
  C.addFile("$RAI_PATH/panda/panda.g");
  cout <<C <<endl;
  C.sortFrames();
  FILE("z.g") <<C;

  C.view();

  rai::Configuration C2("z.g");
  C2.view();
}

//===========================================================================
//
// Jacobian test
//

void testJacobianInFile(const char* filename, const char* shape){
  rai::Configuration C(filename);

  rai::Frame *a=C.getFrame(shape);

  VectorFunction f = ( [&a, &C](const arr& x) -> arr {
    C.setJointState(x);
    return C.kinematics_pos(a);
  } );

  checkJacobian(f, C.q, 1e-4);

//  rai::wait();
}

//===========================================================================

void TEST(Viewer){

  rai::Configuration C;
  C.view();

  C.addFrame("first") ->setShape(rai::ST_marker, {.3}). setPosition({0,0,.5});
  C.view();

  C.addFile(("$RAI_PATH/panda/panda.g"));
  C.view();

  rai::Frame *f = C.addFrame("mesh");
  f->setShape(rai::ST_mesh, {});

  rai::Frame *f2 = C.addFrame("tensor");
  f2->setTensorShape(rai::convert<float>(.02 * randn({5, 5, 5})), {.2, .2, .2});
  C.view(true);

  rai::Configuration C2;
  C2.addConfigurationCopy(C);
  C2.view();

  f->setPosition({.5, .5, 1.});
  f2->setPosition({-.2, -.2, .5});
  C2.frames.elem(-1)->setPosition({-.2, -.2, .5});

  for(uint k=0;k<20;k++){
    f->setConvexMesh(.2*randn({rnd.uni_int(5,10),3}), {255,0,0}, .05);
    f2->setTensorShape(rai::convert<float>(.02 * randn({k+2, k+2, k+2})), {.2, .2, .2});

    rai::wait(.01);
    C.view(false, STRING(k));
    C2.view(false, STRING(k));
  }
  C.view(false);
}

//===========================================================================
//
// Jacobian test
//

void TEST(Kinematics){

  struct MyFct : VectorFunction{
    enum Mode {Pos, Vec, Quat} mode;
    rai::Configuration& C;
    rai::Frame *b;
    rai::Vector &vec;
    MyFct(Mode _mode, rai::Configuration &_C, rai::Frame *_b, rai::Vector &_vec)
      : mode(_mode), C(_C), b(_b), vec(_vec){
      VectorFunction::operator= ( [this](const arr& x) -> arr {
        arr y, J;
        C.setJointState(x);
        switch(mode){
          case Pos:    C.kinematicsPos(y,J,b,vec); break;
          case Vec:    C.kinematicsVec(y,J,b,vec); break;
          case Quat:   C.kinematicsQuat(y,J,b); break;
        }
        y.J() = J;
        return y;
        //if(!!J) cout <<"\nJ=" <<J <<endl;
      } );
    }
    VectorFunction operator()(){ return *this; }
  };

//  rai::Configuration G("arm7.g");
  rai::Configuration C("kinematicTests.g");
//  rai::Configuration G("../../../../rai-robotModels/pr2/pr2.g");
//  rai::Configuration G("../../../projects/17-LGP-push/quatJacTest.g");
//  G.view(true);

  C.calc_indexedActiveJoints();

  C.jacMode = C.JM_sparse;

  for(uint k=0;k<10;k++){
    rai::Frame *b = C.frames.rndElem();
    rai::Vector vec=0, vec2=0;
    vec.setRandom();
    vec2.setRandom();
    arr x(C.getJointStateDimension());
    rndUniform(x,-.5,.5,false);

    cout <<"kinematicsPos:   "; checkJacobian(MyFct(MyFct::Pos  , C, b, vec)(), x, 1e-5);
    cout <<"kinematicsVec:   "; checkJacobian(MyFct(MyFct::Vec  , C, b, vec)(), x, 1e-5);
    cout <<"kinematicsQuat:  "; checkJacobian(MyFct(MyFct::Quat , C, b, vec)(), x, 1e-5);

    //checkJacobian(Convert(T1::f_hess, nullptr), x, 1e-5);
  }
}

//===========================================================================
//
// Graph export test
//

void TEST(Graph){
  if(!rai::FileToken("../../../../rai-robotModels/pr2/pr2.g").exists()) return;
  
//  rai::Configuration G("arm7.g");
//  rai::Configuration K("kinematicTests.g");
  rai::Configuration C("../../../../rai-robotModels/pr2/pr2.g");
//  rai::Configuration G("../../../projects/17-LGP-push/quatJacTest.g");
//  G.view(true);

  C.prefixNames();
  {
    rai::Graph G = C.getGraph();
    G.displayDot();
    rai::wait(.5);
  }

  C.processStructure();
  {
    rai::Graph G = C.getGraph();
    G.displayDot();
    rai::wait(.5);
  }

}

//===========================================================================
//
// Jacobian test
//

void TEST(QuaternionKinematics){
  rai::Configuration G("kinematicTestQuat.g");

  for(uint k=0;k<5;k++){
    rai::Quaternion target;
    target.setRandom();
    G.getFrame("ref")->set_Q()->rot = target;
    G.getFrame("marker")->set_Q()->rot = target;
    rai::Frame *endeff = G.getFrame("endeff");
    arr x = G.getJointState();
    for(uint t=0;t<100;t++){
      arr y,J;
      G.kinematicsQuat(y, J, endeff);  //get the new endeffector position
      arr Jinv = pseudoInverse(J, NoArr, 1e-4); //~J*inverse_SymPosDef(J*~J);
      if(scalarProduct(conv_quat2arr(target),y)<0.) target.flipSign();
      x += 0.05 * Jinv * (conv_quat2arr(target)-y);                  //simulate a time step (only kinematically)
      G.setJointState(x);
      G.view(false, STRING("test quaternion task spaces -- time " <<t));
      rai::wait(.01);
    }
  }
}

//===========================================================================

struct F_Direction : Feature {
  F_Direction() {}
  virtual arr phi(const FrameL& F){
    rai::Frame *f = F.elem();
    CHECK(f->dirDof, "");
    arr y;
    f->dirDof->kinVec(y, y.J());
    return y;
  }
  uint dim_phi(const FrameL& F) { return 3; }
};

void testDirectionKinematics(){
  rai::Configuration C("kinematicTestDir.g");
  rai::Frame *dir = C["dir"];

  for(uint k=0;k<20;k++){
    // arr q = rand(C.getJointStateDimension());
    // C.setJointState(q);
    C.setRandom();
    arr q = C.getJointState();
    cout <<k <<' ' <<q <<dir->dirDof->vec <<endl;

    checkJacobian(F_Direction().asFct({dir}), q, 1e-5);

    C.view(true);
  }
}

//===========================================================================
//
// copy operator test
//

void TEST(Copy){
  rai::Configuration G1("kinematicTests.g");
  //rai::Configuration G1("../../../data/pr2_model/pr2_model.g");
  rai::Configuration G2(G1);

  G1.checkConsistency();
  G2.checkConsistency();

  G1 >>FILE("z.1");
  G2 >>FILE("z.2");

  rai::String g1,g2;
  g1.read(FILE("z.1"), "", "", -1); //no stop symbols
  g2.read(FILE("z.2"), "", "", -1);

  CHECK_EQ(g1, g2, "copy operator failed!")
  cout <<"** copy operator success" <<endl;
}

//===========================================================================
//
// grid test
//

void TEST(Grid){
  rai::Configuration C;
  auto table = C.addFrame("table");
  table-> setShape(rai::ST_ssBox, {.5, .5, .1, .01}) .setPosition({0,0,1}) .setColor({.5});
  auto panda = C.addFile(("$RAI_PATH/panda/panda.g"));
  panda->setParent(table) .setRelativePosition({0,0,.05});
  C.view();

  rai::Configuration Cgrid;
  int d = 10;
  for(int x=0;x<d;x++){
    for(int y=0;y<d;y++){
      auto base = Cgrid.addConfigurationCopy(C, STRING("x" <<x <<"_y" <<y <<"_")); //the string adds a prefix to all frame names
      base->setPosition({double(x-(d-1)/2.), double(y-(d-1)/2.), 1.});
    }
  }
  Cgrid.report();
  Cgrid.view();

  C.get_viewer()->report(cout);
  Cgrid.get_viewer()->renderUntil=rai::_solid;
//  Cgrid.get_viewer()->nonThreaded=true;
  Cgrid.get_viewer()->report(cout);

  Cgrid.animateSpline();
}

//===========================================================================
//
// Kinematic speed test
//

void TEST(KinematicSpeed){
#define NUM 100000
#if 1
//  rai::Configuration K("kinematicTests.g");
  if(!rai::FileToken("../../../../rai-robotModels/pr2/pr2.g").exists()) return;
  rai::Configuration K("../../../../rai-robotModels/pr2/pr2.g");
  K.processStructure();
  uint n=K.getJointStateDimension();
  arr x(n);
  rai::timerStart();
  for(uint k=0;k<NUM;k++){
    rndUniform(x,-.5,.5,false);
    K.setJointState(x);
//    G.view();
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
  rai::Configuration G("arm7.g");
  
  arr x,con,grad;
  uint t;

  //G.swift()->cutoff =.5;

  VectorFunction f = [&G](const arr& x) -> arr {
    G.setJointState(x);
    G.coll_stepFcl();
    arr y;
    G.kinematicsPenetration(y, y.J(), .2);
    return y;
  };

  x = G.getJointState();
  for(t=0;t<100;t++){
    G.setJointState(x);
    G.coll_stepFcl();

    G.coll_reportProxies();

    G.jacMode = G.JM_dense;
    G.kinematicsPenetration(con, grad, .2);
    cout <<"contact meassure = " <<con(0) <<endl;
    //G.view(true);
    G.view(false, STRING("t=" <<t <<"  movement along negative contact gradient (using SWIFT to get contacts)"));
    //x += inverse(grad)*(-.1*c);
    x -= 3e-3*grad; //.1 * (invJ * grad);

    checkJacobian(f, x, 1e10);
  }
}

//===========================================================================

void TEST(Limits){
  rai::Configuration C("arm7.g");

  arr limits = C.getJointLimits();
  cout <<"limits: " <<limits <<endl;
//  VectorFunction F = [&G, &limits](arr& y, arr& J, const arr& x){
//    G.setJointState(x);
//    G.setJacModeAs(J);
//    G.kinematicsLimits(y,J,limits);
//  };

  auto F = C.feature(FS_jointLimits);

  uint n=C.getJointStateDimension();
  arr x(n);
  for(uint k=0;k<10;k++){
    rndUniform(x,-2.,2.,false);
    checkJacobian(F->asFct(F->getFrames(C)),x,1e-4);
    for(uint t=0;t<10;t++){
      arr lim = F->eval(F->getFrames(C));
      cout <<"y=" <<lim <<"  " <<std::flush;
//      cout <<"J:" <<lim.J <<endl;
      for(uint i=0;i<lim.N;i++) if(lim(i)<0.) lim(i)=0.; //penalize only positive
      x -= 1. * pseudoInverse(lim.J()) * lim;
      checkJacobian(F->asFct(F->getFrames(C)),x,1e-4);
      C.setJointState(x);
      C.view();
    }
  }
}

//===========================================================================
//
// set state test
//

void testPlaySpline(){
  rai::Configuration C(("$RAI_PATH/panda/panda.g"));
  C.animateSpline(5);
}

//===========================================================================
//
// ODE test
//

#ifdef RAI_ODE
void TEST(PlayTorqueSequenceInOde){
  rai::Configuration G("arm7.g");
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
    G.gl()->text.clear() <<"play a random torque sequence [using ODE] -- time " <<t;
    G.view();
  }
}

void TEST(MeshShapesInOde){
  rai::Configuration G("testOdeMesh.g");
  for(uint t=0;t<1000;t++){
    //G.clearJointErrors(); exportStateToOde(C,); //try doing this without clearing joint errors...!
    G.ode().step(0.03);
    G.ode().importStateFromOde();
    G.gl()->timedupdate(.01);
  }
}
#endif

//===========================================================================
//
// standard IK test
//

arr generateSpline(uint T, uint n){
  rnd.seed(0);
  arr P(10,n);
  rndUniform(P,-1.,1.,false); P[0]=0.; P[P.d0-1]=0.;
  return rai::BSpline().set(2,P, range(0.,1.,P.d0-1)).eval(range(0.,1.,T));
}

void TEST(FollowRedundantSequence){  
  rai::Configuration C("arm7.g");

  uint t,T,n=C.getJointStateDimension();
  arr x(n),y,J;
  x=.8;     //initialize with intermediate joint positions (non-singular positions)
  rai::Vector rel = C.getFrame("endeff")->get_Q().pos; //this frame describes the relative position of the endeffector wrt. 7th body

  //-- generate a random endeffector trajectory
  arr Z = generateSpline(200, 3); //3D random sequence with limits [-1,1]
  Z *= .5;
  T=Z.d0;
  C.setJointState(x);
  rai::Frame *endeff = C.getFrame("arm7");
  C.kinematicsPos(y, NoArr, endeff, rel);
  for(t=0;t<T;t++) Z[t] += y; //adjust coordinates to be inside the arm range

  arr lines(2*(Z.d0-1), 3);
  for(uint i=0;i<Z.d0-1;i++){ lines[2*i]=Z[i]; lines[2*i+1]=Z[i+1];  }
  C.addFrame("plotPath")
      ->setLines(lines, {})
      .setShape(rai::ST_lines, {1.,1.,0.});
  C.view(false);
  //-- follow the trajectory kinematically
  for(t=0;t<T;t++){
    //Z[t] is the desired endeffector trajectory
    //x is the full joint state, z the endeffector position, J the Jacobian
    C.kinematicsPos(y, J, endeff, rel);  //get the new endeffector position
//    invJ = ~J*inverse_SymPosDef(J*~J);
//    x += invJ * (Z[t]-y);                  //simulate a time step (only kinematically)
    x += ~J * lapack_Ainv_b_sym(J*~J, Z[t]-y);
    C.setJointState(x);
//    cout <<J * invJ <<endl <<x <<endl <<"tracking error = " <<maxDiff(Z[t],y) <<endl;
    C.view(false, STRING("follow redundant trajectory -- time " <<t));
    //G.gl()->timedupdate(.01);
    rai::wait(.01);
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
//  rai::Configuration *G;
//}

//---------- test standard dynamic control
void TEST(Dynamics){
  rai::Configuration C("arm7.g");
  C.processStructure();
  C.sortFrames();
  cout <<C <<endl;

  arr u;
  bool friction=false;
  auto diffEqn = [&C,&u,&friction](const arr& x, const arr& xdot) -> arr{
    checkNan(x);
    C.setJointState(x);
    if(!u.N) u.resize(x.N).setZero();
    if(friction) u = -1e-1 * xdot;
    checkNan(u);
    arr y, ytest;
    y = C.fwdDynamics(xdot, u, true);
    ytest = C.dyn_fwdDynamics(xdot, u);
    arr M, F, Mtest, Ftest;
    cout <<"qdd precision: " <<sumOfSqr(y-ytest) <<endl; // <<M <<endl <<Mtest <<endl
    C.equationOfMotion(M, F, xdot, true);
    C.dyn_MF(Mtest, Ftest, xdot);
    cout <<"M precision:   " <<sumOfSqr(M-Mtest) <<endl; // <<M <<endl <<Mtest <<endl
    cout <<"F precision:   " <<sumOfSqr(F-Ftest) <<endl; // <<F <<endl <<Ftest <<endl;
    checkNan(y);
    return y;
  };
  
  arr q = C.getJointState();
  arr qd = zeros(q.N);
  arr qdd = zeros(q.N);
  arr qdd_des;
  
  double dt=.01;

  ofstream z("z.dyn");
  rai::String text;
  C.view();

  for(uint t=0;t<750;t++){
    if(t>=400){
      friction=false;
//      qdd_ = -1. * qd;
      double tau = .5, xi = 0.9, kp = 1/(tau*tau), kd = 2*xi/tau;
      qdd_des = -kp * q - kd * qd;
      C.inverseDynamics(u, qd, qdd_des);
      arr utest = C.dyn_inverseDyamics(qd, qdd_des);
      cout <<"inv precision: " <<sumOfSqr(u-utest) <<endl; //<<u <<utest <<endl;
      qdd = C.dyn_fwdDynamics(qd, u);
      CHECK(maxDiff(qdd,qdd_des,0)<1e-5,"dynamics and inverse dynamics inconsistent:\n" <<qdd <<'\n' <<qdd_des);
      q  += .5*dt*qd;
      qd +=    dt*qdd;
      q  += .5*dt*qd;
      C.setJointState(q);
      text.clear() <<"t=" <<t <<"  torque controlled PD behavior\n  dynamics test: fwd-inv error =" <<maxDiff(qdd,qdd_des,0) <<" energy =" <<C.getEnergy(qd) <<endl;
    }else{
      //cout <<q <<qd <<qdd <<' ' <<G.getEnergy() <<endl;
      arr x=(q, qd).reshape(2, q.N);
      x = rai::rk4_2ndOrder(x, diffEqn, dt);
      q=x[0]; qd=x[1];
      if(t>200){
        friction=true;
        text.clear() <<"t=" <<t <<"  FRICTION swing using RK4,  energy=" <<C.getEnergy(qd);
      }else{
        friction=false;
        text.clear() <<"t=" <<t <<"  free swing using RK4,  energy=" <<C.getEnergy(qd);
      }
    }
    C.view(false, text);
    rai::wait(.01);
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
      //G.view(true);
    }else{
      dt=.01;
      addContactsToDynamics=false;
    }
    cross=rai::rk4dd_switch(q,qd,s,q,qd,s,ddf_joints,switchfunction,dt,1e-4);
    //G.reportProxies();
    cout <<"*** s = " <<s <<endl;
    G.gl()->text.clear() <<"t=" <<t <<"  using RK4_switch,  energy=" <<G.getEnergy();
    //if(cross) G.view(true);
    G.view(false);
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
  rai::Configuration bl;
  readBlender("blender-export",mesh,bl);
  cout <<"loading time =" <<rai::timerRead() <<"sec" <<endl;
  OpenGL gl;
  G.glAdd(glStandardScene, nullptr);
  G.glAdd(drawTrimesh,&mesh);
  G.view(true, "mesh only");
  G.glAdd(rai::glDrawGraph,&bl);
  G.gl()->text="testing blender import";
  animateConfiguration(bl,gl);
}
#endif

// =============================================================================

void testTexture(){
  {
    rai::Configuration C;
    rai::Frame *f = C.addFrame("box");
    f->setShape(rai::ST_box, {.2, .2, .2});
    f->setPosition({0, 0, 1});
    f->setTextureFile("../../Gui/retired/opengl/box.png",  randn(8,2));
    C.view(true);
  }
  {
    rai::Configuration C;
    rai::Frame *f = C.addFrame("mesh");
    // f->setMeshFile("/home/mtoussai/git/MuJoCo2Rai/fixtures/coffee_machines/nespresso/visuals/model_0.obj");
    f->setMeshFile("/home/mtoussai/git/MuJoCo2Rai/MuJoCo2Rai/meshes/toaster_main_group_main_128_shape0.h5");
    f->setPosition({0, 0, 1});
    C.view(true);
    C.writeMeshes("meshes/");
  }
}

// =============================================================================

void testInertias(){
  rai::Configuration C;
  C.addFile(("$RAI_PATH/tests/compound.g"));
  rai::Frame *obj1 = C.getFrame("obj");
  rai::Frame *obj2 = C.addFrame("obj2");
  rai::Frame *mesh = C.addFrame("mesh", "obj2");
  rai::Mesh M;
  M.setBox(); M.scale(.1, .2, .3);
  // M.translate(1.,0,0); mesh->setRelativePosition({-1.,0,0});
  mesh->setMesh2(M);
  mesh->setMass(-1.);

  cout <<obj1->getInertia() <<endl;
  cout <<obj2->getInertia() <<endl;
  obj1->computeCompoundInertia();
  obj2->computeCompoundInertia();
  cout <<obj1->getInertia() <<endl;
  cout <<obj2->getInertia() <<endl;

  C.processInertias(true);
  cout <<C <<endl;
  cout <<obj1->getInertia() <<endl;
  cout <<obj2->getInertia() <<endl;
  C.view(true);
}

// =============================================================================

void testMergeSceneMesh(){
  rai::Configuration C;
  C.addFile(("$RAI_PATH/robo_casa/kitchen2/kitchen2.g"));
  C.view(true);

  //-- build a mesh which is the union of all convex hulls
  rai::Mesh M;
  for(rai::Frame *f:C.frames){
    if(f->shape && f->shape->_mesh){
      rai::Mesh m = f->shape->mesh();
      m.makeConvexHull();
      M.addMesh(m, f->ensure_X());
    }
  }

  //-- display it
  rai::Configuration C2;
  rai::Frame *f = C2.addFrame("global_collision");
  f->setMesh2(M);
  cout <<"mesh size: " <<M <<endl;
  C2.view(true);

  //-- decompose it into simpler convex parts
  M = M.decompose();
  f->setMesh2(M);
  cout <<"mesh size: " <<M <<endl;
  C2.view(true);
}

// =============================================================================

int MAIN(int argc,char **argv){
  rai::initCmdLine(argc, argv);

  testViewer(); return 0;

  testMini();
  testLoadSave();
  testCopy();
  testGraph();
  testPlaySpline();
  testViewer();
  testKinematics();
  testQuaternionKinematics();
  testDirectionKinematics();
  testKinematicSpeed();
  testGrid();
  testFollowRedundantSequence();
  testDynamics();
  testContacts();
  testLimits();
  testInertias();
  //testTexture();
  //testMergeSceneMesh();
#ifdef RAI_ODE
//  testMeshShapesInOde();
  testPlayTorqueSequenceInOde();
#endif
  //testBlenderImport();

  return 0;
}
