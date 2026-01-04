#include <stdlib.h>

#include <Geo/mesh.h>
#include <Gui/opengl.h>
#include <Kin/viewer.h>
#include <Kin/kin.h>
#include <Kin/frame.h>
#include <Geo/pairCollision.h>
#include <Kin/F_collisions.h>
#include <Kin/F_qFeatures.h>

bool interactive=false;

//===========================================================================

void TEST(GJK_Jacobians) {
  rai::Configuration C;
  rai::Frame base(C), b1(&base, rai::JT_free), B1(&b1, rai::JT_free), b2(&B1, rai::JT_free), B2(&b2, rai::JT_free);

  C.calcDofsFromConfig();
  arr q = C.getJointState();

  C.get_viewer()->opt.polygonLines=true;

  F_PairCollision dist(F_PairCollision::_negScalar);
  F_PairCollision distVec(F_PairCollision::_vector);
  F_PairCollision distNorm(F_PairCollision::_normal);
  F_PairCollision distCenter(F_PairCollision::_center);
  FrameL F = {&B1, &B2};

  for(uint k=0;k<100;k++){
    //randomize shapes
    B1.setConvexMesh(2.*rand(10,3), {128,255,128,100}, .05); //rnd.uni(.01, .3));
    B2.setConvexMesh(2.*rand(10,3), {128,128,255,100}, .05); //rnd.uni(.01, .3));

    //randomize poses
    rndGauss(q, .7);
    C.setJointState(q);

    bool succ = true;

    arr y = dist.eval(F);
    cout <<k <<" dist ";
    succ &= checkJacobian(dist.asFct(F), q, 1e-5);

    arr y2 = distVec.eval(F);
    cout <<k <<" vec  ";
    succ &= checkJacobian(distVec.asFct(F), q, 1e-5);

    arr y3 = distNorm.eval(F);
    cout <<k <<" norm  ";
    succ &= checkJacobian(distNorm.asFct(F), q, 1e-5);

    y3 = distCenter.eval(F);
    cout <<k <<" center  ";
    succ &= checkJacobian(distCenter.asFct(F), q, 1e-5);

    // rai::PairCollision_CvxCvx collInfo(B1.getShape().sscCore(), B2.getShape().sscCore(), B1.ensure_X(), B2.ensure_X(), B1.getSize()(-1), B2.getSize()(-1));
    //    cout <<"distance: " <<y <<" vec=" <<y2 <<" error=" <<length(y2)-fabs(y(0)) <<endl;

    C.proxies.resize(1);
    C.proxies(0).A = B1.ID;
    C.proxies(0).B = B2.ID;
    C.proxies(0).calc_coll(C.frames);

    if(!succ || interactive)
      cout <<C.proxies(0).ensure_coll(C.frames);

    C.view(!succ || interactive, STRING(k));

    if(succ){
      CHECK_ZERO(length(y2)-fabs(y(0)), 1e-3, "");
    }
  }
}

//===========================================================================

void TEST(GJK_Jacobians2) {
  rai::Configuration C;
  C.addFrame("base")->setPosition({.0, .0, 1.});
  for(uint i=0;i<20;i++){
    rai::Frame *a = C.addFrame(STRING("obj_" <<i), "base");
    a->setJoint(rai::JT_free);
    a->set_Q()->setRandom();

    a->setConvexMesh(rand(10,3), {128,128,255,100}, rnd.uni(.01, .1));
    a->setContact(1);
  }

  C.ensure_proxies(true);
  C.coll_reportProxies();
  if(interactive) C.view(true);

  VectorFunction f = [&C](const arr& x) -> arr {
    C.setJointState(x);
    C.ensure_proxies(true);
    arr y;
    C.kinematicsPenetration(y, y.J(), .05);
    return y;
  };

//  checkJacobian(f, K.getJointState(), 1e-4);

  arr q = C.getJointState();
//  K.orsDrawProxies=false;
  double y_last=0.;
  for(uint t=0;t<1000;t++){
    C.setJointState(q);
    C.ensure_proxies(true);
    //C.coll_reportProxies();

//    checkJacobian(f, q, 1e-4);

    F_qQuaternionNorms qn;
    qn.setFrameIDs(framesToIndices(C.frames));

    arr y,J;
    C.kinematicsPenetration(y, J, .05);

    arr y2 = qn.eval(qn.getFrames(C));

    cout <<"total penetration: " <<y(0) <<"  diff:" <<y(0) - y_last <<endl; //" quat-non-normalization=" <<y2(0) <<endl;
    y_last = y(0);
    C.view(interactive, STRING("t=" <<t <<"  movement along negative contact gradient"));

    q -= 1e-2*J + 1e-2*(~y2*y2.J());

    if(y(0)<1e-10) break;
  }

  C.view(true);
}

//===========================================================================

void TEST(GJK_Jacobians3) {
  rai::Configuration C;
  rai::Frame base(C), B1(C), B2(C);
  rai::Joint J1(base, B1), J2(base, B2);
  rai::Shape &s1 = B1.getShape(), &s2=B2.getShape();
  J1.setType(rai::JT_free);
  J2.setType(rai::JT_rigid);
//  B1.set_Q()->setRandom();
  B1.set_Q()->pos = {0.,0., 1.05};
  B2.set_Q()->pos = {0.,0., 1.21};
  B1.set_Q()->pos.x += .03;
  B1.set_Q()->pos.y += .03;
//  B1.set_Q()->rot.addX(.01);
  s1.cont=s2.cont = true;
  B1.name = "1"; B2.name="2";

  s1.type() = s2.type() = rai::ST_ssBox;
  s1.size = {.2, .2, .2, .01 };
  s2.size = {.2, .2, .2, .01 };
  s1.createMeshes(B1.name);
  s1.mesh().C = {.5,.8,.5,.9};
  s2.mesh().C = {.5,.5,.8,.9};
  s2.createMeshes(B2.name);

  C.gl().drawOptions.drawProxies=true;

  C.coll_stepFcl(rai::_broadPhaseOnly);
  C.coll_reportProxies();

  arr q = C.getJointState();

  for(uint t=0;t<10;t++){
    C.setJointState(q);
    C.coll_stepFcl(rai::_broadPhaseOnly);
//    C.stepFcl();
//    K.reportProxies(cout, -1., false);

//    PairCollision collInfo(s1.sscCore(), s2.sscCore(), s1.frame.X, s2.frame.X, s1.size(-1), s2.size(-1));

    F_PairCollision gjk(F_PairCollision::_negScalar);
    FrameL F = {&B1, &B2};
    bool good = checkJacobian(gjk.asFct(F), q, 1e-4);
    if(!good) rndGauss(q, 1e-4, true);

    arr y = gjk.eval(F);

    F_qQuaternionNorms qn;
    arr y2 = qn.eval(C.frames);

    cout <<"contact meassure = " <<y(0) <<endl;
    C.view(interactive, STRING("t=" <<t <<"  movement along negative contact gradient"));

    q -= 1e-2*y.J() + 1e-2*(~y2*y2.J());

    rai::wait(.05);
  }

  C.view(true);
}

//===========================================================================

void TEST(Functional) {
  rai::Configuration C;
  auto base = C.addFrame("base");
  base->setPosition({0.,0.,1.});
  for(uint i=0;i<2;i++){
    rai::Frame *a = C.addFrame(STRING("obj_" <<i), "base");
    a->setJoint(rai::JT_free);
    a->set_Q()->setRandom();

    if(i==0)
      a->setShape(rai::ST_sphere, {1.});
    else
//      a->setShape(rai::ST_capsule, {1.,.3});
      a->setShape(rai::ST_ssBox, {1.3, 1.2, 1.1, .1});
    a->setColor({.8,.8,.8,.6});
    a->setContact(1);
  }

  arr x = C.getJointState();
  for(uint t=0;t<10;t++){
    rndGauss(x, .7);
    C.setJointState(x);

//    F_PairCollision dist(F_PairCollision::_negScalar);
    F_PairFunctional dist;
    auto y = dist.eval({C(1), C(2)});
    checkJacobian(dist.asFct({C(1), C(2)}), x, 1e-4);

    C.get_viewer()->clear();
    C.get_viewer()->updateConfiguration(C);
    C.get_viewer()->addDistMarker(dist.x-dist.d1*dist.g1, dist.x-dist.d2*dist.g2, .1);
    C.view(true);
  }
}

//===========================================================================

void testSweepingSDFs(){
  //-- create a single config with 2 objects
  rai::Configuration C0;
  auto base = C0.addFrame("base");
  base->setPosition({0.,0.,1.});
  for(uint i=0;i<2;i++){
    rai::Frame *a = C0.addFrame(STRING("obj_" <<i), "base");
    a->setJoint(rai::JT_free);
    a->set_Q()->setRandom();
    if(i==0)
//      a->setShape(rai::ST_sphere, {.1});
    a->setShape(rai::ST_capsule, {.5,.1});
    else
//      a->setShape(rai::ST_sphere, {.1});
      a->setShape(rai::ST_capsule, {.5,.1});
//      a->setShape(rai::ST_ssBox, {.3, .2, .1, .03});
//      a->setShape(rai::ST_ssBox, {1.3, 1.2, 1.1, .1});
    a->setColor({.8,.8,.8,.6});
    a->setContact(1);
  }

  rai::Configuration C;
  C.addConfigurationCopy(C0);
  C.addConfigurationCopy(C0);
  FrameL F ({2,2},{C.frames(0,1), C.frames(0,2), C.frames(1,1), C.frames(1,2)});

  C.addFrame("a") ->setParent(F(0,0)). setShape(rai::ST_marker, {.3}). setColor({1.,0.,0.});
  C.addFrame("b") ->setParent(F(0,1)). setShape(rai::ST_marker, {.3}). setColor({1.,0.,0.});
  C.addFrame("c") ->setParent(F(1,0)). setShape(rai::ST_marker, {.3}). setColor({1.,1.,0.});
  C.addFrame("d") ->setParent(F(1,1)). setShape(rai::ST_marker, {.3}). setColor({1.,1.,0.});

  C.addFrame("sweep1");
  C.addFrame("sweep2");

  rai::Mesh sweep1;
  rai::Mesh sweep2;

  arr x = C.getJointState();
  for(uint t=0;t<20;t++){
    rndGauss(x, .7);
    C.setJointState(x);

    F_PairFunctional dist;
//    auto y = dist.eval({C.frames(0,1), C.frames(0,2)});
//    checkJacobian(dist.vf2({C.frames(0,1), C.frames(0,2)}), x, 1e-4);
    dist.setOrder(1);
    auto y = dist.eval(F);
    checkJacobian(dist.asFct(F), x, 1e-4);

    arr V = F(0,0)->getMeshPoints();
    F(0,0)->ensure_X().applyOnPointArray(V);
    arr vel = (F(1,0)->ensure_X().pos - F(0,0)->ensure_X().pos).getArr();
    sweep1.clear();
    sweep1.C = {.7, .9, .7, .3};
    sweep1.V.append(V);
    sweep1.V.append(V+(ones(V.d0)^vel));
    sweep1.makeConvexHull();
    C.getFrame("sweep1")->setMesh2(sweep1) .setColor({.5,1.,.5,.5});

    V = F(0,1)->getMeshPoints();
    F(0,1)->ensure_X().applyOnPointArray(V);
    vel = (F(1,1)->ensure_X().pos - F(0,1)->ensure_X().pos).getArr();
    sweep2.clear();
    sweep2.C = {.7, .7, .9, .3};
    sweep2.V.append(V);
    sweep2.V.append(V+(ones(V.d0)^vel));
    sweep2.makeConvexHull();
    C.getFrame("sweep2")->setMesh2(sweep2) .setColor({.5,.5,1.,.5});

    C.proxies.resize(1);
    C.proxies(0).posA = dist.x-dist.d1*dist.g1;
    C.proxies(0).posB = dist.x-dist.d2*dist.g2;

    C.view(interactive);
  }

  C.view(true);
}

//===========================================================================

void testPoint2PCL(){
  rai::Configuration C;
  C.addFrame("base");
  C.addFrame("sphere") ->setShape(rai::ST_sphere, {.1}) .setPosition({0.,0.,1.}) .setParent(C.frames(0), true) .setJoint(rai::JT_trans3);
  C.addFrame("pcl") ->setPointCloud(randn(20,3), {255,255,0}) .setPosition({0.,0.,1.});
  rai::Frame& m = C.addFrame("marker") ->setShape(rai::ST_marker, {.5});
  // C.animate();
  FrameL F = {C.frames(1), C.frames(2)};

  arr q0 = C.getJointState();
  for(uint t=0;t<20;t++){
    arr q = q0 + .3*randn(q0.N);
    C.setJointState(q);

    F_PairCollision coll;
    auto y = coll.eval(F);
    cout <<q <<' ' <<y <<endl;
    checkJacobian(coll.asFct(F), q, 1e-4);

    m.setPosition(q);
    m.set_X()->rot.setDiff(Vector_x, y.J());
    C.view(interactive);
  }

  C.view(true);
}

//===========================================================================

int MAIN(int argc, char** argv){
  rai::initCmdLine(argc, argv);

  // rnd.seed_random();


  testGJK_Jacobians();
  testGJK_Jacobians2();
  testGJK_Jacobians3();

  // testFunctional();
  testSweepingSDFs();
  testPoint2PCL();

  return 0;
}
