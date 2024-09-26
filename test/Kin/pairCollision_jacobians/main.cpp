#include <stdlib.h>

#include <Geo/mesh.h>
#include <Gui/opengl.h>
#include <Kin/viewer.h>
#include <Kin/kin.h>
#include <Kin/frame.h>
#include <Geo/pairCollision.h>
#include <Kin/F_collisions.h>
#include <Kin/F_qFeatures.h>

//===========================================================================

void TEST(GJK_Jacobians) {
  rai::Configuration C;
  rai::Frame base(C), b1(C), B1(C), b2(C), B2(C);
  rai::Joint j1(base, b1), J1(b1, B1), j2(B1, b2), J2(b2, B2);
  rai::Shape s1(B1), s2(B2);
  j1.setType(rai::JT_free);
  j2.setType(rai::JT_free);
  j1.frame->insertPreLink(rai::Transformation(0))->set_Q()->addRelativeTranslation(1,1,1);
  j2.frame->insertPreLink(rai::Transformation(0))->set_Q()->addRelativeTranslation(-1,-1,1);
  J1.setType(rai::JT_free);
  J2.setType(rai::JT_free);

  C.calcDofsFromConfig();
  arr q = C.getJointState();

//  OpenGL gl;
//  gl.drawOptions.drawWires=true;
//  gl.data().addStandardScene();
//  gl.add(draw);
//  gl.add(K);

  F_PairCollision dist(F_PairCollision::_negScalar);
  F_PairCollision distVec(F_PairCollision::_vector);
  F_PairCollision distNorm(F_PairCollision::_normal);
  F_PairCollision distCenter(F_PairCollision::_center);
  FrameL F = {&B1, &B2};
//  dist.setFrameIDs({B1.ID, B2.ID});
//  distVec.setFrameIDs({B1.ID, B2.ID});
//  distNorm.setFrameIDs({B1.ID, B2.ID});
//  distCenter.setFrameIDs({B1.ID, B2.ID});

  for(uint k=0;k<100;k++){
//    //randomize shapes
    s1.mesh().clear();             s2.mesh().clear();
    s1.sscCore().setRandom();      s2.sscCore().setRandom();
    s1.sscCore().scale(2.);       s2.sscCore().scale(2.);
    s1.mesh().C = {.5,.8,.5,.4};   s2.mesh().C = {.5,.5,.8,.4};
    s1.type() = s2.type() = rai::ST_ssCvx; //ST_mesh;
    s1.size = arr{rnd.uni(.01, .3)}; s2.size = arr{rnd.uni(.01, .3)};
    if(rnd.uni()<.2) s1.sscCore().setDot();
    if(rnd.uni()<.2) s2.sscCore().setDot();
    s1.createMeshes();
    s2.createMeshes();

    //randomize poses
    rndGauss(q, .7);
    C.setJointState(q);

    bool succ = true;

//    if(rnd.uni()<.1){
//      B1.setPosition({-.4,0.,1.});
//      B1.setQuaternion({1,0,0,0});
//      B2.setPosition({.4,0.,1.});
//      B2.setQuaternion({1,0,0,0});
//      B1.setShape(rai::ST_ssBox, {1., .5, .5, .1});
//      B2.setShape(rai::ST_ssBox, {1., .5, .5, .1});
//      B1.setColor({.5,.5,.8,.4});
//      B2.setColor({.5,.8,.5,.4});
//      succ=false;
//      q = C.getJointState();
//    }
    succ=true;

    arr y = dist.eval(F);
    cout <<k <<" dist ";
    succ &= checkJacobian(dist.vf2(F), q, 1e-5);

    arr y2 = distVec.eval(F);
    cout <<k <<" vec  ";
    succ &= checkJacobian(distVec.vf2(F), q, 1e-5);

    arr y3 = distNorm.eval(F);
    cout <<k <<" norm  ";
    succ &= checkJacobian(distNorm.vf2(F), q, 1e-5);

    y3 = distCenter.eval(F);
    cout <<k <<" center  ";
    succ &= checkJacobian(distCenter.vf2(F), q, 1e-5);

    rai::PairCollision collInfo(s1.sscCore(), s2.sscCore(), s1.frame.ensure_X(), s2.frame.ensure_X(), s1.size(-1), s2.size(-1));

    //    cout <<"distance: " <<y <<" vec=" <<y2 <<" error=" <<length(y2)-fabs(y(0)) <<endl;
    if(!succ) cout <<collInfo;

    C.get_viewer()->updateConfiguration(C);
    C.get_viewer()->addDistMarker(collInfo.p1, collInfo.p2, 1.);
    C.view(!succ, STRING(k));
//    C.view(true);

    if(succ){
      CHECK_ZERO(length(y2)-fabs(y(0)), 1e-3, "");
    }
  }
}

//===========================================================================

#if 0

void TEST(GJK_Jacobians2) {
  rai::Configuration C;
  C.addFrame("base");
  for(uint i=0;i<20;i++){
    rai::Frame *a = C.addFrame(STRING("obj_" <<i), "base");
    a->setJoint(rai::JT_free);
    a->set_Q()->setRandom();
    a->set_Q()->pos.z += 1.;

    a->setConvexMesh({}, {}, .02 + .1*rnd.uni());
    a->setColor({.5,.5,.8,.6});
    a->shape->sscCore().setRandom();
    a->shape->createMeshes();
    a->setContact(1);
  }

  C.gl().drawOptions.drawProxies=true;

  C.stepFcl();
//  C.reportProxies();

  VectorFunction f = [&C](const arr& x) -> arr {
    C.setJointState(x);
    C.stepFcl();
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
    C.stepFcl();

//    checkJacobian(f, q, 1e-4);

    F_qQuaternionNorms qn;
    qn.setFrameIDs(framesToIndices(C.frames));
//    C.reportProxies();

    arr y,J;
    C.kinematicsPenetration(y, J, .05);

    arr y2 = qn.eval(qn.getFrames(C));

    cout <<"total penetration: " <<y(0) <<"  diff:" <<y(0) - y_last <<endl; //" quat-non-normalization=" <<y2(0) <<endl;
    y_last = y(0);
    C.view(false, STRING("t=" <<t <<"  movement along negative contact gradient"));

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
  rai::Shape s1(B1), s2(B2);
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
  s1.createMeshes();
  s1.mesh().C = {.5,.8,.5,.9};
  s2.mesh().C = {.5,.5,.8,.9};
  s2.createMeshes();

  C.gl().drawOptions.drawProxies=true;

  C.stepFcl();
  C.reportProxies();

  arr q = C.getJointState();

  for(uint t=0;t<100;t++){
    C.setJointState(q);
    C.stepFcl();
//    C.stepFcl();
//    K.reportProxies(cout, -1., false);

//    PairCollision collInfo(s1.sscCore(), s2.sscCore(), s1.frame.X, s2.frame.X, s1.size(-1), s2.size(-1));

    F_PairCollision gjk(F_PairCollision::_negScalar);
    FrameL F = {&B1, &B2};
    bool good = checkJacobian(gjk.vf2(F), q, 1e-4);
    if(!good) rndGauss(q, 1e-4, true);

    arr y = gjk.eval(F);

    F_qQuaternionNorms qn;
    arr y2 = qn.eval(C.frames);

    cout <<"contact meassure = " <<y(0) <<endl;
    C.view(false, STRING("t=" <<t <<"  movement along negative contact gradient"));

    q -= 1e-2*y.J() + 1e-2*(~y2*y2.J());
  }

  C.view(true);
}

//===========================================================================
#endif

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
    checkJacobian(dist.vf2({C(1), C(2)}), x, 1e-4);

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
    checkJacobian(dist.vf2(F), x, 1e-4);

    arr V = F(0,0)->getMeshPoints();
    F(0,0)->ensure_X().applyOnPointArray(V);
    arr vel = (F(1,0)->ensure_X().pos - F(0,0)->ensure_X().pos).getArr();
    sweep1.clear();
    sweep1.C = {.7, .9, .7, .3};
    sweep1.V.append(V);
    sweep1.V.append(V+(ones(V.d0)^vel));
    sweep1.makeConvexHull();

    V = F(0,1)->getMeshPoints();
    F(0,1)->ensure_X().applyOnPointArray(V);
    vel = (F(1,1)->ensure_X().pos - F(0,1)->ensure_X().pos).getArr();
    sweep2.clear();
    sweep2.C = {.7, .7, .9, .3};
    sweep2.V.append(V);
    sweep2.V.append(V+(ones(V.d0)^vel));
    sweep2.makeConvexHull();

    C.get_viewer()->clear();
    C.get_viewer()->updateConfiguration(C);
    C.get_viewer()->addDistMarker(dist.x-dist.d1*dist.g1, dist.x-dist.d2*dist.g2, .1);
    C.get_viewer()->add().mesh(sweep1);
    C.get_viewer()->add().mesh(sweep2);
    C.view(true);
  }

}

//===========================================================================

int MAIN(int argc, char** argv){
  rai::initCmdLine(argc, argv);

//  rnd.clockSeed();

//  testGJK_Jacobians();
//  testGJK_Jacobians2();
//  testGJK_Jacobians3();

  testFunctional();
  testSweepingSDFs();

  return 0;
}
