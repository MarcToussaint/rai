#include <stdlib.h>
#include <GL/gl.h>

#include <Geo/mesh.h>
#include <Gui/opengl.h>
#include <Kin/kin.h>
#include <Kin/frame.h>
#include <Geo/pairCollision.h>
#include <Kin/kin_swift.h>
#include <Kin/F_collisions.h>
#include <Kin/F_qFeatures.h>
#include <Kin/viewer.h>

//===========================================================================

void TEST(GJK_Jacobians) {
  rai::Configuration C;
  rai::Frame base(C), b1(C), B1(C), b2(C), B2(C);
  rai::Joint j1(base, b1), J1(b1, B1), j2(B1, b2), J2(b2, B2);
  rai::Shape s1(B1), s2(B2);
  j1.type = j2.type = rai::JT_free; //trans3;
  j1.frame->insertPreLink(rai::Transformation(0))->set_Q()->addRelativeTranslation(1,1,1);
  j2.frame->insertPreLink(rai::Transformation(0))->set_Q()->addRelativeTranslation(-1,-1,1);
  J1.type = J2.type = rai::JT_free;

  C.calc_q_from_Q();
  arr q = C.getJointState();

  OpenGL gl;
  gl.drawOptions.drawWires=true;
  gl.add(glStandardScene);
//  gl.add(draw);
//  gl.add(K);

  F_PairCollision dist(F_PairCollision::_negScalar);
  F_PairCollision distVec(F_PairCollision::_vector);
  F_PairCollision distNorm(F_PairCollision::_normal);
  F_PairCollision distCenter(F_PairCollision::_center);
  dist.setFrameIDs({B1.ID, B2.ID});
  distVec.setFrameIDs({B1.ID, B2.ID});
  distNorm.setFrameIDs({B1.ID, B2.ID});
  distCenter.setFrameIDs({B1.ID, B2.ID});

  for(uint k=0;k<100;k++){
//    //randomize shapes
    s1.mesh().clear();             s2.mesh().clear();
    s1.sscCore().setRandom();      s2.sscCore().setRandom();
    s1.sscCore().scale(2.);       s2.sscCore().scale(2.);
    s1.mesh().C = {.5,.8,.5,.4};   s2.mesh().C = {.5,.5,.8,.4};
    s1.type() = s2.type() = rai::ST_ssCvx; //ST_mesh;
    s1.size() = ARR(rnd.uni(.01, .3)); s2.size() = ARR(rnd.uni(.01, .3));
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

    arr y,y2,y3;
    dist.__phi(y, NoArr, C);
    cout <<k <<" dist ";
    succ &= checkJacobian(dist.vf(C), q, 1e-5);

    distVec.__phi(y2, NoArr, C);
    cout <<k <<" vec  ";
    succ &= checkJacobian(distVec.vf(C), q, 1e-5);

    distNorm.__phi(y3, NoArr, C);
    cout <<k <<" norm  ";
    succ &= checkJacobian(distNorm.vf(C), q, 1e-5);

    distCenter.__phi(y3, NoArr, C);
    cout <<k <<" center  ";
    succ &= checkJacobian(distCenter.vf(C), q, 1e-5);

    PairCollision collInfo(s1.sscCore(), s2.sscCore(), s1.frame.ensure_X(), s2.frame.ensure_X(), s1.size(-1), s2.size(-1));

    //    cout <<"distance: " <<y <<" vec=" <<y2 <<" error=" <<length(y2)-fabs(y(0)) <<endl;
    if(!succ) cout <<collInfo;

    gl.add(collInfo);
    gl.add(C);
    gl.update(STRING(k), true);
    if(!succ) gl.watch();

    gl.remove(collInfo);
    gl.remove(C);

    if(succ){
      CHECK_ZERO(length(y2)-fabs(y(0)), 1e-3, "");
    }
  }
  gl.clear();
}

//===========================================================================

void TEST(GJK_Jacobians2) {
  rai::Configuration C;
  auto base = C.addFrame("base");
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

  rai::ConfigurationViewer V;
  V.setConfiguration(C, 0, true);

  C.stepSwift();
//  C.reportProxies();
  C.orsDrawProxies=true;

  VectorFunction f = [&C](arr& y, arr& J, const arr& x) -> void {
    C.setJointState(x);
    C.stepSwift();
    C.kinematicsPenetration(y, (!!J?J:NoArr), .05);
  };

//  checkJacobian(f, K.getJointState(), 1e-4);

  arr q = C.getJointState();
//  K.orsDrawProxies=false;
  double y_last=0.;
  for(uint t=0;t<1000;t++){
    C.setJointState(q);
    C.stepSwift();

//    checkJacobian(f, q, 1e-4);

    F_qQuaternionNorms qn;
    qn.setFrameIDs(framesToIndices(C.frames));
//    C.reportProxies();

    arr y,J;
    C.kinematicsPenetration(y, J, .05);

    arr y2, J2;
    qn.__phi(y2, J2, C);

    cout <<"contact meassure = " <<y(0) <<" diff=" <<y(0) - y_last <<" quat-non-normalization=" <<y2(0) <<endl;
    y_last = y(0);
    V.setConfiguration(C, STRING("t=" <<t <<"  movement along negative contact gradient"), false);

    q -= 1e-2*J + 1e-2*(~y2*J2);

    if(y(0)<1e-10) break;

  }

  V.setConfiguration(C, 0, true);
}

//===========================================================================

void TEST(GJK_Jacobians3) {
  rai::Configuration C;
  rai::Frame base(C), B1(C), B2(C);
  rai::Joint J1(base, B1), J2(base, B2);
  rai::Shape s1(B1), s2(B2);
  J1.type = rai::JT_free;
  J2.type = rai::JT_rigid;
//  B1.set_Q()->setRandom();
  B1.set_Q()->pos = {0.,0., 1.05};
  B2.set_Q()->pos = {0.,0., 1.21};
  B1.set_Q()->pos.x += .03;
  B1.set_Q()->pos.y += .03;
//  B1.set_Q()->rot.addX(.01);
  s1.cont=s2.cont = true;
  B1.name = "1"; B2.name="2";

  s1.type() = s2.type() = rai::ST_ssBox;
  s1.size() = {.2, .2, .2, .01 };
  s2.size() = {.2, .2, .2, .01 };
  s1.createMeshes();
  s1.mesh().C = {.5,.8,.5,.9};
  s2.mesh().C = {.5,.5,.8,.9};
  s2.createMeshes();

  rai::ConfigurationViewer V;
  V.setConfiguration(C, 0, true);

  C.stepSwift();
  C.reportProxies();
  C.orsDrawProxies=true;

  arr q = C.getJointState();

  for(uint t=0;t<100;t++){
    C.setJointState(q);
    C.stepSwift();
//    C.stepFcl();
//    K.reportProxies(cout, -1., false);

//    PairCollision collInfo(s1.sscCore(), s2.sscCore(), s1.frame.X, s2.frame.X, s1.size(-1), s2.size(-1));

    F_PairCollision gjk(F_PairCollision::_negScalar);
    gjk.setFrameIDs({1, 2});
    checkJacobian(gjk.vf(C), q, 1e-4);

    arr y,J;
    gjk.__phi(y, J, C);

    F_qQuaternionNorms qn;
    arr y2, J2;
    qn.__phi(y2, J2, C);

    cout <<"contact meassure = " <<y(0) <<endl;
    V.setConfiguration(C, STRING("t=" <<t <<"  movement along negative contact gradient"), false);

    q -= 1e-2*J + 1e-2*(~y2*J2);
  }

  V.setConfiguration(C, 0, true);
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

  OpenGL gl;
  gl.camera.setDefault();
  gl.drawOptions.drawWires=true;
  gl.add(glStandardScene);

  arr x = C.getJointState();
  for(uint t=0;t<10;t++){
    rndGauss(x, .7);
    C.setJointState(x);

//    F_PairCollision dist(F_PairCollision::_negScalar);
    F_PairFunctional dist;
    auto y = dist.eval({C(1), C(2)});
    checkJacobian(dist.vf2({C(1), C(2)}), x, 1e-4);

    gl.add(dist);
    gl.add(C);
    gl.update(STRING(t), true);
    /*if(!succ)*/ gl.watch();

    gl.remove(C);
    gl.remove(dist);

  }
}

//===========================================================================

int MAIN(int argc, char** argv){
  rai::initCmdLine(argc, argv);

  rnd.clockSeed();

//  testGJK_Jacobians();
//  testGJK_Jacobians2();
//  testGJK_Jacobians3();

  testFunctional();

  return 0;
}
