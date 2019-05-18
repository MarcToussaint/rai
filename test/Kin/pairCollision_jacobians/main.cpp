#include <stdlib.h>
#include <GL/gl.h>

#include <Geo/mesh.h>
#include <Gui/opengl.h>
#include <Kin/kin.h>
#include <Kin/taskMaps.h>
#include <Kin/frame.h>
#include <Geo/pairCollision.h>
#include <Kin/kin_swift.h>
#include <Kin/TM_QuaternionNorms.h>

extern bool orsDrawWires;

//===========================================================================

void TEST(GJK_Jacobians) {
  rai::KinematicWorld K;
  rai::Frame base(K), b1(K), B1(K), b2(K), B2(K);
  rai::Joint j1(base, b1), J1(b1, B1), j2(B1, b2), J2(b2, B2);
  rai::Shape s1(B1), s2(B2);
  j1.type = j2.type = rai::JT_free; //trans3;
  j1.frame->insertPreLink(rai::Transformation(0))->Q.addRelativeTranslation(1,1,1);
  j2.frame->insertPreLink(rai::Transformation(0))->Q.addRelativeTranslation(-1,-1,1);
  J1.type = J2.type = rai::JT_free;
  s1.type() = s2.type() = rai::ST_ssCvx; //ST_mesh;
  s1.size(3) = s2.size(3) = .2;
  s1.sscCore().setRandom();     s2.sscCore().setRandom();
  s1.mesh().C = {.5,.8,.5,.4};  s2.mesh().C = {.5,.5,.8,.4};
  s1.frame.name="s1";           s2.frame.name="s2";

  K.calc_activeSets();
  K.calc_q_from_Q();
  K.calc_fwdPropagateFrames();
  arr q = K.getJointState();

  orsDrawWires=true;
  OpenGL gl;
  gl.add(glStandardScene);
//  gl.add(draw);
//  gl.add(K);

  TM_PairCollision dist(K, "s1", "s2", TM_PairCollision::_negScalar);
  TM_PairCollision distVec(K, "s1", "s2", TM_PairCollision::_vector);
  TM_PairCollision distNorm(K, "s1", "s2", TM_PairCollision::_normal);
  TM_PairCollision distCenter(K, "s1", "s2", TM_PairCollision::_center);

  for(uint k=0;k<100;k++){
    //randomize shapes
    s1.mesh().clear();             s2.mesh().clear();
    s1.sscCore().setRandom();      s2.sscCore().setRandom();
    s1.mesh().C = {.5,.8,.5,.4};   s2.mesh().C = {.5,.5,.8,.4};
    s1.size(3) = rnd.uni(.01, .3); s2.size(3) = rnd.uni(.01, .3);
    if(rnd.uni()<.4) s1.sscCore().setDot();
    if(rnd.uni()<.4) s2.sscCore().setDot();

    //randomize poses
    rndGauss(q, .7);
    K.setJointState(q);

    bool succ = true;

    arr y,y2,y3;
    dist.phi(y, NoArr, K);
    cout <<k <<" dist ";
    succ &= checkJacobian(dist.vf(K), q, 1e-5);

    distVec.phi(y2, NoArr, K);
    cout <<k <<" vec  ";
    succ &= checkJacobian(distVec.vf(K), q, 1e-5);

    distNorm.phi(y3, NoArr, K);
    cout <<k <<" norm  ";
    succ &= checkJacobian(distNorm.vf(K), q, 1e-5);

    distCenter.phi(y3, NoArr, K);
    cout <<k <<" center  ";
    succ &= checkJacobian(distCenter.vf(K), q, 1e-5);

    PairCollision collInfo(s1.sscCore(), s2.sscCore(), s1.frame.X, s2.frame.X, s1.size(3), s2.size(3));

    //    cout <<"distance: " <<y <<" vec=" <<y2 <<" error=" <<length(y2)-fabs(y(0)) <<endl;
    if(!succ) cout <<collInfo;

    gl.add(collInfo);
    gl.add(K);
    gl.update();
    if(!succ) gl.watch();

    gl.remove(collInfo);
    gl.remove(K);

    if(succ){
      CHECK_ZERO(length(y2)-fabs(y(0)), 1e-3, "");
    }
  }
}

//===========================================================================

void TEST(GJK_Jacobians2) {
  rai::KinematicWorld K;
  rai::Frame base(K);
  for(uint i=0;i<20;i++){
    rai::Frame *a = new rai::Frame(K);
    a->name <<"obj_" <<i;

    rai::Joint *j = new rai::Joint(base, *a);
    j->setType(rai::JT_free);
    j->frame->Q.setRandom();
    j->frame->Q.pos.z += 1.;

    rai::Shape *s = new rai::Shape(*a);
    s->cont=true;
    s->type() = rai::ST_ssCvx; //ST_mesh;
    s->size(3) = .02 + .1*rnd.uni();
    s->sscCore().setRandom();
    s->mesh().C = {.5,.5,.8,.6};
  }
  K.calc_activeSets();
  K.calc_fwdPropagateFrames();

  K.watch();

  K.swift().initActivations(K);
  K.stepSwift();
//  K.reportProxies();
  K.orsDrawProxies=true;

  VectorFunction f = [&K](arr& y, arr& J, const arr& x) -> void {
    K.setJointState(x);
    K.stepSwift();
    K.kinematicsProxyCost(y, (!!J?J:NoArr), .2);
//    K.filterProxiesToContacts(.25);
//    K.kinematicsContactCost(y, (!!J?J:NoArr), .2);
  };

//  checkJacobian(f, K.getJointState(), 1e-4);

  arr q = K.getJointState();
//  K.orsDrawProxies=false;
//  K.(true);
  double y_last=0.;
  for(uint t=0;t<1000;t++){
    K.setJointState(q);
    K.stepSwift();

//    checkJacobian(f, q, 1e-4);

    TM_QuaternionNorms qn;
//    K.reportProxies();

    arr y,J;
//    K.filterProxiesToContacts(.25);
//    K.kinematicsContactCost(y, J, .2);
    K.kinematicsProxyCost(y, J, .2);

    arr y2, J2;
    qn.phi(y2, J2, K);

    cout <<"contact meassure = " <<y(0) <<" diff=" <<y(0) - y_last <<" quat-non-normalization=" <<y2(0) <<endl;
    y_last = y(0);
    K.watch(false, STRING("t=" <<t <<"  movement along negative contact gradient"));

    q -= 1e-2*J + 1e-2*(~y2*J2);

    if(y(0)<1e-10) break;

  }

  K.watch(true);
}

//===========================================================================

void TEST(GJK_Jacobians3) {
  rai::KinematicWorld K;
  rai::Frame base(K), B1(K), B2(K);
  rai::Joint J1(base, B1), J2(base, B2);
  rai::Shape s1(B1), s2(B2);
  J1.type = rai::JT_free;
  J2.type = rai::JT_rigid;
//  B1.Q.setRandom();
  B1.Q.pos = {0.,0., 1.05};
  B2.Q.pos = {0.,0., 1.21};
  B1.Q.pos.x += .03;
  B1.Q.pos.y += .03;
//  B1.Q.rot.addX(.01);
  s1.cont=s2.cont = true;
  B1.name = "1"; B2.name="2";

  s1.type() = s2.type() = rai::ST_ssBox;
  s1.size() = {.2, .2, .2, .01 };
  s2.size() = {.2, .2, .2, .01 };
  s1.createMeshes();
  s1.mesh().C = {.5,.8,.5,.9};
  s2.mesh().C = {.5,.5,.8,.9};

  K.calc_activeSets();
  K.calc_fwdPropagateFrames();

  K.watch();

  K.swift().initActivations(K);
  K.stepSwift();
  K.reportProxies();
  K.orsDrawProxies=true;

  arr q = K.getJointState();

  for(uint t=0;t<100;t++){
    K.setJointState(q);
    K.stepSwift();
//    K.reportProxies(cout, -1., false);

//    PairCollision collInfo(s1.sscCore(), s2.sscCore(), s1.frame.X, s2.frame.X, s1.size(3), s2.size(3));

    TM_PairCollision gjk(1, 2, TM_PairCollision::_negScalar);
    checkJacobian(gjk.vf(K), q, 1e-4);

    arr y,J;
    gjk.phi(y, J, K);

    TM_QuaternionNorms qn;
    arr y2, J2;
    qn.phi(y2, J2, K);

    cout <<"contact meassure = " <<y(0) <<endl;
    K.watch(false, STRING("t=" <<t <<"  movement along negative contact gradient"));

    q -= 1e-2*J + 1e-2*(~y2*J2);
  }

  K.watch(true);
}

//===========================================================================

int MAIN(int argc, char** argv){
  rai::initCmdLine(argc, argv);

  rnd.clockSeed();

  testGJK_Jacobians();
  testGJK_Jacobians2();
  testGJK_Jacobians3();

  return 0;
}
