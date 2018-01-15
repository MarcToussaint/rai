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
  mlr::KinematicWorld K;
  mlr::Frame base(K), b1(K), B1(K), b2(K), B2(K);
  mlr::Joint j1(base, b1), J1(b1, B1), j2(B1, b2), J2(b2, B2);
  mlr::Shape s1(B1), s2(B2);
  j1.type = j2.type = mlr::JT_trans3;
  j1.frame.insertPreLink(mlr::Transformation(0))->Q.addRelativeTranslation(1,1,1);
  j2.frame.insertPreLink(mlr::Transformation(0))->Q.addRelativeTranslation(-1,-1,1);
  J1.type = J2.type = mlr::JT_quatBall;
  s1.type() = s2.type() = mlr::ST_ssCvx; //ST_mesh;
  s1.size(3) = s2.size(3) = .2;
  s1.sscCore().setRandom(); s1.mesh().C = {.5,.8,.5,.4};
  s2.sscCore().setRandom(); s2.mesh().C = {.5,.5,.8,.4};
  s1.frame.name="s1";
  s2.frame.name="s2";

  K.calc_activeSets();
  K.calc_q_from_Q();
  K.calc_fwdPropagateFrames();
  arr q = K.getJointState();

  orsDrawWires=true;
  OpenGL gl;
  gl.add(glStandardScene);
//  gl.add(draw);
//  gl.add(K);

  TM_PairCollision dist(K, "s1", "s2", true);
  TM_PairCollision distVec(K, "s1", "s2", false);

  for(uint k=0;k<100;k++){
    rndGauss(q, 1.);
    K.setJointState(q);

    PairCollision collInfo(s1.sscCore(), s2.sscCore(), s1.frame.X, s2.frame.X, s1.size(3), s2.size(3));

    arr y,y2;
    dist.phi(y, NoArr, K);
    checkJacobian(dist.vf(K), q, 1e-4);

    distVec.phi(y2, NoArr, K);
    checkJacobian(dist.vf(K), q, 1e-4);

//    cout <<"distance: " <<y <<" vec=" <<y2 <<" error=" <<length(y2)-fabs(y(0)) <<endl;
    CHECK_ZERO(length(y2)-fabs(y(0)), 1e-3, "");

    gl.add(collInfo);
    gl.add(K);
    gl.update();
//    gl.watch();

    gl.remove(collInfo);
    gl.remove(K);
  }
}

//===========================================================================

void TEST(GJK_Jacobians2) {
  mlr::KinematicWorld K;
  mlr::Frame base(K);
  for(uint i=0;i<20;i++){
    mlr::Frame *a = new mlr::Frame(K);
    a->name <<"obj_" <<i;

    mlr::Joint *j = new mlr::Joint(base, *a);
    j->type = mlr::JT_free;
    j->frame.Q.setRandom();
    j->frame.Q.pos.z += 1.;

    mlr::Shape *s = new mlr::Shape(*a);
    s->cont=true;
    s->type() = mlr::ST_ssCvx; //ST_mesh;
    s->size(3) = .02 + .1*rnd.uni();
    s->sscCore().setRandom();
    s->mesh().C = {.5,.5,.8,.6};
  }
  K.calc_activeSets();
  K.calc_fwdPropagateFrames();

  K.gl().update();

  K.swift().initActivations(K, 0);
  K.stepSwift();
//  K.reportProxies();
  K.orsDrawProxies=true;

  VectorFunction f = [&K](arr& y, arr& J, const arr& x) -> void {
    K.setJointState(x);
    K.stepSwift();
//    K.kinematicsProxyCost(y, (&J?J:NoArr), .2);
    K.filterProxiesToContacts(.25);
    K.kinematicsContactCost(y, (&J?J:NoArr), .2);
  };

//  checkJacobian(f, K.getJointState(), 1e-4);


  arr q = K.getJointState();
//  K.orsDrawProxies=false;
//  K.
  (true);
  double y_last=0.;
  for(uint t=0;t<1000;t++){
    K.setJointState(q);
    K.stepSwift();

//    checkJacobian(f, q, 1e-4);

    TM_QuaternionNorms qn;
//    K.reportProxies();

    arr y,J;
    K.filterProxiesToContacts(.25);
    K.kinematicsContactCost(y, (&J?J:NoArr), .2);
//    K.kinematicsProxyCost(y, J, .2);

    arr y2, J2;
    qn.phi(y2, J2, K);

    cout <<"contact meassure = " <<y(0) <<" diff=" <<y(0) - y_last <<" quat-non-normalization=" <<y2(0) <<endl;
    y_last = y(0);
    K.watch(false, STRING("t=" <<t <<"  movement along negative contact gradient"));

    q -= 1e-3*J + 1e-2*(~y2*J2);

    if(y(0)<1e-10) break;

  }

//  K.gl().watch();
}

//===========================================================================

void TEST(GJK_Jacobians3) {
  mlr::KinematicWorld K;
  mlr::Frame base(K), B1(K), B2(K);
  mlr::Joint J1(base, B1), J2(base, B2);
  mlr::Shape s1(B1), s2(B2);
  J1.type = mlr::JT_free;
  J2.type = mlr::JT_rigid;
//  B1.Q.setRandom();
  B1.Q.pos = {0.,0., 1.05};
  B2.Q.pos = {0.,0., 1.21};
  B1.Q.pos.x += .03;
  B1.Q.pos.y += .03;
//  B1.Q.rot.addX(.01);
  s1.cont=s2.cont = true;
  B1.name = "1"; B2.name="2";

  s1.type() = s2.type() = mlr::ST_ssBox;
  s1.size() = {.2, .2, .2, .01 };
  s2.size() = {.2, .2, .2, .01 };
  s1.getGeom().createMeshes();
  s1.mesh().C = {.5,.8,.5,.9};
  s2.mesh().C = {.5,.5,.8,.9
};

  K.calc_activeSets();
  K.calc_fwdPropagateFrames();

  K.gl().update();

  K.swift().initActivations(K, 0);
  K.stepSwift();
  K.reportProxies();
  K.orsDrawProxies=true;

  arr q = K.getJointState();

  double y_last=0.;
  for(uint t=0;t<1000;t++){
    K.setJointState(q);
    K.stepSwift();
//    K.reportProxies(cout, -1., false);


    PairCollision collInfo(s1.sscCore(), s2.sscCore(), s1.frame.X, s2.frame.X, s1.size(3), s2.size(3));

    TM_PairCollision gjk(1, 2, true);
    checkJacobian(gjk.vf(K), q, 1e-4);

    arr y,J;
    gjk.phi(y, J, K);

    TM_QuaternionNorms qn;
    arr y2, J2;
    qn.phi(y2, J2, K);

    cout <<"contact meassure = " <</*y_last - */y(0) <<endl;
    y_last = y(0);
    K.watch(false, STRING("t=" <<t <<"  movement along negative contact gradient"));

    q -= 1e-2*J + 1e-2*(~y2*J2);
  }

  K.gl().watch();
}

//===========================================================================

int MAIN(int argc, char** argv){
  mlr::initCmdLine(argc, argv);

  rnd.clockSeed();

  testGJK_Jacobians();
  testGJK_Jacobians2();
  testGJK_Jacobians3();

  return 0;
}
