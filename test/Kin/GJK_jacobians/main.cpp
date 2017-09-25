#include <stdlib.h>
#include <GL/gl.h>

#include <Geo/mesh.h>
#include <Gui/opengl.h>
#include <Kin/kin.h>
#include <Kin/taskMaps.h>
#include <Kin/frame.h>
#include <Geo/pairCollide.h>

extern bool orsDrawWires;



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

  //  animateConfiguration(W);
  orsDrawWires=true;
  OpenGL gl;
  gl.add(glStandardScene);
//  gl.add(draw);
//  gl.add(K);


  VectorFunction f = [&K, &s1, &s2](arr& v, arr& J, const arr& x) -> void {
    K.setJointState(x);

    mlr::Mesh *m1, *m2;
    if(s1.type()==mlr::ST_mesh) m1=&s1.mesh(); else m1=&s1.sscCore();
    if(s2.type()==mlr::ST_mesh) m2=&s2.mesh(); else m2=&s2.sscCore();

    PairCollision coll(*m1, *m2, s1.frame.X, s2.frame.X/*, s1.size(3), s2.size(3)*/);

    arr Jp1, Jp2, Jx1, Jx2;
    K.jacobianPos(Jp1, &s1.frame, coll.p1);
    K.jacobianPos(Jp2, &s2.frame, coll.p2);
    K.axesMatrix(Jx1, &s1.frame);
    K.axesMatrix(Jx2, &s2.frame);

    coll.kinVector(v, J, Jp1, Jp2, Jx1, Jx2);

    //reduce by radiipt1==GJK_vertex && pt2==GJK_face
#if 1
    double rad=0.;
    if(s1.type()==mlr::ST_ssCvx) rad += s1.size(3);
    if(s2.type()==mlr::ST_ssCvx) rad += s2.size(3);
    double l2=sumOfSqr(v), l=sqrt(l2);
//    if(s1.type()==mlr::ST_ssCvx) p1 -= s1.size(3)/l*mlr::Vector(v);
//    if(s2.type()==mlr::ST_ssCvx) p2 += s2.size(3)/l*mlr::Vector(v);
    if(rad>0.){
      double fac = (l-rad)/l;
      if(&J){
        arr d_fac = (1.-(l-rad)/l)/l2 *(~v)*J;
        J = J*fac + v*d_fac;
      }
      v *= fac;
    }
//    if(d2>1e-10)
//      CHECK_ZERO(l2-d2, 1e-6,"");
#endif
  };

  TaskMap_GJK gjk(K, "s1", "s2", true);

  for(uint k=0;k<100;k++){
    rndGauss(q, 1.);
    K.setJointState(q);

    PairCollision collInfo(s1.sscCore(), s2.sscCore(), s1.frame.X, s2.frame.X);
    cout <<collInfo;

    arr y,y2,J;
    //test both, the explicit code above as well as the wrapped TaskMap_GJK
    f(y2, NoArr, q);
    checkJacobian(f, q, 1e-4);

    gjk.phi(y, NoArr, K);
    checkJacobian(gjk.vf(K), q, 1e-4);

    cout <<"distances: " <<length(y2) <<' ' <<length(y) <<endl;
//    cout <<"vec=" <<y <<" sqr=" <<sumOfSqr(y) <<" f=" <<y2 <<endl;

    gl.add(collInfo);
    gl.add(K);
    gl.update();
    gl.watch();
    if(collInfo.simplex1.d0==2 && collInfo.simplex2.d0==3){
      gl.watch();
    }

    gl.remove(collInfo);
    gl.remove(K);
  }
}


int MAIN(int argc, char** argv){
  mlr::initCmdLine(argc, argv);

  testGJK_Jacobians();

  return 0;
}
