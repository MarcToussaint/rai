#include <stdlib.h>
#include <GL/gl.h>

#include <Geo/mesh.h>
#include <Gui/opengl.h>
#include <Kin/kin.h>
#include <Kin/taskMaps.h>

mlr::Vector p1, p2;
mlr::Vector e1, e2;
GJK_point_type pt1, pt2;

void draw(void*){
  glDisable(GL_DEPTH_TEST);

  glColor(1., 0., 0., .9);
  glDrawDiamond(p1.x, p1.y, p1.z, .1, .1, .1);
  glDrawDiamond(p2.x, p2.y, p2.z, .1, .1, .1);
  glBegin(GL_LINES);
  glVertex3f(p1.x, p1.y, p1.z);
  glVertex3f(p2.x, p2.y, p2.z);
  glEnd();
  glBegin(GL_LINES);
  glVertex3f(p1.x, p1.y, p1.z);
  glVertex3f(p1.x+e1.x, p1.y+e1.y, p1.z+e1.z);
  glEnd();
  glBegin(GL_LINES);
  glVertex3f(p2.x, p2.y, p2.z);
  glVertex3f(p2.x+e2.x, p2.y+e2.y, p2.z+e2.z);
  glEnd();
  glLoadIdentity();
}

extern bool orsDrawWires;

void TEST(GJK_Jacobians) {
  mlr::KinematicWorld W;
  mlr::Body base(W), b1(W), B1(W), b2(W), B2(W);
  mlr::Joint j1(W, &base, &b1), J1(W, &b1, &B1), j2(W, &base, &b2), J2(W, &b2, &B2);
  mlr::Shape s1(W, B1), s2(W, B2);
  j1.type = j2.type = mlr::JT_trans3;
  j1.A.addRelativeTranslation(1,1,1);
  j2.A.addRelativeTranslation(-1,-1,1);
  J1.type = J2.type = mlr::JT_quatBall;
  s1.type = s2.type = mlr::ST_ssCvx;
  s1.size(3) = .5;  s2.size(3) = .5;
  s1.sscCore.setRandom();
  s2.sscCore.setRandom();
  s1.name="s1";
  s2.name="s2";


  W.calc_fwdPropagateFrames();
  arr q = W.getJointState();

  //  animateConfiguration(W);
  orsDrawWires=true;
  W.gl().add(draw);

  VectorFunction f = [&W, &s1, &s2](arr& v, arr& J, const arr& x) -> void {
    W.setJointState(x);

    double d2 = GJK_sqrDistance(s1.sscCore, s2.sscCore, s1.X, s2.X, p1, p2, e1, e2, pt1, pt2);
    if(&J) cout <<"point types= " <<pt1 <<' ' <<pt2 <<endl;
    if(d2<1e-10) LOG(-1) <<"zero distance";
    arr y1, J1, y2, J2;

    W.kinematicsPos(y1, (&J?J1:NoArr), s1.body, s1.body->X.rot/(p1-s1.body->X.pos));
    W.kinematicsPos(y2, (&J?J2:NoArr), s2.body, s2.body->X.rot/(p2-s2.body->X.pos));

    v = y1 - y2;
    if(&J){
      J = J1 - J2;
      if((pt1==GJK_vertex && pt2==GJK_face) || (pt1==GJK_face && pt2==GJK_vertex)){
        arr vec, Jv, n = v/length(v);
        J = n*(~n*J);
        if(pt1==GJK_vertex) W.kinematicsVec(vec, Jv, s2.body, s2.body->X.rot/(p1-p2));
        if(pt2==GJK_vertex) W.kinematicsVec(vec, Jv, s1.body, s1.body->X.rot/(p1-p2));
        J += Jv;
      }
      if(pt1==GJK_edge && pt2==GJK_edge){
        arr vec, Jv, n, a, b;
        n = v/length(v);
        J = n*(~n*J);

        W.kinematicsVec(vec, Jv, s1.body, s1.body->X.rot/e1);
        a=conv_vec2arr(e1);
        b=conv_vec2arr(e2);
        double ab=scalarProduct(a,b);
        J += (a-b*ab) * (1./(1.-ab*ab)) * (~v*(b*~b -eye(3,3))) * Jv;

        W.kinematicsVec(vec, Jv, s2.body, s2.body->X.rot/e2);
        a=conv_vec2arr(e2);
        b=conv_vec2arr(e1);
        J += (a-b*ab) * (1./(1.-ab*ab)) * (~v*(b*~b -eye(3,3))) * Jv;
      }
      if((pt1==GJK_vertex && pt2==GJK_edge) || (pt1==GJK_edge && pt2==GJK_vertex)){
        arr vec, Jv, n;
        if(pt1==GJK_vertex) n=conv_vec2arr(e2); else n=conv_vec2arr(e1);
        J = J - n*(~n*J);
        if(pt1==GJK_vertex) W.kinematicsVec(vec, Jv, s2.body, s2.body->X.rot/(p1-p2));
        if(pt2==GJK_vertex) W.kinematicsVec(vec, Jv, s1.body, s1.body->X.rot/(p1-p2));
        J += n*(~n*Jv);
      }
    }
    //reduce by radii
    double l2=sumOfSqr(v), l=sqrt(l2);
    p1 -= s1.size(3)/l*mlr::Vector(v);
    p2 += s2.size(3)/l*mlr::Vector(v);
    double fac = (l-s1.size(3)-s2.size(3))/l;
    if(&J){
      arr d_fac = (1.-(l-s1.size(3)-s2.size(3))/l)/l2 *(~v)*J;
      J = J*fac + v*d_fac;
    }
    v *= fac;
    CHECK_ZERO(l2-d2, 1e-6,"");
  };

  TaskMap_GJK gjk(W, "s1", "s2", true);

  for(uint k=0;k<30;k++){
    rndGauss(q, .3);

    arr y,J;
    //    f(y, J, q);
    checkJacobian(f, q, 1e-4);

//    checkJacobian(gjk.vf(W), q, 1e-4);

    W.gl().update();
  }


}


int MAIN(int argc, char** argv){
  mlr::initCmdLine(argc, argv);

  testGJK_Jacobians();

  return 0;
}
