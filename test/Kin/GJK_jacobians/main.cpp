#include <stdlib.h>
#include <GL/gl.h>

#include <Geo/mesh.h>
#include <Gui/opengl.h>
#include <Kin/kin.h>
#include <Kin/taskMaps.h>
#include <Kin/frame.h>

mlr::Vector p1, p2;
mlr::Vector e1, e2;
GJK_point_type pt1, pt2;

void draw(void*){
  glColor(0., 1., 0., 1.);
  glDrawDiamond(p1.x, p1.y, p1.z, .1, .1, .1);
  glColor(0., 0., 1., 1.);
  glDrawDiamond(p2.x, p2.y, p2.z, .1, .1, .1);
  glColor(1., 0., 0., 1.);
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

#include <ccd/ccd.h>
#include <ccd/quat.h> // for work with quaternions

void support_mesh(const void *_obj, const ccd_vec3_t *_dir, ccd_vec3_t *v){
  mlr::Mesh *m = (mlr::Mesh*)_obj;
  arr dir(_dir->v, 3, true);
  uint vertex = m->support(dir);
  memmove(v->v, &m->V(vertex, 0), 3*m->V.sizeT);
}

double GJK_libccd_penetration(arr& dir, arr& pos, const mlr::Mesh& m1, const mlr::Mesh& m2){
  ccd_t ccd;
  CCD_INIT(&ccd); // initialize ccd_t struct

  // set up ccd_t struct
  ccd.support1       = support_mesh; // support function for first object
  ccd.support2       = support_mesh; // support function for second object
  ccd.max_iterations = 100;     // maximal number of iterations
  ccd.epa_tolerance  = 0.0001;  // maximal tolerance fro EPA part

  ccd_real_t depth;
  ccd_vec3_t _dir, _pos;

  //    S.m1.translate(randn(3));
  //    int intersect = ccdGJKIntersect(&S.m1, &S.m2, &ccd, &v1, &v2);
  int non_intersect = ccdGJKPenetration(&m1, &m2, &ccd, &depth, &_dir, &_pos);
  //    int intersect = ccdMPRIntersect(&S.m1, &S.m2, &ccd);
//  int non_intersect = ccdMPRPenetration(&m1, &m2, &ccd, &depth, &_dir, &_pos);
  if(non_intersect){ dir.clear(); pos.clear(); return 0.; }

  dir.setCarray(_dir.v, 3);
  pos.setCarray(_pos.v, 3);
  return depth;
}


void TEST(GJK_Jacobians) {
  mlr::KinematicWorld K;
  mlr::Frame base(K), b1(K), B1(K), b2(K), B2(K);
  mlr::Joint j1(base, b1), J1(b1, B1), j2(B1, b2), J2(b2, B2);
  mlr::Shape s1(B1), s2(B2);
  j1.type = j2.type = mlr::JT_trans3;
  j1.frame.insertPreLink(mlr::Transformation(0))->Q.addRelativeTranslation(1,1,1);
  j2.frame.insertPreLink(mlr::Transformation(0))->Q.addRelativeTranslation(-1,-1,1);
  J1.type = J2.type = mlr::JT_rigid; //quatBall;
  s1.type = s2.type = mlr::ST_mesh;
//  s1.size(3) = .5;  s2.size(3) = .5;
  s1.mesh.setRandom(); s1.mesh.C = {.5,.8,.5,.7};
  s2.mesh.setRandom(); s2.mesh.C = {.5,.5,.8,.7};
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
  gl.add(draw);
  gl.add(K);


  VectorFunction f = [&K, &s1, &s2](arr& v, arr& J, const arr& x) -> void {
    K.setJointState(x);

    mlr::Mesh *m1, *m2;
    if(s1.type==mlr::ST_mesh) m1=&s1.mesh; else m1=&s1.sscCore;
    if(s2.type==mlr::ST_mesh) m2=&s2.mesh; else m2=&s2.sscCore;

    double d2 = GJK_sqrDistance(*m1, *m2, s1.frame.X, s2.frame.X, p1, p2, e1, e2, pt1, pt2);
    if(&J) cout <<"point types= " <<pt1 <<' ' <<pt2 <<endl;
//    if(fabs(d2)<1e-10) LOG(-1) <<"zero distance";
    arr y1, J1, y2, J2;

    if(d2<1e-10){
      arr dir, pos;
      mlr::Mesh M1(*m1); s1.frame.X.applyOnPointArray(M1.V);
      mlr::Mesh M2(*m2); s2.frame.X.applyOnPointArray(M2.V);
      double penetration = GJK_libccd_penetration(dir, pos, M1, M2);
      if(&J) cout <<"penetration=" <<penetration <<endl;
      mlr::Vector cen = pos; //.5*(p1+p2);
      p1 = cen + .5*penetration*mlr::Vector(dir);
      p2 = cen - .5*penetration*mlr::Vector(dir);
      pt1=GJK_vertex; pt2=GJK_face;
    }

    K.kinematicsPos(y1, (&J?J1:NoArr), &s1.frame, s1.frame.X.rot/(p1-s1.frame.X.pos));
    K.kinematicsPos(y2, (&J?J2:NoArr), &s2.frame, s2.frame.X.rot/(p2-s2.frame.X.pos));

//    cout <<p1 <<y1 <<p2 <<y2 <<endl;
    v = y1 - y2;
    if(&J){
      J = J1 - J2;
      if((pt1==GJK_vertex && pt2==GJK_face) || (pt1==GJK_face && pt2==GJK_vertex)){
        arr vec, Jv, n = v/length(v);
        J = n*(~n*J);
        if(pt1==GJK_vertex) K.kinematicsVec(vec, Jv, &s2.frame, s2.frame.X.rot/(p1-p2));
        if(pt2==GJK_vertex) K.kinematicsVec(vec, Jv, &s1.frame, s1.frame.X.rot/(p1-p2));
        J += Jv;
      }
      if(pt1==GJK_edge && pt2==GJK_edge){
        arr vec, Jv, n, a, b;
        n = v/length(v);
        J = n*(~n*J);

        K.kinematicsVec(vec, Jv, &s1.frame, s1.frame.X.rot/e1);
        a=conv_vec2arr(e1);
        b=conv_vec2arr(e2);
        double ab=scalarProduct(a,b);
        J += (a-b*ab) * (1./(1.-ab*ab)) * (~v*(b*~b -eye(3,3))) * Jv;

        K.kinematicsVec(vec, Jv, &s2.frame, s2.frame.X.rot/e2);
        a=conv_vec2arr(e2);
        b=conv_vec2arr(e1);
        J += (a-b*ab) * (1./(1.-ab*ab)) * (~v*(b*~b -eye(3,3))) * Jv;
      }
      if((pt1==GJK_vertex && pt2==GJK_edge) || (pt1==GJK_edge && pt2==GJK_vertex)){
        arr vec, Jv, n;
        if(pt1==GJK_vertex) n=conv_vec2arr(e2); else n=conv_vec2arr(e1);
        J = J - n*(~n*J);
        if(pt1==GJK_vertex) K.kinematicsVec(vec, Jv, &s2.frame, s2.frame.X.rot/(p1-p2));
        if(pt2==GJK_vertex) K.kinematicsVec(vec, Jv, &s1.frame, s1.frame.X.rot/(p1-p2));
        J += n*(~n*Jv);
      }
    }
    //reduce by radiipt1==GJK_vertex && pt2==GJK_face
    double rad=0.;
    if(s1.type==mlr::ST_ssCvx) rad += s1.size(3);
    if(s2.type==mlr::ST_ssCvx) rad += s2.size(3);
    double l2=sumOfSqr(v), l=sqrt(l2);
    if(s1.type==mlr::ST_ssCvx) p1 -= s1.size(3)/l*mlr::Vector(v);
    if(s2.type==mlr::ST_ssCvx) p2 += s2.size(3)/l*mlr::Vector(v);
    if(rad>0.){
      double fac = (l-rad)/l;
      if(&J){
        arr d_fac = (1.-(l-rad)/l)/l2 *(~v)*J;
        J = J*fac + v*d_fac;
      }
      v *= fac;
    }
    if(d2>1e-10)
      CHECK_ZERO(l2-d2, 1e-6,"");
  };

  TaskMap_GJK gjk(K, "s1", "s2", true);

  for(uint k=0;k<30;k++){
    rndGauss(q, 1.);
    K.setJointState(q);

    arr y,y2,J;
    //test both, the explicit code above as well as the wrapped TaskMap_GJK
    f(y2, NoArr, q);
    checkJacobian(f, q, 1e-4);
    gjk.phi(y, NoArr, K);
    f(y2, NoArr, q);
    checkJacobian(gjk.vf(K), q, 1e-4);
    cout <<"vec=" <<y <<" sqr=" <<sumOfSqr(y) <<" f=" <<y2 <<endl;

    gl.update();
    gl.watch();
  }
}


int MAIN(int argc, char** argv){
  mlr::initCmdLine(argc, argv);

  testGJK_Jacobians();

  return 0;
}
