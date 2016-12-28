#include <stdlib.h>
#include <GL/gl.h>

#include <Geo/mesh.h>
#include <Gui/opengl.h>

mlr::Mesh m1, m2;
mlr::Transformation t1, t2;
mlr::Vector p1, p2;

void draw(void*){
  glDisable(GL_DEPTH_TEST);

  glColor(.8, .8, .8, .8);
  glTransform(t1);  m1.glDraw(NoOpenGL);
  glTransform(t2);  m2.glDraw(NoOpenGL);
  glLoadIdentity();

  glColor(1., 0., 0., .9);
  glDrawDiamond(p1.x, p1.y, p1.z, .1, .1, .1);
  glDrawDiamond(p2.x, p2.y, p2.z, .1, .1, .1);
  glBegin(GL_LINES);
  glVertex3f(p1.x, p1.y, p1.z);
  glVertex3f(p2.x, p2.y, p2.z);
  glEnd();
  glLoadIdentity();
}

//===========================================================================

extern bool orsDrawWires;
void TEST(GJK) {
  OpenGL gl;
  gl.add(glStandardScene);
  gl.add(draw, &m2);
  orsDrawWires = true;

  t1.setZero();
  t2.setZero();

  m1.setRandom();  m1.clean();  t1.pos.set(-0., -0., 1.);
  m2.setRandom();  m2.clean();  t2.pos.set( 0., 0., 1.5);

  gl.update();

  for(uint i=0;i<50;i++){
    t1.pos.y += .1; t1.addRelativeRotationDeg(10, 0., 1., 0.);
    t2.pos.y -= .2; t2.addRelativeRotationDeg(10, 1., 0., 0.);

    double d=GJK_sqrDistance(m1, m2, t1, t2, p1, p2, NoVector, NoVector, NoPointType, NoPointType);
    double c_dist=(t1.pos-t2.pos).length();
    cout <<"distance = " <<d <<"\np1=" <<p1 <<"\np2=" <<p2 <<"\ncenter dist=" <<c_dist <<endl;
//    CHECK_LE(d, c_dist+1.,"distance doesn't make sense");
    CHECK_GE(d, c_dist-3.,"distance doesn't make sense");
    gl.timedupdate(.1);
  }
}

//===========================================================================

void TEST(Volume){
  mlr::Mesh m;
  for(uint k=1;k<10;k++){
    cout <<"sphere fineness " <<k <<endl;
    m.setSphere(k);
    double A=m.getArea(), V=m.getVolume();
    cout <<"area  = " <<A<<" error=" <<4.*MLR_PI-A <<endl;
    cout <<"volume= " <<V<<" error=" <<4./3.*MLR_PI-V <<endl;
    cout  <<endl;
  }
  //cout <<"\ncircum=" <<m.getCircum() <<endl;
}

//===========================================================================

void TEST(DistanceFunctions) {
  mlr::Transformation t;
  t.setRandom();
  mlr::Mesh m;
  OpenGL gl;
  gl.add(glStandardScene,NULL);
  gl.add(m);

  mlr::Array<ScalarFunction*> fcts = {
    new DistanceFunction_Sphere(t, 1.),
    new DistanceFunction_Box(t, 1., 2., 3., 1.),
    new DistanceFunction_Cylinder(t, 1., 2.)
  };

  for(ScalarFunction* f: fcts){
    //-- check hessian and gradient
    for(uint i=0;i<1000;i++){
      arr x(3);
      rndUniform(x, -5., 5.);
      bool suc=true;
      suc &= checkGradient(*f, x, 1e-6);
      suc &= checkHessian(*f, x, 1e-6);
      if(!suc){
        arr g,H;
        (*f)(g,H,x); //set breakpoint here;
        HALT("x=" <<x);
      }
    }

    //-- display
    m.setImplicitSurface(*f,-10.,10.,100);
    gl.watch();
  }
}

//===========================================================================

void TEST(DistanceFunctions2) {
  //-- check hessian and gradient
  for(uint i=0;i<1000;i++){
    arr x(14);
    rndUniform(x, -5., 5.);

    bool suc=true;
    suc &= checkGradient(DistanceFunction_SSBox, x, 1e-6);
//    suc &= checkHessian(DistanceFunction_SSBox, x, 1e-6);
    if(!suc){
      arr g,H;
      cout <<"f=" <<DistanceFunction_SSBox(g,H,x); //set breakpoint here;
      HALT("x=" <<x);
    }
  }
}

//===========================================================================
//
// implicit surfaces
//

ScalarFunction blobby = [](arr&,arr&, const arr& X){
    double x=X(0), y=X(1), z=X(2);
    return x*x*x*x - 5*x*x+ y*y*y*y - 5*y*y + z*z*z*z - 5*z*z + 11.8;
  };

ScalarFunction sphere=[](arr&,arr&, const arr& X){
    double x=X(0), y=X(1), z=X(2);
    return (x*x +y*y+z*z)-1.;
  };

ScalarFunction torus = [](arr&,arr&, const arr& X){
    double x=X(0), y=X(1), z=X(2);
    double r=sqrt(x*x + y*y);
    return z*z + (1.-r)*(1.-r) - .1;
  };

//double sigmoid(double x){ return .5*(1.+x/(1.+::fabs(x))); }
double sigmoid(double x){ return 1./(1.+exp(-x)); }

double box(double x,double lo,double hi,double steep=10.){
  //outside=0, inside=2, border=1
  double xa = x-lo; xa*=steep;
  double xb = hi-x; xb*=steep;
  return 2.*(1.-sigmoid(xa)*sigmoid(xb));
}

ScalarFunction cylinder = [](arr&,arr&, const arr& X){
    double x=X(0), y=X(1), z=X(2);
    return x*x + y*y + box(z,-1.,1.) - 1.;
  };

void TEST(SimpleImplicitSurfaces) {
  mlr::Mesh m;
  OpenGL gl;
  gl.add(glStandardScene,NULL);
  gl.add(m);

  mlr::Array<ScalarFunction*> fcts = {&blobby, &sphere, &torus, &cylinder};

  for(ScalarFunction* f: fcts){
    m.setImplicitSurface(*f,-10.,10.,100);
    gl.watch();
  }
}

//===========================================================================

int MAIN(int argc, char** argv){
  mlr::initCmdLine(argc, argv);

  testGJK();
  testVolume();
  testDistanceFunctions();
  testDistanceFunctions2();
  testSimpleImplicitSurfaces();

  return 0;
}
