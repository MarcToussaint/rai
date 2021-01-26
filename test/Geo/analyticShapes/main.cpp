#include <stdlib.h>
#include <GL/gl.h>

#include <Geo/analyticShapes.h>
#include <Geo/mesh.h>
#include <Gui/opengl.h>

#include <Optim/newton.h>

//===========================================================================

void TEST(DistanceFunctions) {
  rai::Transformation t;
  t.setRandom();
  rai::Mesh m;
  m.C = {.5, .5, .5, .5};

  struct DrawPair : GLDrawer {
    arr P1, P2;

    void glDraw(OpenGL& gl){
      glColor(0., 1., 0., 1.);
      glDrawDiamond(P1(0), P1(1), P1(2), .05, .05, .05);

      glColor(0., 0., 1., 1.);
      glDrawDiamond(P2(0), P2(1), P2(2), .05, .05, .05);

      glColor(1., 0., 0., 1.);
      glLineWidth(2.f);
      glDrawProxy(P1, P2, .02);
      glLineWidth(1.f);
      glLoadIdentity();
    }
  } pairDrawer;

  OpenGL gl;
  gl.add(glStandardScene,nullptr);
  gl.add(pairDrawer);
  gl.add(m);

  rai::Array<shared_ptr<ScalarFunction>> fcts = {
    make_shared<DistanceFunction_Sphere>(t, 1.),
    make_shared<DistanceFunction_ssBox>(t, 1., 2., 3., 1.),
    make_shared<DistanceFunction_Cylinder>(t, 2., 1.),
    make_shared<DistanceFunction_Capsule>(t, 2., 1.)
  };

  for(shared_ptr<ScalarFunction>& f: fcts){
    m.setImplicitSurfaceBySphereProjection(*f, 3.);

    //-- check hessian and gradient
    for(uint i=0;i<100;i++){
      arr x(3);
      rndUniform(x, -1., 1.);
      bool suc=true;
      suc &= checkGradient(*f, x, 1e-6);
      suc &= checkHessian(*f, x, 1e-6);

      {
        arr g;
        double d = (*f)(g, NoArr, x);
        pairDrawer.P1 = x;
        pairDrawer.P2 = x-d*g;
        gl.update(0, true);
      }

      if(!suc){
        arr g,H;
        (*f)(g,H,x); //set breakpoint here;
        HALT("x=" <<x);
      }
    }

    //-- display
    gl.watch();
  }
}

//===========================================================================

void TEST(DistanceFunctions2) {
  //-- check hessian and gradient
  for(uint i=0;i<100;i++){
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

  //-- display
//  rai::Mesh m;
//  m.setImplicitSurface(DistanceFunction_SSBox,-10.,10.,100); //only works for 3D ScalarFunction
//  OpenGL gl;
//  gl.add(m);
//  gl.watch();

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
  rai::Mesh m;
  OpenGL gl;
  gl.add(glStandardScene,nullptr);
  gl.add(m);

  rai::Array<ScalarFunction*> fcts = {&blobby, &sphere, &torus, &cylinder};

  for(ScalarFunction* f: fcts){
    m.setImplicitSurface(*f,-10.,10.,100);
    gl.watch();
  }
}

//===========================================================================

void projectToSurface(){
  rai::Transformation pose;
  pose.setRandom();

  rai::Array<shared_ptr<ScalarFunction>> fcts = {
    make_shared<DistanceFunction_Sphere>(pose, 1.),
    make_shared<DistanceFunction_ssBox>(pose, 1., 2., 3., .2),
    make_shared<DistanceFunction_Cylinder>(pose, 2., .2),
    make_shared<DistanceFunction_Capsule>(pose, 2., .2)
  };

  rai::Mesh m;
  OpenGL gl;
  gl.drawOptions.drawWires=true;
  gl.add(glStandardScene,nullptr);
  gl.add(m);

  ofstream fil("z.obj");

  for(shared_ptr<ScalarFunction>& fct:fcts){
    for(uint k=0;k<1000;k++){
      arr x = randn(3);
      x += pose.pos.getArr();
      checkGradient(*fct, x, 1e-4);
      checkHessian(*fct, x, 1e-4);
      x.writeRaw(fil);
      fil <<(*fct)(NoArr, NoArr, x) <<endl;
    }

    m.setImplicitSurfaceBySphereProjection(*fct, 10., 3);

    gl.watch();
  }
}

//===========================================================================

int MAIN(int argc, char** argv){
  rai::initCmdLine(argc, argv);

//  testDistanceFunctions();
//  testDistanceFunctions2();
//  testSimpleImplicitSurfaces();

  projectToSurface();

  return 0;
}
