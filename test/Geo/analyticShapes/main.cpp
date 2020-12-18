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
  OpenGL gl;
  gl.add(glStandardScene,nullptr);
  gl.add(m);

  rai::Array<shared_ptr<ScalarFunction>> fcts = {
    make_shared<DistanceFunction_Sphere>(t, 1.),
    make_shared<DistanceFunction_ssBox>(t, 1., 2., 3., 1.),
    make_shared<DistanceFunction_Cylinder>(t, 1., 2.),
    make_shared<DistanceFunction_Capsule>(t, 1., 2.)
  };

  for(shared_ptr<ScalarFunction>& f: fcts){
    //-- check hessian and gradient
    for(uint i=0;i<100;i++){
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
    make_shared<DistanceFunction_ssBox>(pose, 1., 2., 3., 1.),
    make_shared<DistanceFunction_Cylinder>(pose, 1., 2.),
    make_shared<DistanceFunction_Capsule>(pose, 1., 2.)
  };


  ScalarFunction f = [&fcts](arr& g, arr& H, const arr& x){
    double d = (*fcts(1))(g, H, x);
    H *= 2.*d;
    H += 2.*(g^g);
    g *= 2.*d;
    return d*d;
  };

  arr X = randn(1000,3);
  for(uint i=0;i<X.d0;i++) X[i] *= 10./length(X[i]);

  for(uint i=0;i<X.d0;i++){
    cout <<i <<'-' <<flush;
    arr x = X[i];
    //    cout <<"initial x: " <<x <<" f(x): " <<(*fcts(0))(NoArr, NoArr, x) <<endl;
    OptNewton newton(x, f);
    newton.options
        .set_verbose(0)
        .set_maxStep(10.)
        .set_damping(1e-10);
    newton.run();
    //  cout <<"final x: " <<x <<" f(x): " <<(*fcts(0))(NoArr, NoArr, x) <<endl;
  }
  cout <<endl;

  rai::Mesh m;
  m.V=X;
  m.makeConvexHull();

  OpenGL gl;
  gl.drawOptions.drawWires=true;
  gl.add(glStandardScene,nullptr);
  gl.add(m);
  gl.watch();



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
