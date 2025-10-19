#include <Geo/mesh.h>
#include <Gui/opengl.h>
#include <Gui/RenderData.h>
#include <Geo/qhull.h>
#include <Geo/signedDistanceFunctions.h>

#include <math.h>

//===========================================================================

void TEST(Primitives) {
  rai::Mesh mesh;

  OpenGL gl;
  gl.data().addStandardScene();

  mesh.setSphere(2);
  cout <<"#V=" <<mesh.V.d0 <<endl;
  gl.data().add().mesh(mesh);
  gl.update(true);

  mesh.setSSBox(3,3,3,1,2);
  cout <<"#V=" <<mesh.V.d0 <<endl;
  gl.data().clear().addStandardScene().add().mesh(mesh);
  gl.update(true);

  mesh.setCapsule(1,3,2);
  cout <<"#V=" <<mesh.V.d0 <<endl;
  gl.data().clear().addStandardScene().add().mesh(mesh);
  gl.update(true);
}

//===========================================================================

void TEST(Volume){
  rai::Mesh m;
  for(uint k=1;k<8;k++){
    cout <<"sphere fineness " <<k <<endl;
    m.setSphere(k);
    double A=m.getArea(), V=m.getVolume();
    cout <<"area  = " <<A<<" error=" <<4.*RAI_PI-A <<endl;
    cout <<"volume= " <<V<<" error=" <<4./3.*RAI_PI-V <<endl;
    cout  <<endl;
  }
  //cout <<"\ncircum=" <<m.getCircum() <<endl;
}

//===========================================================================

void TEST(DistanceFunctions) {
  rai::Transformation t;
  t.setRandom();
  rai::Mesh m;
  OpenGL gl;

  rai::Array<shared_ptr<SDF>> fcts = {
                                      make_shared<SDF_Sphere>(t, 1.),
      make_shared<SDF_ssBox>(t, arr{1., 2., 3.}, 1.),
      make_shared<SDF_Cylinder>(t, 2., 1.)
  };

  for(shared_ptr<SDF>& f: fcts){
    //-- check hessian and gradient
    for(uint i=0;i<100;i++){
      arr x(3);
      rndUniform(x, -5., 5.);
      bool suc=true;
      suc &= checkGradient(f->f_scalar(), x, 1e-6);
      suc &= checkHessian(f->f_scalar(), x, 1e-6);
      if(!suc){
        arr g,H;
        f->f(g,H,x); //set breakpoint here;
        HALT("x=" <<x);
      }
    }

    //-- display
    m.setImplicitSurface(f->f_scalar(),-10.,10.,100);
    gl.data().clear().addStandardScene().add().mesh(m);
    gl.update(true);
  }
}

//===========================================================================
//
// implicit surfaces
//

double blobby(arr&,arr&,const arr& X){
    double x=X(0), y=X(1), z=X(2);
    return x*x*x*x - 5*x*x+ y*y*y*y - 5*y*y + z*z*z*z - 5*z*z + 11.8;
  };

double sphere(arr&,arr&,const arr& X){
    double x=X(0), y=X(1), z=X(2);
    return (x*x +y*y+z*z)-1.;
  };

double torus(arr&,arr&,const arr& X){
    double x=X(0), y=X(1), z=X(2);
    double r = sqrt(x*x + y*y);
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

double cylinder(arr&,arr&,const arr& X){
    double x=X(0), y=X(1), z=X(2);
    return x*x + y*y + box(z,-1.,1.) - 1.;
  };

void TEST(SimpleImplicitSurfaces) {
  rai::Mesh m;
  OpenGL gl;

  auto fcts = {&blobby, &sphere, &torus, &cylinder};

  for(auto* f: fcts){
    m.setImplicitSurface(*f, -10.,10.,100);
    gl.data().clear().addStandardScene().add().mesh(m);
    gl.update(true);
  }
}

//===========================================================================

int MAIN(int argc, char** argv){
  rai::initCmdLine(argc, argv);

  testPrimitives();
  testVolume();
  testDistanceFunctions();
  testSimpleImplicitSurfaces();

  return 0;
}
