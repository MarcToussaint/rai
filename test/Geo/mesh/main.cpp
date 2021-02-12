#include <stdlib.h>
#include <GL/gl.h>

#include <Geo/mesh.h>
#include <Gui/opengl.h>
#include <Geo/qhull.h>
#include <Geo/analyticShapes.h>

void drawInit(void*, OpenGL& gl){
  glStandardLight(nullptr, gl);
  glDrawAxes(1.);
  glColor(1.,.5,0.);
}

//===========================================================================

void TEST(Primitives) {
  rai::Mesh mesh;

  OpenGL gl;
  gl.add(drawInit,0);
  gl.add(mesh);
  gl.drawOptions.drawWires=true;

  mesh.setSphere(2);
  cout <<"#V=" <<mesh.V.d0 <<endl;
  gl.watch();

  mesh.setSSBox(3,3,3,1,2);
  cout <<"#V=" <<mesh.V.d0 <<endl;
  gl.watch();

  mesh.setCapsule(1,3,2);
  cout <<"#V=" <<mesh.V.d0 <<endl;
  gl.watch();
}

//===========================================================================

void TEST(FuseVertices) {
  if(!rai::FileToken("../../../../rai-robotModels/pr2/head_v0/head_pan.stl", false).exists()) return;

  OpenGL gl;
  rai::Mesh mesh;
  mesh.readFile("../../../../rai-robotModels/pr2/head_v0/head_pan.stl");
  gl.add(drawInit,0);
  gl.add(mesh);
  gl.watch();

  mesh.writeTriFile(("z.full.tri"));
  mesh.fuseNearVertices(1e-6);
  mesh.writeTriFile(("z.e4.tri"));
  gl.watch();

  mesh.fuseNearVertices(1e-5);
  mesh.writeTriFile(("z.e3.tri"));
  gl.watch();

  mesh.fuseNearVertices(1e-4);
  mesh.writeTriFile(("z.e2.tri"));
  gl.watch();
}

//===========================================================================

void TEST(AddMesh) {
  if(!rai::FileToken("../../../../rai-robotModels/pr2/head_v0/head_pan.stl", false).exists()) return;
  rai::Mesh mesh1,mesh2;
  OpenGL gl;
  gl.add(drawInit,0);
  gl.add(mesh1);
  mesh1.readTriFile(FILE("z.e3.tri"));
  mesh2.readTriFile(FILE("z.e3.tri"));
  uint i,m=0; double my=mesh1.V(m,1);
  for(i=0;i<mesh1.V.d0;i++) if(mesh1.V(i,1)>mesh1.V(m,1)){ m=i; my=mesh1.V(m,1); }
  mesh2.translate(0,my,0);
  mesh1.addMesh(mesh2);
  //mesh1.writeTriFile(("z.e3.tri"));
  //mesh1.writeOffFile(("z.e3.off"));

  gl.watch();
}

//===========================================================================

void TEST(Meshes3) {
  if(!rai::FileToken("../../../../rai-robotModels/pr2/head_v0/head_pan.stl", false).exists()) return;
  rai::Mesh mesh;
  OpenGL gl;
  gl.add(drawInit,0);
  gl.add(mesh);
  //MeshSetSphere(mesh,0);
  mesh.readTriFile(FILE("z.e4.tri"));
  gl.reportSelects=true;
  gl.watch();
  if(gl.topSelection){
    cout <<gl.topSelection->name <<endl;
    uint i=gl.topSelection->name >> 4;
    mesh.skin(i);
  }
  gl.watch();
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
  gl.add(glStandardScene,nullptr);
  gl.add(m);

  rai::Array<ScalarFunction*> fcts = {
    new DistanceFunction_Sphere(t, 1.),
    new DistanceFunction_ssBox(t, 1., 2., 3., 1.),
    new DistanceFunction_Cylinder(t, 2., 1.)
  };

  for(ScalarFunction* f: fcts){
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
  rai::Mesh m;
  m.setImplicitSurface(DistanceFunction_SSBox,-10.,10.,100);
  OpenGL gl;
  gl.add(m);
  gl.watch();

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

int MAIN(int argc, char** argv){
  rai::initCmdLine(argc, argv);

  testPrimitives();
  testFuseVertices();
  testAddMesh();
  testMeshes3();
  testVolume();
  testDistanceFunctions();
//  testDistanceFunctions2();
  testSimpleImplicitSurfaces();

  return 0;
}
