#include <Geo/signedDistanceFunctions.h>
#include <Geo/mesh.h>
#include <Gui/opengl.h>
#include <Gui/RenderData.h>

#include <Optim/newton.h>

//===========================================================================

void TEST(DistanceFunctions) {
  rai::Transformation t;
  t.setRandom();
  rai::Mesh m;
  m.C = {.5, .5, .5, .5};

  OpenGL gl;

  rai::Array<shared_ptr<SDF>> fcts = {
    make_shared<SDF_SuperQuadric>(t, arr{2., 2., 2.}, 1.2),
//    make_shared<SDF_Sphere>(t, 1.),
//    make_shared<SDF_ssBox>(t, 1., 2., 3., 1.),
//    make_shared<SDF_Cylinder>(t, 2., 1.),
//    make_shared<SDF_Capsule>(t, 2., 1.)
  };

  auto f2 = make_shared<SDF_GridData>(*fcts(0), arr{-2.,-2.,-2.}, arr{2.,2.,2.}, uintA{100,100,100});
  fcts.append(f2);

  for(shared_ptr<SDF>& f: fcts){
    m.setImplicitSurfaceBySphereProjection(*f, 3.);

    //-- check hessian and gradient
    for(uint i=0;i<10;i++){
      arr x(3);
      rndUniform(x, -1., 1.);
      bool suc=true;
      suc &= checkGradient(*f, x, 1e-6);
      suc &= checkHessian(*f, x, 1e-6);

      {
        arr g;
        double d = (*f)(g, NoArr, x);
        {
          auto lock = gl.data().dataLock(RAI_HERE);
          gl.data().clear().addStandardScene().add().mesh(m);
          gl.data().addDistMarker(x, x-d*g);
        }
        gl.update();
      }

      if(!suc){
        arr g,H;
        (*f)(g,H,x); //set breakpoint here;
        LOG(-1)  <<"x=" <<x;
      }
    }

    //-- display
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
//    suc &= checkHessian(SDF_SSBox, x, 1e-6);
    if(!suc){
      arr g,H;
      cout <<"f=" <<DistanceFunction_SSBox(g,H,x); //set breakpoint here;
      HALT("x=" <<x);
    }
  }

  //-- display
//  rai::Mesh m;
//  m.setImplicitSurface(SDF_SSBox,-10.,10.,100); //only works for 3D ScalarFunction
//  OpenGL gl;
//  gl.add(m);
//  gl.watch();

}

//===========================================================================
//
// implicit surfaces
//

void TEST(SimpleImplicitSurfaces) {
  rai::Transformation pose;
  pose.setRandom();

  rai::Array<shared_ptr<SDF>> fcts = {
    make_shared<SDF_ssBox>(pose, arr{1., 2., 3.}, .2),
    make_shared<SDF_Blobby>(),
    make_shared<SDF_Torus>(),
  };

  fcts.append( make_shared<SDF_GridData>(*fcts(0), arr{-2.,-3.,-4.}, arr{2.,3.,4.}, uintA{20,20,20}) );
  fcts.append( make_shared<SDF_GridData>(*fcts(1), arr{-5.,-5.,-5.}, arr{5.,5.,5.}, uintA{100,100,100}) );
  fcts.append( make_shared<SDF_GridData>(*fcts(2), arr{-5.,-5.,-5.}, arr{5.,5.,5.}, uintA{100,100,100}) );

  rai::Mesh m;
  OpenGL gl;

  for(shared_ptr<SDF>& f: fcts){
    m.setImplicitSurface(*f,-10.,10.,100);
    gl.data().clear().addStandardScene().add().mesh(m);
    gl.update(true);
  }
}

//===========================================================================

void projectToSurface(){
  rai::Transformation pose;
  pose.setRandom();

  rai::Array<shared_ptr<ScalarFunction>> fcts = {
    make_shared<SDF_Sphere>(pose, 1.),
    make_shared<SDF_ssBox>(pose, arr{1., 2., 3.}, .2),
    make_shared<SDF_Cylinder>(pose, 2., .2),
    make_shared<SDF_Capsule>(pose, 2., .2)
  };

  rai::Mesh m;
  OpenGL gl;

  ofstream fil("z.obj");

  for(shared_ptr<ScalarFunction>& fct:fcts){
    for(uint k=0;k<1000;k++){
      arr x = randn(3);
      x += pose.pos.getArr();
      checkGradient(*fct, x, 1e-4);
      checkHessian(*fct, x, 1e-4);
      fil <<x.modRaw();
      fil <<(*fct)(NoArr, NoArr, x) <<endl;
    }

    m.setImplicitSurfaceBySphereProjection(*fct, 10., 3);

    gl.data().clear().addStandardScene().add().mesh(m);
    gl.update(true);
  }
}

//===========================================================================

void display(){
  rai::Transformation pose=0;
//  pose.setRandom();

  rai::Array<shared_ptr<SDF>> fcts = {
    make_shared<SDF_Sphere>(pose, 1.),
//    make_shared<SDF_ssBox>(pose, arr{1., 2., 3.}, .0),
//    make_shared<SDF_Cylinder>(pose, 2., .2),
//    make_shared<SDF_Capsule>(pose, 2., .2)
  };

  arr lo =  arr{-1.,-1.,-1.};
  arr up = arr{1.,1.,1.};
  auto f2 = make_shared<SDF_GridData>(*fcts(0), lo, up, uintA{20,20,20});
  fcts.append(f2);

  OpenGL gl;

  for(shared_ptr<SDF>& fct:fcts){
    fct->animateSlices(lo, up, .1);
    rai::wait();
  }
}

//===========================================================================

int MAIN(int argc, char** argv){
  rai::initCmdLine(argc, argv);

  testDistanceFunctions();
  testDistanceFunctions2();
  testSimpleImplicitSurfaces();

  projectToSurface();
//  display();

  return 0;
}
