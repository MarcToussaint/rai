#include <Gui/RenderData.h>

#include <Kin/kin.h>
#include <Kin/frame.h>
#include <Gui/opengl.h>
#include <Algo/spline.h>
#include <Core/h5.h>

void testConfig(){
  rai::Configuration C;
  rai::Frame *f = C.addFile(rai::raiPath("../rai-robotModels/panda/panda.g"));
  f->set_X()->appendRelativeRotationDeg(90,0,0,1);

//  C.view(false, "test");

  rai::RenderData scene;
//  scene.drawShadows=false;
//  scene.drawTransparents=false;

  // meshes of C
  for(rai::Frame *f:C.frames){
    if(f->shape && f->shape->type()==rai::ST_mesh){
      scene.add(f->ensure_X()).mesh(f->shape->mesh());
    }
  }

#if 1
  scene.addStandardScene();
#else
  { // floor
    rai::Mesh m;
    m.setQuad();
    m.scale(5., 5., 0.);
    m.C = {1., .95, .9};
    scene.add().mesh(m, 0);
  }
  scene.addLight({5.,5.,5.}, {0.,0.,1.});
  scene.addLight({-5.,0.,5.}, {0.,0.,1.});
#endif

  scene.addAxes(.2, "t(.2 .2 .2)");

  scene.addText("bla:dat 0.13098 t sec() []", 10.0, 20.0, 1.);

  rai::Mesh m;
  m.setSSBox(.4, .4, .4, .1, 2);
  //m.setSphere(0); m.scale(.2);
  m.C = {1., .5, .5, .5};
  scene.add(rai::Transformation("t(0 0 1.)")).mesh(m, .9);

  m.setSSBox(.2, .2, .2, .05, 2);
  m.C = {.5, 1., .5, .5};
  scene.add(rai::Transformation("t(0 -.3 1.)")).mesh(m, .9);

  m.setSSBox(.2, .2, .2, .05, 2);
  m.C = {.5, .5, 1.};
  scene.add(rai::Transformation("t(-.4 -.4 .4)")).mesh(m, .9);

  m.setBox({.2,.2,0.}, {.3,.3,.1}, false);
  m.C = {.5};
  scene.add().mesh(m, .9);

  byteA img;
  read_ppm(img, "../retired/opengl/box.ppm",false);
  add_alpha_channel(img, 120);
  scene.addQuad(img, 20, 20, 60, 60);

  OpenGL gl;
  gl.camera.setDefault();
#if 1
  gl.add(&scene);
  gl.update(true);
#else
  gl.add(scene);
  gl.renderInBack(300, 500); //offscreen rendering tested
  write_png(gl.captureImage, "z.png");
  gl.update(true);
#endif
}

void testTensor(){

  rai::RenderData scene;

  // rai::Mesh m;
  // m.setBox();
  // m.C = m.V; //colors equal coordinates!!! to transfer to shader
  // scene.add(rai::Transformation("t(0 0 1.)")).mesh(m, .9);

  scene.addStandardScene();

  rai::Mesh m;
  m.setSSBox(.4, .4, .4, .1, 2);
  m.C = {1., .5, .5, 1.};
  scene.add(rai::Transformation("t(0 1. 1.)")).mesh(m, .9);

#if 0
  uint d=5;
  arr data = rand({d,d,d});
  arr size = {.3, .2, .5};
  data *= .01;
  cout <<data <<endl;
#else
  rai::H5_Reader h5("/home/mtoussai/git/repair/bone.h5");
  rai::Array<int16_t> density = h5.read<int16_t>("density_org");
  arr pixdim = h5.read<double>("pixdim");

  double threshold = rai::getParameter<double>("threshold");
  double scale = rai::getParameter<double>("scale");
  floatA data(density.d2, density.d1, density.d0);
  for(uint i=0;i<density.d0;i++) for(uint j=0;j<density.d1;j++) for(uint k=0;k<density.d2;k++){
        data(k,j,i) = (float(density(i,j,k)) - threshold) * scale;
      }
  arr size = .001*rai::convert<double>(data.dim()) % arr{pixdim(2), pixdim(1), pixdim(0)};
  LOG(0) <<"data size: " <<data.dim() <<" max: " <<max(data) <<" min: " <<min(data) <<" pixdim: " <<pixdim <<" size: " <<size;
#endif

#if 1
  // cout <<data <<endl;
  scene.add(rai::Transformation("t(0 0 1.) q(1 .5 0 0)"), rai::_tensor).tensor(data, 3.*size);
  scene.items(-1)->scale = 3.*size;
  scene.items(-1)->flatColor = {128, 255, 255};

  OpenGL gl;
  gl.camera.setDefault();
  gl.add(&scene);
  gl.update(true);
#else
  rai::Configuration C;
  rai::Frame *f = C.addFrame("tensor");
  f->setPosition({0.,0.,1.}) .setQuaternion({1.,.5,0.,0.});
  f->setTensorShape(data, 3.*size);
  // f->setColor({1.,1.,0.});
  // C.addFrame("box", "tensor") ->setShape(rai::ST_box, 3.*size) .setColor({1.,1.,0.,.2});
  C.view(true);
#endif
}

int main(int argc, char **argv){
  rai::initCmdLine(argc, argv);

  testConfig();
  testTensor();

  return 0;
}
