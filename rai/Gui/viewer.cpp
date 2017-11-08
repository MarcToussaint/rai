#include "viewer.h"
#include <Gui/opengl.h>
#include <Geo/mesh.h>

//===========================================================================
//
// ImageViewer
//

struct sImageViewer{
  OpenGL gl;
  sImageViewer(const char* tit) : gl(tit) {}
};

ImageViewer::ImageViewer(const char* img_name)
  : Thread(STRING("ImageViewer_"<<img_name), -1),
    img(this, img_name, true){
  threadOpen();
}

ImageViewer::~ImageViewer(){
  threadClose();
}

void ImageViewer::open(){
  s = new sImageViewer(STRING("ImageViewer: "<<img.data->name));
  s->gl.openWindow();
  s->gl.update();
}

void ImageViewer::close(){ delete s; }

void ImageViewer::step(){
  s->gl.dataLock.writeLock();
  s->gl.background = img.get();
  if(flipImage) flip_image(s->gl.background);
#if 1 //draw a center
  uint ci = s->gl.background.d0/2;
  uint cj = s->gl.background.d1/2;
  uint skip = s->gl.background.d1*s->gl.background.d2;
  byte *p, *pstop;
  p=&s->gl.background(ci-5, cj-5, 0);
  pstop=&s->gl.background(ci-5, cj+5, 0);
  for(;p<=pstop;p++) *p = 0;
  p=&s->gl.background(ci+5, cj-5, 0);
  pstop=&s->gl.background(ci+5, cj+5, 0);
  for(;p<=pstop;p++) *p = 0;
  p=&s->gl.background(ci-5, cj-5, 0);
  pstop=&s->gl.background(ci+5, cj-5, 0);
  for(;p<=pstop;p+=skip) p[0]=p[1]=p[2]=0;
  p=&s->gl.background(ci-5, cj+5, 0);
  pstop=&s->gl.background(ci+5, cj+5, 0);
  for(;p<=pstop;p+=skip) p[0]=p[1]=p[2]=0;
#endif
  s->gl.dataLock.unlock();
  if(!s->gl.background.N) return;
  if(s->gl.height!= s->gl.background.d0 || s->gl.width!= s->gl.background.d1)
    s->gl.resize(s->gl.background.d1, s->gl.background.d0);

  s->gl.update(name, false, false, true);
}


//===========================================================================
//
// PointCloudViewer
//

struct sPointCloudViewer{
  OpenGL gl;
  sPointCloudViewer(const char* tit) : gl(tit,640,480){}
  mlr::Mesh pc;
};

void glDrawAxes(void*){
  glDrawAxes(1.);
}

PointCloudViewer::PointCloudViewer(const char* pts_name, const char* rgb_name)
  : Thread(STRING("PointCloudViewer_"<<pts_name <<'_' <<rgb_name), .1),
    pts(this, pts_name),
    rgb(this, rgb_name){
  threadLoop();
}

PointCloudViewer::~PointCloudViewer(){
  threadClose();
}

void PointCloudViewer::open(){
  s = new sPointCloudViewer(STRING("PointCloudViewer: "<<pts.name <<' ' <<rgb.name));
#if 0
  s->gl.add(glDrawAxes);
  s->gl.add(s->pc);
  s->gl.camera.setKinect();
  //  s->gl.reportSelects = true;
#else
  s->gl.add(glStandardScene);
  s->gl.add(s->pc);
  s->gl.camera.setDefault();
#endif
}

void PointCloudViewer::close(){
  delete s;
}

void PointCloudViewer::step(){
  s->gl.dataLock.writeLock();
  s->pc.V=pts.get();
  copy(s->pc.C, rgb.get()());
  uint n=s->pc.V.N/3;
  if(n!=s->pc.C.N/3){
    s->gl.dataLock.unlock();
    return;
  }
  s->pc.C /= 255.;
  s->pc.V.reshape(n,3);
  s->pc.C.reshape(n,3);
  s->gl.dataLock.unlock();

  s->gl.update(NULL, false, false, true);
}

//===========================================================================
//
// MeshAViewer
//

MeshAViewer::MeshAViewer(const char* meshes_name)
  : Thread(STRING("MeshAViewer_"<<meshes_name), .1),
    meshes(this, meshes_name){
  threadLoop();
}

MeshAViewer::~MeshAViewer(){
  threadClose();
}

void MeshAViewer::open(){
  gl = new OpenGL(STRING("MeshAViewer: "<<meshes.name));
  gl->add(glStandardScene);
  gl->add(glDrawMeshes, &copy);
  gl->camera.setDefault();
}

void MeshAViewer::close(){
  delete gl;
}

void MeshAViewer::step(){
  gl->dataLock.writeLock();
  copy = meshes.get();
  gl->dataLock.unlock();

  gl->update(NULL, false, false, true);
}

