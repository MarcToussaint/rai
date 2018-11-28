/*  ------------------------------------------------------------------
    Copyright (c) 2017 Marc Toussaint
    email: marc.toussaint@informatik.uni-stuttgart.de

    This code is distributed under the MIT License.
    Please see <root-path>/LICENSE for details.
    --------------------------------------------------------------  */

#include "viewer.h"
#include <Gui/opengl.h>
#include <Geo/mesh.h>

//===========================================================================
//
// ImageViewer
//

struct sImageViewer {
  OpenGL gl;
  sImageViewer(const char* tit) : gl(tit) {}
};

ImageViewer::ImageViewer(const char* img_name)
  : Thread(STRING("ImageViewer_"<<img_name), -1),
    img(this, img_name, true) {
  threadOpen();
}

ImageViewer::ImageViewer(const Var<byteA>& _img, double beatIntervalSec)
  : Thread("ImageViewer", beatIntervalSec),
    img(this, _img, (beatIntervalSec<0.)){
  if(beatIntervalSec>=0.) threadLoop(); else threadStep();
}

ImageViewer::~ImageViewer() {
  threadClose();
}

void ImageViewer::open() {
  s = new sImageViewer(STRING("ImageViewer: "<<img.data->name));
  s->gl.openWindow();
  s->gl.update();
}

void ImageViewer::close() { delete s; }

void ImageViewer::step() {
  s->gl.dataLock.writeLock();
  s->gl.background = img.get();
  if(flipImage) flip_image(s->gl.background);
#if 0 //draw a center
  uint ci = s->gl.background.d0/2;
  uint cj = s->gl.background.d1/2;
  uint skip = s->gl.background.d1*s->gl.background.d2;
  byte *p, *pstop;
  p=&s->gl.background(ci-5, cj-5, 0);
  pstop=&s->gl.background(ci-5, cj+5, 0);
  for(; p<=pstop; p++) *p = 0;
  p=&s->gl.background(ci+5, cj-5, 0);
  pstop=&s->gl.background(ci+5, cj+5, 0);
  for(; p<=pstop; p++) *p = 0;
  p=&s->gl.background(ci-5, cj-5, 0);
  pstop=&s->gl.background(ci+5, cj-5, 0);
  for(; p<=pstop; p+=skip) p[0]=p[1]=p[2]=0;
  p=&s->gl.background(ci-5, cj+5, 0);
  pstop=&s->gl.background(ci+5, cj+5, 0);
  for(; p<=pstop; p+=skip) p[0]=p[1]=p[2]=0;
#endif
  s->gl.dataLock.unlock();
  if(!s->gl.background.N) return;
  if(s->gl.height!= s->gl.background.d0 || s->gl.width!= s->gl.background.d1)
    s->gl.resize(s->gl.background.d1, s->gl.background.d0);
    
  s->gl.update(name, false, false, true);
}

//===========================================================================

ImageViewerCallback::ImageViewerCallback(const Var<byteA>& _img)
  : img(_img){
  img.data->callbacks.append(new Callback<void(Var_base*,int)>(this, std::bind(&ImageViewerCallback::call, this, std::placeholders::_1, std::placeholders::_2)));
}

ImageViewerCallback::~ImageViewerCallback(){
  img.data->callbacks.removeCallback(this);
  if(gl) delete gl;
}

void ImageViewerCallback::call(Var_base* v, int revision){
  if(!gl){
    gl = new OpenGL(STRING("ImageViewer: "<<img.data->name));
  }

  gl->dataLock.writeLock();
  img.checkLocked();
  gl->background = img();
  if(flipImage) flip_image(gl->background);
  gl->dataLock.unlock();
  if(!gl->background.N) return;

  if(gl->height!= gl->background.d0 || gl->width!= gl->background.d1)
    gl->resize(gl->background.d1, gl->background.d0);

  gl->update(0, false, false, true);
}


//===========================================================================
//
// PointCloudViewer
//

struct sPointCloudViewer {
  OpenGL gl;
  sPointCloudViewer(const char* tit) : gl(tit) {}
  rai::Mesh pc;
};

void glDrawAxes(void*) {
  glDrawAxes(1.);
}

PointCloudViewer::PointCloudViewer(const char* pts_name, const char* rgb_name)
  : Thread(STRING("PointCloudViewer_"<<pts_name <<'_' <<rgb_name), .1),
    pts(this, pts_name),
    rgb(this, rgb_name) {
  threadLoop();
}

PointCloudViewer::PointCloudViewer(const Var<arr>& _pts, const Var<byteA>& _rgb, double beatIntervalSec)
  : Thread("PointCloudViewer", beatIntervalSec),
    pts(this, _pts, (beatIntervalSec<0.)),
    rgb(this, _rgb, (beatIntervalSec<0.)){
  if(beatIntervalSec>=0.) threadLoop(); else threadStep();
}

PointCloudViewer::~PointCloudViewer() {
  threadClose();
}

void PointCloudViewer::open() {
  s = new sPointCloudViewer(STRING("PointCloudViewer: "<<pts.name <<' ' <<rgb.name));
#if 1
  s->gl.add(glDrawAxes);
  s->gl.add(glStandardLight);
  s->gl.add(s->pc);
  //  s->gl.reportSelects = true;
#else
  s->gl.add(glStandardScene);
  s->gl.add(s->pc);
//  s->gl.camera.setDefault();
#endif
}

void PointCloudViewer::close() {
  delete s;
}

void PointCloudViewer::step() {
  uint W,H;

  s->gl.dataLock.writeLock();
  s->pc.V=pts.get();
  copy(s->pc.C, rgb.get()());
  H=s->pc.C.d0;
  W=s->pc.C.d1;
  uint n=s->pc.V.N/3;
  if(n!=s->pc.C.N/3) {
    s->gl.dataLock.unlock();
    return;
  }
  s->pc.C /= 255.;
  s->pc.V.reshape(n,3);
  s->pc.C.reshape(n,3);
  s->gl.dataLock.unlock();

  if(W!=s->gl.width || H!=s->gl.height) s->gl.resize(W,H);

  
  s->gl.update(NULL, false, false, true);
}

//===========================================================================

PointCloudViewerCallback::PointCloudViewerCallback(const Var<arr>& _pts, const Var<byteA>& _rgb)
  : pts(_pts),
    rgb(_rgb){
  pts.data->callbacks.append(new Callback<void(Var_base*,int)>(this, std::bind(&PointCloudViewerCallback::call, this, std::placeholders::_1, std::placeholders::_2)));
}

PointCloudViewerCallback::~PointCloudViewerCallback(){
  pts.data->callbacks.removeCallback(this);
  if(s) delete s;
}

void PointCloudViewerCallback::call(Var_base* v, int revision){
  if(!s){
    s = new sPointCloudViewer(STRING("PointCloudViewer: "<<pts.name <<' ' <<rgb.name));
    s->gl.add(glStandardScene);
    s->gl.add(s->pc);
  }

  uint W,H;

  s->gl.dataLock.writeLock();
  pts.checkLocked();
  s->pc.V=pts();
  copy(s->pc.C, rgb.get()());
  H=s->pc.C.d0;
  W=s->pc.C.d1;
  uint n=s->pc.V.N/3;
  if(n!=s->pc.C.N/3) {
    s->gl.dataLock.unlock();
    return;
  }
  s->pc.C /= 255.;
  s->pc.V.reshape(n,3);
  s->pc.C.reshape(n,3);
  s->gl.dataLock.unlock();

  if(W!=s->gl.width || H!=s->gl.height) s->gl.resize(W,H);


  s->gl.update(NULL, false, false, true);
}


//===========================================================================
//
// MeshAViewer
//

MeshAViewer::MeshAViewer(const char* meshes_name)
  : Thread(STRING("MeshAViewer_"<<meshes_name), .1),
    meshes(this, meshes_name) {
  threadLoop();
}

MeshAViewer::~MeshAViewer() {
  threadClose();
}

void MeshAViewer::open() {
  gl = new OpenGL(STRING("MeshAViewer: "<<meshes.name));
  gl->add(glStandardScene);
  gl->add(glDrawMeshes, &copy);
  gl->camera.setDefault();
}

void MeshAViewer::close() {
  delete gl;
}

void MeshAViewer::step() {
  gl->dataLock.writeLock();
  copy = meshes.get();
  gl->dataLock.unlock();
  
  gl->update(NULL, false, false, true);
}

