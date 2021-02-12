/*  ------------------------------------------------------------------
    Copyright (c) 2011-2020 Marc Toussaint
    email: toussaint@tu-berlin.de

    This code is distributed under the MIT License.
    Please see <root-path>/LICENSE for details.
    --------------------------------------------------------------  */

#include "viewer.h"
#include "../Gui/opengl.h"
#include "../Geo/mesh.h"

//===========================================================================
//
// ImageViewer
//

struct sImageViewer {
  OpenGL gl;
  sImageViewer(const char* tit) : gl(tit) {}
};

//ImageViewer::ImageViewer(const char* img_name)
//  : Thread(STRING("ImageViewer_"<<img_name), -1),
//    img(this, img_name, true) {
//  threadOpen();
//}

ImageViewer::ImageViewer(const Var<byteA>& _img, double beatIntervalSec)
  : Thread(STRING("ImageViewer_" <<_img.name()), beatIntervalSec),
    img(this, _img, (beatIntervalSec<0.)) {
  if(beatIntervalSec>=0.) threadLoop(); else threadStep();
}

ImageViewer::~ImageViewer() {
  threadClose();
}

void ImageViewer::open() {
  self = make_unique<sImageViewer>(STRING("ImageViewer: "<<img.data->name));
  self->gl.openWindow();
  self->gl.update();
}

void ImageViewer::close() { self.reset(); }

void ImageViewer::step() {
  {
    auto _dataLock = self->gl.dataLock(RAI_HERE);
    self->gl.background = img.get();
    if(flipImage) flip_image(self->gl.background);
#if 0 //draw a center
    uint ci = self->gl.background.d0/2;
    uint cj = self->gl.background.d1/2;
    uint skip = self->gl.background.d1*self->gl.background.d2;
    byte* p, *pstop;
    p=&self->gl.background(ci-5, cj-5, 0);
    pstop=&self->gl.background(ci-5, cj+5, 0);
    for(; p<=pstop; p++) *p = 0;
    p=&self->gl.background(ci+5, cj-5, 0);
    pstop=&self->gl.background(ci+5, cj+5, 0);
    for(; p<=pstop; p++) *p = 0;
    p=&self->gl.background(ci-5, cj-5, 0);
    pstop=&self->gl.background(ci+5, cj-5, 0);
    for(; p<=pstop; p+=skip) p[0]=p[1]=p[2]=0;
    p=&self->gl.background(ci-5, cj+5, 0);
    pstop=&self->gl.background(ci+5, cj+5, 0);
    for(; p<=pstop; p+=skip) p[0]=p[1]=p[2]=0;
#endif

    if(!self->gl.background.N) return;
    if(self->gl.height!= self->gl.background.d0 || self->gl.width!= self->gl.background.d1)
      self->gl.resize(self->gl.background.d1, self->gl.background.d0);
  }

  self->gl.update(name, false); //, false, false, true);
}

//===========================================================================
//
// ImageViewerFloat
//

ImageViewerFloat::ImageViewerFloat(const Var<floatA>& _img, double beatIntervalSec, float _scale)
  : Thread(STRING("ImageViewerFloat_" <<_img.name()), beatIntervalSec),
    img(this, _img, (beatIntervalSec<0.)),
    scale(_scale) {
  gl = make_shared<OpenGL>(STRING("ImageViewerFloat: "<<img.data->name));
  if(beatIntervalSec>=0.) threadLoop(); else threadStep();
}

ImageViewerFloat::~ImageViewerFloat() {
  threadClose();
}

void ImageViewerFloat::step() {
  floatA img_copy;
  {
    auto _dataLock = gl->dataLock(RAI_HERE);
    img_copy = img.get();
    if(flipImage) flip_image(img_copy);
    if(scale!=1.f) img_copy *= scale;

    if(!img_copy.N) return;
    if(gl->height!= img_copy.d0 || gl->width!= img_copy.d1) gl->resize(img_copy.d1, img_copy.d0);
  }

  gl->watchImage(img_copy, false, 1.);
}

//===========================================================================

ImageViewerCallback::ImageViewerCallback(const Var<byteA>& _img)
  : img(_img) {
  img.data->callbacks.append(new Callback<void(Var_base*)>(this, std::bind(&ImageViewerCallback::call, this, std::placeholders::_1)));
}

ImageViewerCallback::~ImageViewerCallback() {
  img.data->callbacks.removeCallback(this);
  if(gl) delete gl;
}

void ImageViewerCallback::call(Var_base* v) {
  if(!gl) {
    gl = new OpenGL(STRING("ImageViewer: "<<img.data->name));
  }

  {
    auto _dataLock = gl->dataLock(RAI_HERE);
    img.checkLocked();
    gl->background = img();
    if(flipImage) flip_image(gl->background);
  }

  if(!gl->background.N) return;

  if(gl->height!= gl->background.d0 || gl->width!= gl->background.d1)
    gl->resize(gl->background.d1, gl->background.d0);

  gl->update(); //0, false, false, true);
}

//===========================================================================
//
// PointCloudViewer
//

struct sPointCloudViewer {
  OpenGL gl;
  sPointCloudViewer(const char* tit) : gl(tit) { gl.drawOptions.pclPointSize=3.; }
  rai::Mesh pc;
};

void glDrawAxes(void*) {
  glDrawAxes(1.);
}

//PointCloudViewer::PointCloudViewer(const char* pts_name, const char* rgb_name)
//  : Thread(STRING("PointCloudViewer_"<<pts_name <<'_' <<rgb_name), .1),
//    pts(this, pts_name),
//    rgb(this, rgb_name) {
//  threadLoop();
//}

PointCloudViewer::PointCloudViewer()
  : Thread("PointCloudViewer"),
    pts(this, true),
    rgb(this, true) {
  threadOpen(true);
}

PointCloudViewer::PointCloudViewer(const Var<arr>& _pts, const Var<byteA>& _rgb, double beatIntervalSec)
  : Thread(STRING("PointCloudViewer_"<<_pts.name() <<'_' <<_rgb.name()), beatIntervalSec),
    pts(this, _pts, (beatIntervalSec<0.)),
    rgb(this, _rgb, (beatIntervalSec<0.)) {
  if(beatIntervalSec>=0.) threadLoop(); else threadStep();
}

PointCloudViewer::~PointCloudViewer() {
  threadClose();
}

void PointCloudViewer::open() {
  self = make_unique<sPointCloudViewer>(STRING("PointCloudViewer: "<<pts.name() <<' ' <<rgb.name()));
#if 1
  self->gl.add(glStandardOriginAxes);
  self->gl.add(glStandardLight);
  self->gl.add(self->pc);
  //  self->gl.reportSelects = true;
#else
  self->gl.add(glStandardScene);
  self->gl.add(self->pc);
//  self->gl.camera.setDefault();
#endif
}

void PointCloudViewer::close() {
  self->gl.clear();
  self.reset();
}

void PointCloudViewer::step() {
  uint W, H;

  {
    auto _dataLock = self->gl.dataLock(RAI_HERE);
    self->pc.V=pts.get();
    copy(self->pc.C, rgb.get()());
    self->pc.Vn=normals.get();
    H=self->pc.C.d0;
    W=self->pc.C.d1;
    uint n=self->pc.V.N/3;
    if(n!=self->pc.C.N/3) {

      return;
    }
    self->pc.C /= 255.;
    self->pc.V.reshape(n, 3);
    self->pc.C.reshape(n, 3);
    if(self->pc.Vn.N) self->pc.Vn.reshape(n, 3);
  }

  if(W!=self->gl.width || H!=self->gl.height) self->gl.resize(W, H);

  self->gl.update(); //nullptr, false, false, true);
}

//===========================================================================

PointCloudViewerCallback::PointCloudViewerCallback(const Var<arr>& _pts, const Var<byteA>& _rgb)
  : pts(_pts),
    rgb(_rgb) {
  pts.data->callbacks.append(new Callback<void(Var_base*)>(this, std::bind(&PointCloudViewerCallback::call, this, std::placeholders::_1)));
}

PointCloudViewerCallback::~PointCloudViewerCallback() {
  pts.data->callbacks.removeCallback(this);
}

void PointCloudViewerCallback::call(Var_base* v) {
  if(!self) {
    self = make_unique<sPointCloudViewer>(STRING("PointCloudViewer: "<<pts.name() <<' ' <<rgb.name()));
    self->gl.add(glStandardScene);
    self->gl.add(self->pc);
  }

  uint W, H;

  {
    auto _dataLock = self->gl.dataLock(RAI_HERE);
    pts.checkLocked();
    self->pc.V=pts();
    copy(self->pc.C, rgb.get()());
    H=self->pc.C.d0;
    W=self->pc.C.d1;
    uint n=self->pc.V.N/3;
    if(n!=self->pc.C.N/3) {

      return;
    }
    self->pc.C /= 255.;
    self->pc.V.reshape(n, 3);
    self->pc.C.reshape(n, 3);
  }

  if(W!=self->gl.width || H!=self->gl.height) self->gl.resize(W, H);

  self->gl.update(); //nullptr, false, false, true);
}

//===========================================================================
//
// MeshAViewer
//

MeshAViewer::MeshAViewer(const Var<MeshA>& _meshes)
  : Thread(STRING("MeshAViewer_"<<_meshes.name()), .1),
    meshes(this, _meshes) {
  threadLoop();
}

MeshAViewer::~MeshAViewer() {
  threadClose();
}

void MeshAViewer::open() {
  gl = new OpenGL(STRING("MeshAViewer: "<<meshes.name()));
  gl->add(glStandardScene);
  gl->add(glDrawMeshes, &copy);
  gl->camera.setDefault();
}

void MeshAViewer::close() {
  delete gl;
}

void MeshAViewer::step() {
  {
    auto _dataLock = gl->dataLock(RAI_HERE);
    copy = meshes.get();
  }

  gl->update(); //nullptr, false, false, true);
}

//===========================================================================
//
// PlotViewer
//

PlotViewer::PlotViewer(const Var<arr>& _data, double beatIntervalSec)
  : Thread(STRING("PlotViewer_" <<_data.name()), beatIntervalSec),
    data(this, _data, (beatIntervalSec<0.)) {
  if(beatIntervalSec>=0.) threadLoop(); else threadOpen();
}

PlotViewer::~PlotViewer() {
  threadClose();
}

void PlotViewer::open() {
  gl = new OpenGL(STRING("PlotViewer: "<<data.name()));
  gl->add(*this);

  gl->setClearColors(1., 1., 1., 1.);
  double xl=0., xh=1., yl=-.5, yh=+.5;
  gl->camera.setPosition(.5*(xh+xl), .5*(yh+yl), 5.);
  gl->camera.focus(.5*(xh+xl), .5*(yh+yl), .0);
  gl->camera.setWHRatio((xh-xl)/(yh-yl));
  gl->camera.setHeightAbs(1.2*(yh-yl));
}

void PlotViewer::step() {
  arr x = data.get();
  int r = data.getRevision();
  if(!x.N) return;
  if(x0.N!=x.N) x0=x;
  CHECK_EQ(x.nd, 1, "");
  if(!plot.N) plot.resize(T-1, x0.N).setZero();
  plot.append(x);
  plot.reshape(plot.N/x.N, x.N);
  if(plot.d0>T) plot.delRows(0, plot.d0-T);
  gl->update(STRING("data revision" <<r), true);
}

void PlotViewer::close() {
  delete gl;
}

void PlotViewer::glDraw(OpenGL&) {
#ifdef RAI_GL
  //  rai::Color c;
//  glColor(c.r, c.g, c.b);
  glColor(0., 0., 0.);
  for(uint i=0; i<plot.d1; i++) {
    glBegin(GL_LINE_STRIP);
    for(uint x=0; x<plot.d0; x++) {
      float _x = float(x)/plot.d0;
      float _y = plot(x, i) - x0(i);
      glVertex3f(_x, _y, -1.f);
    }
    glEnd();
  }
#endif
}

