/*  ------------------------------------------------------------------
    Copyright (c) 2011-2024 Marc Toussaint
    email: toussaint@tu-berlin.de

    This code is distributed under the MIT License.
    Please see <root-path>/LICENSE for details.
    --------------------------------------------------------------  */

#include "cameraview.h"
#include "frame.h"
#include "../Geo/depth2PointCloud.h"

//===========================================================================

rai::CameraView::CameraView(const rai::Configuration& _C, bool _offscreen) {
  updateConfiguration(_C);
  gl = make_shared<OpenGL>("CameraView", 640, 480, _offscreen);
  gl->camera.setDefault();
  gl->add(this);
}

rai::CameraView::Sensor& rai::CameraView::addSensor(rai::Frame* frame, uint width, uint height, double focalLength, double orthoAbsHeight, const arr& zRange, const char* backgroundImageFile) {
  Sensor& sen = sensors.append();
  sen.name = frame->name;
  sen.frame = frame;
  rai::Camera& cam = sen.cam;
  sen.width=width;
  sen.height=height;

  cam.setZero();
  if(zRange.N) cam.setZRange(zRange(0), zRange(1));
  if(focalLength>0.) cam.setFocalLength(focalLength);
  if(orthoAbsHeight>0.) cam.setHeightAbs(orthoAbsHeight);

  cam.setWHRatio((double)width/height);

  if(sen.frame) cam.X = sen.frame->ensure_X();

  //also select sensor
  gl->resize(sen.width, sen.height);
  currentSensor=&sen;

  return sen;
}

rai::CameraView::Sensor& rai::CameraView::addSensor(rai::Frame* frame) {
  CHECK(frame, "frame is not defined");

  double width=400., height=200.;
  double focalLength=-1.;
  double orthoAbsHeight=-1.;
  arr zRange;

  CHECK(frame->ats, "");
  frame->ats->get<double>(focalLength, "focalLength");
  frame->ats->get<double>(orthoAbsHeight, "orthoAbsHeight");
  frame->ats->get<arr>(zRange, "zRange");
  frame->ats->get<double>(width, "width");
  frame->ats->get<double>(height, "height");

  return addSensor(frame, width, height, focalLength, orthoAbsHeight, zRange);
}

rai::CameraView::Sensor& rai::CameraView::selectSensor(Frame *frame) {
  CHECK(frame, "you need to specify a frame, nullptr not allowed");
  Sensor* sen=0;
  for(Sensor& s:sensors) if(s.frame==frame) { sen=&s; break; }
  if(!sen) {
//    LOG(0) <<"can't find that sensor: " <<sensorName <<" -- trying to add it";
    return addSensor(frame);
  }

  gl->resize(sen->width, sen->height);
  currentSensor=sen;
  return *sen;
}

#if 0
void rai::CameraView::updateConfiguration(const rai::Configuration& newC) {
  auto _dataLock = gl->dataLock(RAI_HERE);
  if(newC.frames.N==C.frames.N) {
#if 0
    C.setFrameState(newC.getFrameState());
#else
    for(uint i=0; i<C.frames.N; i++) {
      rai::Frame* f = newC.frames.elem(i);
      if(f->shape) C.frames.elem(i)->set_X() = f->ensure_X();
    }
#endif
  } else {
    C.copy(newC);
    //deep copy meshes!
    for(rai::Frame* f:C.frames) if(f->shape) {
        if(f->shape->_mesh) {
          shared_ptr<Mesh> org = f->shape->_mesh;
          f->shape->_mesh = make_shared<Mesh> (*org.get());
          f->shape->glListId = 0;
        }
      }
    if(renderMode==seg) { //update frameIDmap
      frameIDmap.resize(C.frames.N).setZero();
      for(rai::Frame* f:C.frames) {
        int* label=f->ats->find<int>("label");
        if(label) frameIDmap(f->ID) = *label;
      }
    }
  }
}
#endif

void rai::CameraView::computeImageAndDepth(byteA& image, floatA& depth) {
  updateCamera();
  if(renderMode==visuals) renderUntil=_shadow;
//  else if(renderMode==all) renderUntil=_all;
  // gl->update(nullptr, true);
  gl->renderInBack();
  image = gl->captureImage;
  flip_image(image);
  if(renderMode==seg && frameIDmap.N) {
    byteA seg(image.d0*image.d1);
    image.reshape(image.d0*image.d1, 3);
    for(uint i=0; i<image.d0; i++) {
      uint id = color2id(image.p+3*i);
      if(id<frameIDmap.N) {
        seg(i) = frameIDmap(id);
      } else
        seg(i) = 0;
    }
    image = seg;
    image.reshape(gl->height, gl->width);
  }
  if(true) { //(!!depth) {
    depth = gl->captureDepth;
    flip_image(depth);
    for(float& d:depth) {
      if(d==1.f || d==0.f) d=-1.f;
      else d = gl->camera.glConvertToTrueDepth(d);
    }
  }
}

byteA rai::CameraView::computeSegmentationImage() {
  updateCamera();
  renderFlatColors=true;
  gl->renderInBack();
  renderFlatColors=false;
  byteA seg = gl->captureImage;
  flip_image(seg);
  return seg;
}

uintA rai::CameraView::computeSegmentationID() {
  byteA seg = computeSegmentationImage();
  uintA segmentation(seg.d0, seg.d1);
  for(uint i=0; i<segmentation.N; i++) {
    segmentation.elem(i) = color2id(seg.p+3*i);
  }
  return segmentation;
}

void rai::CameraView::updateCamera() {
  for(Sensor& sen:sensors) {
    if(sen.frame) sen.cam.X = sen.frame->ensure_X();
  }

  if(currentSensor) {
    gl->camera = currentSensor->cam;
  }
}

//===========================================================================

rai::Sim_CameraView::Sim_CameraView(Var<rai::Configuration>& _kin,
                                    Var<byteA> _color,
                                    Var<floatA> _depth,
                                    double beatIntervalSec, const char* _cameraFrameName, bool _idColors, const byteA& _frameIDmap)
  : Thread("Sim_CameraView", beatIntervalSec),
    model(this, _kin, (beatIntervalSec<0.)),
    color(this, _color),
    depth(this, _depth),
    V(model.get()()) {
  if(_cameraFrameName) {
    rai::Frame* f = model.get()->getFrame(_cameraFrameName);
    V.addSensor(f);
    V.selectSensor(f);
  }
  if(_idColors) {
    V.renderMode = V.seg;
    if(_frameIDmap.N)
      V.frameIDmap = _frameIDmap;
    else {
      NIY; //V.C.clear();;
      V.updateConfiguration(model.get());
    }
  } else {
    V.renderMode = V.visuals;
  }
  if(beatIntervalSec>=0.) threadLoop(); else threadStep();
}

rai::Sim_CameraView::~Sim_CameraView() {
  threadClose();
}

void rai::Sim_CameraView::step() {
  byteA img;
  floatA dep;
  V.updateConfiguration(model.get());
  V.computeImageAndDepth(img, dep);
  color.set() = img;
  depth.set() = dep;
}

arr rai::Sim_CameraView::getFxycxy() {
  auto sen = V.currentSensor;
  return arr{sen->cam.focalLength* sen->height, sen->cam.focalLength* sen->height, .5*(sen->width-1.), .5*(sen->height-1.)};
}
