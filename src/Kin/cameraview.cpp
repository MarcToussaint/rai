/*  ------------------------------------------------------------------
    Copyright (c) 2011-2020 Marc Toussaint
    email: toussaint@tu-berlin.de

    This code is distributed under the MIT License.
    Please see <root-path>/LICENSE for details.
    --------------------------------------------------------------  */

#include "cameraview.h"
#include "frame.h"
#include "../Geo/depth2PointCloud.h"

//===========================================================================

rai::CameraView::CameraView(const rai::Configuration& _C, bool _offscreen)
  : gl("CameraView", 640, 480, _offscreen) {

  updateConfiguration(_C);

  gl.add(*this);
}

rai::CameraView::Sensor& rai::CameraView::addSensor(const char* name, const char* frameAttached, uint width, uint height, double focalLength, double orthoAbsHeight, const arr& zRange, const char* backgroundImageFile) {
  Sensor& sen = sensors.append();
  sen.name = name;
  sen.frame = C.getFrame(frameAttached)->ID;
  rai::Camera& cam = sen.cam;
  sen.width=width;
  sen.height=height;

  cam.setZero();
  if(zRange.N) cam.setZRange(zRange(0), zRange(1));
  if(focalLength>0.) cam.setFocalLength(focalLength);
  if(orthoAbsHeight>0.) cam.setHeightAbs(orthoAbsHeight);

  cam.setWHRatio((double)width/height);

  if(sen.frame>=0) cam.X = C.frames.elem(sen.frame)->ensure_X();

  //also select sensor
  gl.resize(sen.width, sen.height);
  currentSensor=&sen;

  return sen;
}

rai::CameraView::Sensor& rai::CameraView::addSensor(const char* frameAttached) {
  rai::Frame* frame = C.getFrame(frameAttached);

  CHECK(frame, "frame '" <<frameAttached <<"' is not defined");

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

  return addSensor(frameAttached, frameAttached, width, height, focalLength, orthoAbsHeight, zRange);
}

rai::CameraView::Sensor& rai::CameraView::selectSensor(const char* sensorName) {
  CHECK(sensorName, "you need to specify a sensor name, nullptr not allowed");
  Sensor* sen=0;
  for(Sensor& s:sensors) if(s.name==sensorName) { sen=&s; break; }
  if(!sen){
//    LOG(0) <<"can't find that sensor: " <<sensorName <<" -- trying to add it";
    return addSensor(sensorName);
  }

  gl.resize(sen->width, sen->height);
  currentSensor=sen;
  return *sen;
}

void rai::CameraView::updateConfiguration(const rai::Configuration& newC) {
  arr X = newC.getFrameState();
  auto _dataLock = gl.dataLock(RAI_HERE);
  if(X.d0==C.frames.N) {
    C.setFrameState(X);
  } else {
    C.copy(newC);
    //deep copy meshes!
    for(rai::Frame* f:C.frames) if(f->shape) {
        shared_ptr<Mesh> org = f->shape->_mesh;
        f->shape->_mesh = make_shared<Mesh> (*org.get());
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

void rai::CameraView::computeImageAndDepth(byteA& image, floatA& depth) {
  updateCamera();
  //  renderMode=all;
  // gl.update(nullptr, true);
  gl.renderInBack();
  image = gl.captureImage;
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
    image.reshape(gl.height, gl.width);
  }
  if(true) { //(!!depth) {
    depth = gl.captureDepth;
    flip_image(depth);
    for(float& d:depth) {
      if(d==1.f || d==0.f) d=-1.f;
      else d = gl.camera.glConvertToTrueDepth(d);
    }
  }
}

byteA rai::CameraView::computeSegmentationImage() {
  updateCamera();
  renderMode=seg;
  gl.renderInBack();
  byteA seg = gl.captureImage;
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
    if(sen.frame>=0) sen.cam.X = C.frames.elem(sen.frame)->ensure_X();
  }

  if(currentSensor) {
    gl.background = currentSensor->backgroundImage;
    gl.backgroundZoom = (double)currentSensor->height/gl.background.d0;
    gl.camera = currentSensor->cam;
  }
}

void rai::CameraView::glDraw(OpenGL& gl) {
  if(renderMode==all || renderMode==visuals) {
    glStandardScene(nullptr, gl);
    gl.drawOptions.drawMode_idColor = false;
    gl.drawOptions.drawColors = true;
    if(renderMode==visuals) {
      gl.drawOptions.drawVisualsOnly=true;
    } else {
      gl.drawOptions.drawVisualsOnly=false;
    }

    C.glDraw(gl);

    if(renderMode!=visuals) {
      for(Sensor& sen:sensors) {
        glTransform(sen.cam.X);
        glDrawCamera(sen.cam);
        glDrawText(STRING("SENSOR " <<sen.name), 0., 0., 0.);
      }
    }
  }

  if(renderMode==seg) {
    gl.clearColor=1.;
    gl.background.clear();
    gl.drawOptions.drawMode_idColor = true;
    gl.drawOptions.drawColors=false;
    gl.drawOptions.drawVisualsOnly=true;
    C.glDraw(gl);
    gl.drawOptions.drawMode_idColor = false;
    gl.drawOptions.drawColors=true;
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
    V.addSensor(_cameraFrameName);
    V.selectSensor(_cameraFrameName);
  }
  if(_idColors) {
    V.renderMode = V.seg;
    if(_frameIDmap.N)
      V.frameIDmap = _frameIDmap;
    else {
      V.C.clear();;
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
  return arr{sen->cam.focalLength*sen->height, sen->cam.focalLength*sen->height, .5*(sen->width-1.), .5*(sen->height-1.)};
}
