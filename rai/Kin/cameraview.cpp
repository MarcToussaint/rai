#include "cameraview.h"
#include <Kin/frame.h>

extern bool Geo_mesh_drawColors; //UGLY!!

//===========================================================================

rai::CameraView::CameraView(const rai::KinematicWorld& _K, bool _offscreen, int _watchComputations)
  : gl("CameraView", 640, 480, _offscreen), watchComputations(_watchComputations) {

  updateConfiguration(_K);

  gl.add(*this);
}

rai::CameraView::Sensor& rai::CameraView::addSensor(const char* name, const char* frameAttached, uint width, uint height, double focalLength, double orthoAbsHeight, const arr& zRange, const char* backgroundImageFile){
  Sensor& sen = sensors.append();
  sen.name = name;
  sen.frame = K.getFrameByName(frameAttached)->ID;
  rai::Camera& cam = sen.cam;
  sen.width=width;
  sen.height=height;

  cam.setZero();
  if(zRange.N) cam.setZRange(zRange(0), zRange(1));
  if(focalLength>0.) cam.setFocalLength(focalLength);
  if(orthoAbsHeight>0.) cam.setHeightAbs(orthoAbsHeight);

  cam.setWHRatio((double)width/height);

  if(sen.frame>=0) cam.X = K.frames(sen.frame)->X;

  done(__func__);
  return sen;
}

rai::CameraView::Sensor& rai::CameraView::addSensor(const char* name, const char* frameAttached){
  rai::Frame *frame = K.getFrameByName(frameAttached);

  CHECK(frame, "frame '" <<frameAttached <<"' is not defined");

  double width=400., height=200.;
  double focalLength=-1.;
  double orthoAbsHeight=-1.;
  arr zRange;

  frame->ats.get<double>(focalLength, "focalLength");
  frame->ats.get<double>(orthoAbsHeight, "orthoAbsHeight");
  frame->ats.get<arr>(zRange, "zRange");
  frame->ats.get<double>(width, "width");
  frame->ats.get<double>(height, "height");

  return addSensor(name, frameAttached, width, height, focalLength, orthoAbsHeight, zRange);
}

rai::CameraView::Sensor& rai::CameraView::selectSensor(const char* sensorName){
  CHECK(sensorName, "you need to specify a sensor name, NULL not allowed");
  Sensor *sen=0;
  for(Sensor& s:sensors) if(s.name==sensorName){ sen=&s; break; }
  if(!sen) LOG(-2) <<"can't find that sensor: " <<sensorName;

  gl.resize(sen->width, sen->height);
  currentSensor=sen;
  done(__func__);
  return *sen;
}

void rai::CameraView::updateConfiguration(const rai::KinematicWorld& newC){
  arr X = newC.getFrameState();
  auto _dataLock = gl.dataLock(RAI_HERE);
  if(X.d0==K.frames.N){
    K.setFrameState(X);
  }else{
    K.copy(newC);
    //deep copy meshes!
    for(rai::Frame *f:K.frames) if(f->shape){
        ptr<Mesh> org = f->shape->_mesh;
        f->shape->_mesh = make_shared<Mesh> ( *org.get() );
    }
    if(renderMode==seg){//update frameIDmap
      frameIDmap.resize(K.frames.N).setZero();
      for(rai::Frame *f:K.frames){
        int *label=f->ats.find<int>("label");
        if(label) frameIDmap(f->ID) = *label;
      }
    }
  }
}

void rai::CameraView::computeImageAndDepth(byteA& image, floatA& depth){
  updateCamera();
  //  renderMode=all;
  gl.update(NULL, true);
  image = gl.captureImage;
  flip_image(image);
  if(renderMode==seg && frameIDmap.N){
    byteA seg(image.d0*image.d1);
    image.reshape(image.d0*image.d1, 3);
    for(uint i=0; i<image.d0; i++){
      uint id = color2id(image.p+3*i);
      if(id<frameIDmap.N){
        seg(i) = frameIDmap(id);
      }else
        seg(i) = 0;
    }
    image = seg;
    image.reshape(gl.height, gl.width);
  }
  if(!!depth){
    depth = gl.captureDepth;
    flip_image(depth);
    for(float& d:depth){
      if(d==1.f || d==0.f) d=-1.f;
      else d = gl.camera.glConvertToTrueDepth(d);
    }
  }
  done(__func__);
}

void rai::CameraView::computeSegmentation(byteA& segmentation){
  updateCamera();
  renderMode=seg;
  gl.update(NULL, true);
  segmentation = gl.captureImage;
  flip_image(segmentation);
  done(__func__);
}



void rai::CameraView::computeKinectDepth(uint16A& kinect_depth, const arr& depth){
  kinect_depth.resize(depth.d0, depth.d1);
  for(uint i=0; i<depth.N; i++) kinect_depth.elem(i) = (uint16_t) (depth.elem(i) * 1000.);
}

void rai::CameraView::computePointCloud(arr& pts, const floatA& depth, bool globalCoordinates){
  uint H=depth.d0, W=depth.d1;

  pts.resize(H*W, 3);

  if(currentSensor) gl.camera = currentSensor->cam;

  CHECK(gl.camera.focalLength>0, "need a focal length greater zero!(not implemented for ortho yet)");
  int centerX = (W >> 1);
  int centerY = (H >> 1);
  double focal_x = 1./(gl.camera.focalLength*H);
  double focal_y = 1./(gl.camera.focalLength*H);

  uint i=0;
  for(int y=-centerY+1; y<=centerY; y++) for(int x=-centerX+1; x<=centerX; x++, i++) {
    double d = depth.elem(i);
    if(d>=0) {  //2^11-1
      pts(i, 0) = d*focal_x*x;
      pts(i, 1) = -d*focal_y*y;
      pts(i, 2) = -d;
    } else {
      pts(i, 0) = 0.;
      pts(i, 1) = 0.;
      pts(i, 2) = 1.;
    }
  }

  pts.reshape(H, W, 3);

  if(globalCoordinates){
    gl.camera.X.applyOnPointArray(pts);
  }
  done(__func__ );
}

void rai::CameraView::watch_PCL(const arr& pts, const byteA& rgb){

}

void rai::CameraView::updateCamera(){
  for(Sensor& sen:sensors){
    if(sen.frame>=0) sen.cam.X = K.frames(sen.frame)->X;
  }

  if(currentSensor){
    gl.background = currentSensor->backgroundImage;
    gl.backgroundZoom = (double)currentSensor->height/gl.background.d0;
    gl.camera = currentSensor->cam;
  }
}

void rai::CameraView::glDraw(OpenGL& gl) {
  if(renderMode==all || renderMode==visuals){
    glStandardScene(NULL, gl);
    gl.drawMode_idColor = false;
    if(renderMode==visuals){
      K.orsDrawVisualsOnly=true;
      K.orsDrawMarkers = false;
    }else{
      K.orsDrawVisualsOnly=false;
      K.orsDrawMarkers = true;
    }

    K.glDraw(gl);

    if(renderMode!=visuals){
      for(Sensor& sen:sensors){
        glDrawCamera(sen.cam);
        glDrawText(STRING("SENSOR " <<sen.name), 0., 0., 0.);
      }
    }
  }

  if(renderMode==seg){
    gl.setClearColors(1, 1, 1, 0);
    gl.background.clear();
    gl.drawMode_idColor = true;
    Geo_mesh_drawColors = false;
    K.orsDrawMarkers = false;
    K.orsDrawVisualsOnly=true;
    K.glDraw(gl);
    gl.drawMode_idColor = false;
    Geo_mesh_drawColors = true;
  }
}

void rai::CameraView::done(const char* _func_){
  if(watchComputations){
    gl.text = _func_;
    if(watchComputations==1) gl.update();
    else gl.watch();
  }
}


//===========================================================================

rai::Sim_CameraView::Sim_CameraView(Var<rai::KinematicWorld>& _kin,
                                    Var<byteA> _color,
                                    Var<floatA> _depth,
                                    double beatIntervalSec, const char* _cameraFrameName, bool _idColors, const byteA& _frameIDmap)
  : Thread("Sim_CameraView", beatIntervalSec),
    model(this, _kin, (beatIntervalSec<0.)),
    color(this, _color),
    depth(this, _depth),
    C(model.get()()){
  if(_cameraFrameName){
    C.addSensor(_cameraFrameName, _cameraFrameName);
    C.selectSensor(_cameraFrameName);
  }
  if(_idColors){
    C.renderMode = C.seg;
    if(!!_frameIDmap)
        C.frameIDmap = _frameIDmap;
    else{
        C.K.clear();;
                C.updateConfiguration(model.get());
    }
  }else{
    C.renderMode = C.visuals;
  }
  if(beatIntervalSec>=0.) threadLoop(); else threadStep();
}

rai::Sim_CameraView::~Sim_CameraView() {
  threadClose();
}

void rai::Sim_CameraView::step() {
  byteA img;
  floatA dep;
  C.updateConfiguration(model.get());
  C.computeImageAndDepth(img, dep);
  color.set() = img;
  depth.set() = dep;
}


arr rai::Sim_CameraView::getFxypxy(){
  auto sen = C.currentSensor;
  return ARR(sen->cam.focalLength*sen->height, sen->cam.focalLength*sen->height, .5*(sen->width-1.), .5*(sen->height-1.));
}
