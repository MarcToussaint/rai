#include "cameraview.h"
#include <Kin/frame.h>

extern bool Geo_mesh_drawColors; //UGLY!!

rai::CameraView::CameraView(const rai::KinematicWorld& _K, bool _background, int _watchComputations)
  : K(_K), background(_background), watchComputations(_watchComputations) {

  gl.add(*this);
}

rai::CameraView::Sensor&rai::CameraView::addSensor(const char* name, const char* frameAttached, uint width, uint height, double focalLength, double orthoAbsHeight, const arr& zRange, const char* backgroundImageFile){
  Sensor& sen = sensors.append();
  sen.name = name;
  sen.frame = K.getFrameByName(frameAttached);
  rai::Camera& cam = sen.cam;
  sen.width=width;
  sen.height=height;

  cam.setZero();
  if(zRange.N) cam.setZRange(zRange(0), zRange(1));
  if(focalLength>0.) cam.setFocalLength(focalLength);
  if(orthoAbsHeight>0.) cam.setHeightAbs(orthoAbsHeight);

  cam.setWHRatio((double)width/height);
  done(__func__);
  return sen;
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

void rai::CameraView::computeImageAndDepth(byteA& image, arr& depth){
  renderMode=all;
  if(!background)
    gl.update(NULL, true, true, true);
  else
    gl.renderInBack(true, true, gl.width, gl.height);
  image = gl.captureImage;
  floatA glDepth = gl.captureDepth;
  flip_image(image);
  flip_image(glDepth);
  depth.resize(glDepth.d0, glDepth.d1);
  for(uint i=0; i<depth.N; i++){
    double d=glDepth.elem(i);
    if(d==1. || d==0.) depth.elem(i)=-1.;
    else depth.elem(i) = gl.camera.glConvertToTrueDepth(d);
  }
  done(__func__);
}

void rai::CameraView::computeKinectDepth(uint16A& kinect_depth, const arr& depth){
  kinect_depth.resize(depth.d0, depth.d1);
  for(uint i=0; i<depth.N; i++) kinect_depth.elem(i) = (uint16_t) (depth.elem(i) * 1000.);
}

void rai::CameraView::computePointCloud(arr& pts, const arr& depth, bool globalCoordinates){
  uint H=depth.d0, W=depth.d1;

  pts.resize(H*W, 3);

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

void rai::CameraView::computeSegmentation(byteA& segmentation){
  renderMode=seg;
  if(!background)
    gl.update(NULL, true, true, true);
  else
    gl.renderInBack(true, true, gl.width, gl.height);
  segmentation = gl.captureImage;
  flip_image(segmentation);
  done(__func__);
}

void rai::CameraView::watch_PCL(const arr& pts, const byteA& rgb){

}

void rai::CameraView::glDraw(OpenGL& gl) {
  for(Sensor& sen:sensors){
    if(sen.frame) sen.cam.X = sen.frame->X;
  }

  if(currentSensor){
    gl.background = currentSensor->backgroundImage;
    gl.backgroundZoom = (double)currentSensor->height/gl.background.d0;
    gl.camera = currentSensor->cam;
  }

  if(renderMode==all){
    glStandardScene(NULL);
    K.orsDrawMarkers = true;
    K.glDraw(gl);

    for(Sensor& sen:sensors){
      glDrawCamera(sen.cam);
      glDrawText(STRING("SENSOR " <<sen.name), 0., 0., 0.);
    }
  }

  if(renderMode==seg){
    gl.setClearColors(1, 1, 1, 0);
    gl.background.clear();
    gl.drawMode_idColor = true;
    Geo_mesh_drawColors = false;
    K.orsDrawMarkers = false;
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

