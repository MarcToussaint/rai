/*  ------------------------------------------------------------------
    Copyright (c) 2011-2024 Marc Toussaint
    email: toussaint@tu-berlin.de

    This code is distributed under the MIT License.
    Please see <root-path>/LICENSE for details.
    --------------------------------------------------------------  */

#include "cameraview.h"
#include "frame.h"
#include "../Geo/depth2PointCloud.h"

namespace rai {

//===========================================================================

CameraView::CameraView(const Configuration& _C, bool _offscreen) {
  updateConfiguration(_C);
  gl = make_shared<OpenGL>("CameraView", 640, 480, _offscreen);
  gl->camera.setDefault();
  gl->add(this);
}

CameraView::CameraFrame& CameraView::setCamera(Frame* frame, uint width, uint height, double focalLength, double orthoAbsHeight, const arr& zRange, const char* backgroundImageFile) {
  currentCamera = make_shared<CameraFrame>(*frame);
  cameras.append(currentCamera);
  Camera& cam = currentCamera->cam;

  cam.setZero();
  cam.setWidthHeight(width, height);
  if(zRange.N) cam.setZRange(zRange(0), zRange(1));
  if(focalLength>0.) cam.setFocalLength(focalLength);
  if(orthoAbsHeight>0.) cam.setHeightAbs(orthoAbsHeight);

  cam.X = currentCamera->frame.ensure_X();
  gl->resize(cam.width, cam.height);
  return *currentCamera;
}

CameraView::CameraFrame& CameraView::setCamera(Frame* frame) {
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

  return setCamera(frame, width, height, focalLength, orthoAbsHeight, zRange);
}

CameraView::CameraFrame& CameraView::selectSensor(Frame *frame) {
  CHECK(frame, "you need to specify a frame, nullptr not allowed");
  bool found=false;
  for(shared_ptr<CameraFrame>& c:cameras) if(&c->frame==frame) { currentCamera=c; found=true; break; }
  if(!found) {
    return setCamera(frame);
  }
  gl->resize(currentCamera->cam.width, currentCamera->cam.height);
  return *currentCamera;
}

void CameraView::computeImageAndDepth(byteA& image, floatA& depth, bool _simulateDepthNoise) {
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
  {
    depth = gl->captureDepth;
    flip_image(depth);
    for(float& d:depth) {
      if(d==1.f || d==0.f) d=-1.f;
      else d = gl->camera.glConvertToTrueDepth(d);
    }
  }

  if(_simulateDepthNoise){
    CHECK(currentCamera, "depth noise only works for a frame-attached camera -- use setCamera")
    if(!opt) opt = make_shared<DepthNoiseOptions>();
    currentCamera->offset.set(opt->binocular_baseline, .0, .0);
    byteA image2;
    floatA depth2;
    computeImageAndDepth(image2, depth2, false);
    rai::simulateDepthNoise(depth, depth2, currentCamera->offset.x, currentCamera->cam.getFxycxy(), opt);
    currentCamera->offset.setZero();
  }
}

byteA CameraView::computeSegmentationImage() {
  updateCamera();
  renderFlatColors=true;
  gl->renderInBack();
  renderFlatColors=false;
  byteA seg = gl->captureImage;
  flip_image(seg);
  return seg;
}

uintA CameraView::computeSegmentationID() {
  byteA seg = computeSegmentationImage();
  uintA segmentation(seg.d0, seg.d1);
  for(uint i=0; i<segmentation.N; i++) {
    segmentation.elem(i) = color2id(seg.p+3*i);
  }
  return segmentation;
}

void CameraView::updateCamera() {
  for(shared_ptr<CameraFrame>& cam:cameras){
    cam->cam.X = cam->frame.ensure_X();
    if(!cam->offset.isZero) cam->cam.X.appendRelativeTranslation(cam->offset);
  }
  if(currentCamera) gl->camera = currentCamera->cam;
}

//===========================================================================

void simulateDepthNoise(floatA& depth, const floatA& depth2, double offset, const arr& fxycxy, shared_ptr<DepthNoiseOptions> opt){
  //-- wierd noise
  floatA noise = rai::convert<float>(opt->noise_wide*randn(depth.d0, depth.d1));
  //smooth noise
  for(uint k=0;k<3;k++){
    noise = rai::integral(noise);
    noise = rai::differencing(noise, 20); //PARAMETER
  }
  //local noise
  noise += rai::convert<float>(opt->noise_local*randn(depth.d0, depth.d1));
  for(uint k=0;k<3;k++){
    noise = rai::integral(noise);
    noise = rai::differencing(noise, 3); //PARAMETER
  }
  //pixel noise
  noise += rai::convert<float>(opt->noise_pixel*randn(depth.d0, depth.d1));

  // { OpenGL gl;  noise *= 255.f;  gl.watchImage(noise, true); }

  //-- smoothed depth
  for(int k=0;k<opt->depth_smoothing;k++){
    depth = rai::integral(depth);
    depth = rai::differencing(depth, 5); //PARAMETER
  }

  //-- shadows
  // byteA shadowImg(depth.d0, depth.d1);
  // shadowImg = 255;
  for(uint i=0;i<depth.d0;i++){
    for(uint j=0;j<depth.d1; j++){
      float& d = depth(i,j);
      int j2 = int(j) - fxycxy(0)*d*offset;
      bool shadow=false;
      if(j2<0) shadow=true;
      if(j2>=int(depth.d1)) shadow=true;
      else if(fabs(depth2(i, j2) - d)>.05) shadow=true;  //PARAMETER

      d += opt->noise_all*noise(i,j);

      if(shadow){
        d=-1.;
        // shadowImg(i,j) = 0;
      }
    }
  }

  // { OpenGL gl;  gl.watchImage(shadowImg, true); }
}

} //namespace
