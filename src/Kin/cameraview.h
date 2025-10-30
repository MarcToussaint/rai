/*  ------------------------------------------------------------------
    Copyright (c) 2011-2024 Marc Toussaint
    email: toussaint@tu-berlin.de

    This code is distributed under the MIT License.
    Please see <root-path>/LICENSE for details.
    --------------------------------------------------------------  */

#pragma once

#include "kin.h"
#include "viewer.h"
#include "../Gui/opengl.h"

namespace rai {

struct DepthNoiseOptions {
  RAI_PARAM("DepthNoise/", double, binocular_baseline, .03)
  RAI_PARAM("DepthNoise/", int, depth_smoothing, 1)
  RAI_PARAM("DepthNoise/", double, noise_all, .02)
  RAI_PARAM("DepthNoise/", double, noise_wide, 4.)
  RAI_PARAM("DepthNoise/", double, noise_local, .4)
  RAI_PARAM("DepthNoise/", double, noise_pixel, .04)
};

struct CameraView : ConfigurationViewer {

  /*! a camera attached (and defined by the attributes of) a Frame */
  struct CameraFrame {
    rai::Frame& frame;
    rai::Camera cam;
    rai::Vector offset=0;
    CameraFrame(rai::Frame& _frame) : frame(_frame) {}
  };

  //-- description of world configuration
  rai::Array<shared_ptr<CameraFrame>> cameras;  //the list of sensors

  enum RenderMode { all, seg, visuals };

  //-- run parameter
  shared_ptr<CameraFrame> currentCamera;
  RenderMode renderMode=all;
  byteA frameIDmap;
  shared_ptr<DepthNoiseOptions> opt;

  //-- evaluation outputs
  CameraView(const rai::Configuration& _C, bool _offscreen=true);
  ~CameraView() {}

  //-- loading the configuration: the meshes, the robot model, the tote, the sensors; all ends up in K
  CameraFrame& setCamera(rai::Frame* frame, uint width, uint height, double focalLength=-1., double orthoAbsHeight=-1., const arr& zRange= {}, const char* backgroundImageFile=0);
  CameraFrame& setCamera(rai::Frame* frame); //read everything from the frame attributes
  CameraFrame& selectSensor(rai::Frame* frame); //set the OpenGL sensor

  void computeImageAndDepth(byteA& image, floatA& depth, bool _simulateDepthNoise=false);
  byteA computeSegmentationImage();
  uintA computeSegmentationID();

  arr getFxycxy() { CHECK(currentCamera, "no sensor selected yet"); return currentCamera->cam.getFxycxy(); }

 private:
  void updateCamera();
};

//===========================================================================

void simulateDepthNoise(floatA& depth, const floatA& depth2, const arr& fxycxy, shared_ptr<DepthNoiseOptions> opt);

}
