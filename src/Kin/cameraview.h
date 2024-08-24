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

struct CameraView : ConfigurationViewer {

  /*! describes a sensor from which we can take 'images' within the simulation (e.g.: kinect, suctionRingView, etc) */
  struct Sensor {
    rai::String name;
    rai::Camera cam;     ///< this includes the transformation X
    uint width=640, height=480;
    rai::Frame *frame=0;
    Sensor() {}
    rai::Transformation& pose() { return cam.X; }
    arr getFxycxy() { return cam.getFxycxy(width, height); }
  };

  //-- description of world configuration
  rai::Array<Sensor> sensors;  //the list of sensors

  enum RenderMode { all, seg, visuals };

  //-- run parameter
  Sensor* currentSensor=0;
  RenderMode renderMode=all;
  byteA frameIDmap;

  //-- evaluation outputs
  CameraView(const rai::Configuration& _C, bool _offscreen=true);
  ~CameraView() {}

  //-- loading the configuration: the meshes, the robot model, the tote, the sensors; all ends up in K
  Sensor& addSensor(rai::Frame* frame, uint width, uint height, double focalLength=-1., double orthoAbsHeight=-1., const arr& zRange= {}, const char* backgroundImageFile=0);
  Sensor& addSensor(rai::Frame* frame); //read everything from the frame attributes
  Sensor& selectSensor(rai::Frame* frame); //set the OpenGL sensor

  void computeImageAndDepth(byteA& image, floatA& depth);
  byteA computeSegmentationImage();
  uintA computeSegmentationID();

  arr getFxycxy() { CHECK(currentSensor, "no sensor selected yet"); return currentSensor->getFxycxy(); }

 private:
  void updateCamera();
};

//===========================================================================

struct Sim_CameraView : Thread {
  Var<rai::Configuration> model;

  //-- outputs
  Var<byteA> color;
  Var<floatA> depth;

  //-- internal
  CameraView V;

  Sim_CameraView(Var<rai::Configuration>& _kin,
                 Var<byteA> _color,
                 Var<floatA> _depth,
                 double beatIntervalSec=-1., const char* _cameraFrameName=nullptr, bool _idColors=false, const byteA& _frameIDmap= {});
  ~Sim_CameraView();

  void step();

  arr getFxycxy();
};

}
