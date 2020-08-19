/*  ------------------------------------------------------------------
    Copyright (c) 2011-2020 Marc Toussaint
    email: toussaint@tu-berlin.de

    This code is distributed under the MIT License.
    Please see <root-path>/LICENSE for details.
    --------------------------------------------------------------  */

#pragma once

#include "kin.h"
#include "../Gui/opengl.h"

namespace rai {

struct CameraView : GLDrawer {

  /*! describes a sensor from which we can take 'images' within the simulation (e.g.: kinect, suctionRingView, etc) */
  struct Sensor {
    rai::String name;
    rai::Camera cam;     ///< this includes the transformation X
    uint width=640, height=480;
    byteA backgroundImage;
    int frame=-1;
    Sensor() {}
    rai::Transformation& pose() { return cam.X; }
  };

  //-- description of world configuration
  rai::Configuration C;        //COPY of the configuration
  rai::Array<Sensor> sensors;  //the list of sensors

  enum RenderMode { all, seg, visuals };
  OpenGL gl;

  //-- run parameter
  Sensor* currentSensor=0;
  int watchComputations=0;
  RenderMode renderMode=all;
  byteA frameIDmap;

  //-- evaluation outputs
  CameraView(const rai::Configuration& _C, bool _offscreen=true, int _watchComputations=0);
  ~CameraView() {}

  //-- loading the configuration: the meshes, the robot model, the tote, the sensors; all ends up in K
  Sensor& addSensor(const char* name, const char* frameAttached, uint width, uint height, double focalLength=-1., double orthoAbsHeight=-1., const arr& zRange= {}, const char* backgroundImageFile=0);
  Sensor& addSensor(const char* frameAttached); //read everything from the frame attributes

  Sensor& selectSensor(const char* sensorName); //set the OpenGL sensor

  void updateConfiguration(const Configuration& newC);

  //-- compute/analyze a camera perspective (stored in classes' output fields)
  void computeImageAndDepth(byteA& image, floatA& depth);
  void computeKinectDepth(uint16A& kinect_depth, const arr& depth);
  void computePointCloud(arr& pts, const floatA& depth, bool globalCoordinates=true); // point cloud (rgb of every point is given in image)
  void computeSegmentation(byteA& segmentation);     // -> segmentation

  //-- displays
  void watch_PCL(const arr& pts, const byteA& rgb);

  void glDraw(OpenGL& gl);

 private:
  void updateCamera();
  void done(const char* _code_);
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
                 double beatIntervalSec=-1., const char* _cameraFrameName=nullptr, bool _idColors=false, const byteA& _frameIDmap=NoByteA);
  ~Sim_CameraView();

  void step();

  arr getFxypxy();
};

}
