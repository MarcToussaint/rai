/*  ------------------------------------------------------------------
    Copyright (c) 2011-2020 Marc Toussaint
    email: toussaint@tu-berlin.de

    This code is distributed under the MIT License.
    Please see <root-path>/LICENSE for details.
    --------------------------------------------------------------  */

#pragma once

#include "kin.h"
#include "proxy.h"
#include "../Core/thread.h"
#include "../Gui/opengl.h"
#include "../Geo/mesh.h"

//===========================================================================

void renderConfigurations(const ConfigurationL& cs, const char* filePrefix="vid/z.", int tprefix=0, int w=-1, int h=-1, rai::Camera* camera=nullptr);

//===========================================================================

struct KinViewer : Thread {
  Var<rai::Configuration> world;
  MeshA meshesCopy;
  ProxyA proxiesCopy;
  struct OpenGL* gl;
  int cameraFrameID=-1;
  KinViewer(const Var<rai::Configuration>& _kin, double beatIntervalSec=-1., const char* _cameraFrameName=nullptr);
  ~KinViewer();
  void open();
  void step();
  void close();
};

//===========================================================================

struct KinPathViewer : Thread {
  Var<ConfigurationL> configurations;
  //-- internal (private)
  rai::Configuration copy;
  struct OpenGL* gl;
  uint t;
  int tprefix;
  bool writeToFiles;
  rai::String text;

  void setConfigurations(const ConfigurationL& cs);
  void clear();

  KinPathViewer(const Var<ConfigurationL>& _configurations, double beatIntervalSec=.2, int tprefix=0);
  ~KinPathViewer();
  void open();
  void step();
  void close();
};

//===========================================================================

struct KinPoseViewer : Thread, GLDrawer {
  Var<rai::Configuration> model;
  Var<arr> frameState;
  MeshA meshesCopy;
  uint frameCount=0;
  //-- internal (private)
  OpenGL gl;

//  KinPoseViewer(const char* modelVarName, const StringA& poseVarNames, double beatIntervalSec=-1.);
  KinPoseViewer(Var<rai::Configuration>& _kin, const Var<arr>& _frameState, double beatIntervalSec=-1.);
  ~KinPoseViewer();

  void open();
  void step();
  void close();

  void glDraw(OpenGL& gl);
};

//===========================================================================

struct ComputeCameraView : Thread {
  Var<rai::Configuration> modelWorld;
  Var<byteA> cameraView;
  Var<uint16A> cameraDepth;
  Var<rai::Transformation> cameraFrame;

  //-- internal (private)
  OpenGL gl;
  rai::Configuration copy;
  bool getDepth;

  ComputeCameraView(const Var<rai::Configuration>& _modelWorld, double beatIntervalSec=-1.);
  ~ComputeCameraView();
  void open();
  void step();
  void close();
};

