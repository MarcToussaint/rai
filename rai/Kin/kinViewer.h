/*  ------------------------------------------------------------------
    Copyright (c) 2017 Marc Toussaint
    email: marc.toussaint@informatik.uni-stuttgart.de

    This code is distributed under the MIT License.
    Please see <root-path>/LICENSE for details.
    --------------------------------------------------------------  */

#pragma once

#include <Core/thread.h>
#include <Gui/opengl.h>
#include "kin.h"
#include <Geo/mesh.h>
#include "proxy.h"

//===========================================================================

void renderConfigurations(const WorldL& cs, const char* filePrefix="vid/z.", int tprefix=0, int w=-1, int h=-1, rai::Camera *camera=NULL);

//===========================================================================

struct KinViewer : Thread {
  Var<rai::KinematicWorld> world;
  MeshA meshesCopy;
  ProxyA proxiesCopy;
  struct OpenGL *gl;
  int cameraFrameID=-1;
  KinViewer(const Var<rai::KinematicWorld>& _kin, double beatIntervalSec=-1., const char* _cameraFrameName=NULL);
  ~KinViewer();
  void open();
  void step();
  void close();
};

//===========================================================================

struct KinPathViewer : Thread {
  Var<WorldL> configurations;
  //-- internal (private)
  rai::KinematicWorld copy;
  struct OpenGL *gl;
  uint t;
  int tprefix;
  bool writeToFiles;
  rai::String text;
  
  void setConfigurations(const WorldL& cs);
  void clear();
  
  KinPathViewer(const Var<WorldL>& _configurations, double beatIntervalSec=.2, int tprefix=0);
  ~KinPathViewer();
  void open();
  void step();
  void close();
};

//===========================================================================

struct KinPoseViewer : Thread, GLDrawer {
  Var<rai::KinematicWorld> model;
  Var<arr> frameState; ///< poses to be watched
  uint frameCount=0;
  //-- internal (private)
  OpenGL gl;
  rai::KinematicWorld copy;
  WorldL copies;
  
//  KinPoseViewer(const char* modelVarName, const StringA& poseVarNames, double beatIntervalSec=-1.);
  KinPoseViewer(Var<rai::KinematicWorld>& _kin, const Var<arr>& _frameState, double beatIntervalSec=-1.);
  ~KinPoseViewer();
  
  void recopyKinematics(const rai::KinematicWorld& world=NoWorld);
  
  void open();
  void step();
  void close();

  void glDraw(OpenGL &gl);
};

//===========================================================================

struct ComputeCameraView : Thread {
  Var<rai::KinematicWorld> modelWorld;
  Var<byteA> cameraView;
  Var<uint16A> cameraDepth;
  Var<rai::Transformation> cameraFrame;
  
  //-- internal (private)
  OpenGL gl;
  rai::KinematicWorld copy;
  bool getDepth;
  
  ComputeCameraView(const Var<rai::KinematicWorld>& _modelWorld, double beatIntervalSec=-1.);
  ~ComputeCameraView();
  void open();
  void step();
  void close();
};

