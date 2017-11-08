/*  ------------------------------------------------------------------
    Copyright 2016 Marc Toussaint
    email: marc.toussaint@informatik.uni-stuttgart.de
    
    This program is free software: you can redistribute it and/or modify
    it under the terms of the GNU General Public License as published by
    the Free Software Foundation, either version 3 of the License, or (at
    your option) any later version. This program is distributed without
    any warranty. See the GNU General Public License for more details.
    You should have received a COPYING file of the full GNU General Public
    License along with this program. If not, see
    <http://www.gnu.org/licenses/>
    --------------------------------------------------------------  */


#pragma once

#include <Core/thread.h>
#include <Gui/opengl.h>
#include "kin.h"
#include <Geo/mesh.h>

//===========================================================================

void renderConfigurations(const WorldL& cs, const char* filePrefix="vid/z.path.", int tprefix=0, int w=-1, int h=-1, mlr::Camera *camera=NULL);

//===========================================================================

struct OrsViewer_old : Thread {
  Access<mlr::KinematicWorld> modelWorld;
  //-- outputs
  Access<byteA> modelCameraView;
  Access<floatA> modelDepthView;
  //-- internal (private)
  mlr::KinematicWorld copy;
  bool computeCameraView;

  OrsViewer_old(const char* varname="modelWorld", double beatIntervalSec=-1., bool computeCameraView=false);
  ~OrsViewer_old();
  void open();
  void step();
  void close() {}
};

//===========================================================================

struct OrsViewer : Thread {
  Access<mlr::KinematicWorld> world;
  MeshA meshesCopy;
  ProxyL proxiesCopy;
  struct OpenGL *gl;
  OrsViewer(const char* world_name="modelWorld", double beatIntervalSec=-1.);
  ~OrsViewer();
  void open();
  void step();
  void close();
};

//===========================================================================

struct OrsPathViewer : Thread {
  Access<WorldL> configurations;
  //-- internal (private)
  mlr::KinematicWorld copy;
  uint t;
  int tprefix;
  bool writeToFiles;

  void setConfigurations(const WorldL& cs);
  void clear();

  OrsPathViewer(const char* varname, double beatIntervalSec=.2, int tprefix=0);
  ~OrsPathViewer();
  void open();
  void step();
  void close(){}
};

//===========================================================================

struct OrsPoseViewer : Thread {
  Access<mlr::KinematicWorld> modelWorld;
  mlr::Array<Access<arr>*> poses; ///< poses to be watched
  //-- internal (private)
  OpenGL gl;
  mlr::KinematicWorld copy;
  WorldL copies;

  OrsPoseViewer(const char* modelVarName, const StringA& poseVarNames, double beatIntervalSec=-1.);
  ~OrsPoseViewer();

  void recopyKinematics(const mlr::KinematicWorld& world=NoWorld);

  void open();
  void step();
  void close();
};

//===========================================================================

struct ComputeCameraView : Thread {
  Access<mlr::KinematicWorld> modelWorld;
  Access<byteA> cameraView;
  Access<uint16A> cameraDepth;
  Access<mlr::Transformation> cameraFrame;

  //-- internal (private)
  OpenGL gl;
  mlr::KinematicWorld copy;
  bool getDepth;

  ComputeCameraView(double beatIntervalSec=-1., const char* modelWorld_name="modelWorld");
  ~ComputeCameraView();
  void open();
  void step();
  void close();
};

