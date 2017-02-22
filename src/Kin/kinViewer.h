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

//===========================================================================

struct OrsViewer : Thread {
  Access_typed<mlr::KinematicWorld> modelWorld;
  //-- outputs
  Access_typed<byteA> modelCameraView;
  Access_typed<floatA> modelDepthView;
  //-- internal (private)
  mlr::KinematicWorld copy;
  bool computeCameraView;

  OrsViewer(const char* varname="modelWorld", double beatIntervalSec=-1., bool computeCameraView=false);
  ~OrsViewer();
  void open();
  void step();
  void close() {}
};

//===========================================================================

struct OrsPathViewer : Thread {
  Access_typed<WorldL> configurations;
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
  Access_typed<mlr::KinematicWorld> modelWorld;
  mlr::Array<Access_typed<arr>*> poses; ///< poses to be watched
  //-- internal (private)
  OpenGL gl;
  mlr::KinematicWorld copy;
  WorldL copies;

  OrsPoseViewer(const char* modelVarName, const StringA& poseVarNames, double beatIntervalSec=-1.);
  ~OrsPoseViewer();

  void recopyKinematics();

  void open();
  void step();
  void close();
};

//===========================================================================

struct ComputeCameraView : Thread {
  Access_typed<mlr::KinematicWorld> modelWorld;
  Access_typed<byteA> cameraView;
  //-- internal (private)
  OpenGL gl;
  mlr::KinematicWorld copy;
  uint skipFrames, frame;

  ComputeCameraView(uint skipFrames=0);
  ~ComputeCameraView();
  void open();
  void step();
  void close();
};

