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

  OrsViewer(const char* varname="modelWorld", bool computeCameraView=false)
    : Thread("OrsViewer", .2),
      modelWorld(this, varname, false),
      modelCameraView(this, "modelCameraView"),
      modelDepthView(this, "modelDepthView"),
      computeCameraView(computeCameraView){}
  ~OrsViewer(){ threadClose(); }
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

  void setConfigurations(const WorldL& cs){
    configurations.writeAccess();
    listResize(configurations(), cs.N);
    for(uint i=0;i<cs.N;i++) configurations()(i)->copy(*cs(i), true);
    configurations.deAccess();
  }
  void clear(){
    listDelete(configurations.set()());
  }

  OrsPathViewer(const char* varname, double beatIntervalSec=.2, int tprefix=0)
    : Thread("OrsPathViewer", beatIntervalSec),
      configurations(this, varname, true),
      tprefix(tprefix), writeToFiles(false){}
  ~OrsPathViewer(){ threadClose(); }
  void open();
  void step();
  void close() {}
};

//===========================================================================

struct OrsPoseViewer : Thread {
  mlr::Array<Access_typed<arr>*> poses; ///< poses to be watched
  //-- internal (private)
  OpenGL gl;
  WorldL copies;

  OrsPoseViewer(const StringA& poseVarNames, mlr::KinematicWorld& world, double beatIntervalSec=.2)
    : Thread("OrsPoseViewer", beatIntervalSec){
    for(const String& varname: poseVarNames){
      poses.append( new Access_typed<arr>(this, varname, true) );
      copies.append( new mlr::KinematicWorld() );
    }
    computeMeshNormals(world.shapes);
    for(mlr::KinematicWorld *w: copies) w->copy(world, true);
  }
  ~OrsPoseViewer(){}
  void open();
  void step();
  void close() {}
};

//===========================================================================

struct ComputeCameraView : Thread {
  Access_typed<mlr::KinematicWorld> modelWorld;
  Access_typed<byteA> cameraView;
  OpenGL gl;
  uint skipFrames, frame;
  ComputeCameraView(uint skipFrames=0)
    : Thread("OrsViewer"),
      modelWorld(this, "modelWorld", true),
      cameraView(this, "cameraView"),
      skipFrames(skipFrames), frame(0){}
  void open();
  void step();
  void close() {}
};

