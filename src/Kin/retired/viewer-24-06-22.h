/*  ------------------------------------------------------------------
    Copyright (c) 2011-2024 Marc Toussaint
    email: toussaint@tu-berlin.de

    This code is distributed under the MIT License.
    Please see <root-path>/LICENSE for details.
    --------------------------------------------------------------  */

#pragma once

#include "kin.h"
#include "proxy.h"

namespace rai {

struct ViewableConfigCopy : GLDrawer {
  Configuration C;
  shared_ptr<struct OpenGL> gl;

  ~ViewableConfigCopy();
  OpenGL& ensure_gl();
  void close_gl();
  void recopyMeshes(const Configuration& _C);
  void updateConfiguration(const rai::Configuration& newC);
};

struct ConfigurationViewer : ViewableConfigCopy {

  int setConfiguration(const Configuration& _C, const char* text=0, bool watch=false);
  int setPath(rai::Configuration& _C, const arr& jointPath, const char* text=0, bool watch=false, bool full=true);
  int setPath(const arr& _framePath, const char* text=0, bool watch=false, bool full=true);
  bool playVideo(const FrameL& timeSlices, bool watch=true, double delay=1., const char* saveVideoPath=nullptr); ///< display the trajectory; use "z.vid/" as vid prefix
  bool playVideo(bool watch=true, double delay=1., const char* saveVideoPath=nullptr); ///< display the trajectory; use "z.vid/" as vid prefix
  rai::Camera& displayCamera();   ///< access to the display camera to change the view
  byteA getRgb();
  floatA getDepth();
  void savePng(const char* saveVideoPath="z.vid/");

  rai::Configuration& getConfiguration() { return C; }

  int update(bool watch=false);
  void raiseWindow();
  void glDraw(OpenGL&);
  void setCamera(rai::Frame* cam);

  //mimic a OpenGL, directly calling the same methods in its gl
  int _update(const char* text=nullptr, bool nonThreaded=false);
  int _watch(const char* text=nullptr);
  void _add(GLDrawer& c);
  void _resetPressedKey();
  void clear();
 private://draw data

  arr framePath;
  FrameL drawSubFrames;
  int drawTimeSlice;
  bool drawFullPath;
  int tprefix;
  bool writeToFiles;
  uint pngCount=0;
 public:
  String drawText;
  bool drawFrameLines=true;
  double phaseOffset=0., phaseFactor=-1.;
};

}
