/*  ------------------------------------------------------------------
    Copyright (c) 2011-2024 Marc Toussaint
    email: toussaint@tu-berlin.de

    This code is distributed under the MIT License.
    Please see <root-path>/LICENSE for details.
    --------------------------------------------------------------  */

#pragma once

#include "kin.h"
#include "proxy.h"

#include <Gui/RenderScene.h>

namespace rai {

struct ViewableConfigCopy : GLDrawer {
  Configuration C;

  ~ViewableConfigCopy();
};

struct ConfigurationViewer : RenderScene {
  shared_ptr<struct OpenGL> gl;
  intA frame2objID;

  ~ConfigurationViewer();

  OpenGL& ensure_gl();
  void close_gl();
  void recopyMeshes(const Configuration& C);
  void updateConfiguration(const rai::Configuration& C);

  int setConfiguration(const Configuration& _C, const char* _text=0, bool watch=false, const FrameL& timeSlices={});
  bool playVideo(const FrameL& timeSlices, bool watch=true, double delay=1., const char* saveVideoPath=nullptr); ///< display the trajectory; use "z.vid/" as vid prefix
  int view_slice(uint t, bool watch=false);

  rai::Camera& displayCamera();   ///< access to the display camera to change the view
  byteA getRgb();
  floatA getDepth();
  void savePng(const char* saveVideoPath="z.vid/");

  int update(bool watch=false);
  void raiseWindow();
  void glDraw(OpenGL&);
  void setCamera(rai::Frame* cam);

  //mimic a OpenGL, directly calling the same methods in its gl
  int _update(const char* text=nullptr, bool nonThreaded=false);
  int _watch(const char* text=nullptr);
  void _add(GLDrawer& c);
  void _resetPressedKey();

private://draw data
  FrameL slices;
  int drawSlice;
  bool abortPlay;
  uint pngCount=0;
  bool drawFrameLines=true;
public:
  String text;
  StringA sliceTexts;
  double phaseOffset=0., phaseFactor=-1.;
};

}
