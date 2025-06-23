/*  ------------------------------------------------------------------
    Copyright (c) 2011-2024 Marc Toussaint
    email: toussaint@tu-berlin.de

    This code is distributed under the MIT License.
    Please see <root-path>/LICENSE for details.
    --------------------------------------------------------------  */

#pragma once

#include "kin.h"
#include "proxy.h"

#include "../Gui/RenderData.h"
#include "../Core/thread.h"

namespace rai {

struct ConfigurationViewer : RenderData {
  shared_ptr<struct OpenGL> gl;
  intA frame2itemID;

  ~ConfigurationViewer();

  OpenGL& ensure_gl();
  void close_gl();

  void recopyMeshes(const FrameL& frames);
  ConfigurationViewer& updateConfiguration(const rai::Configuration& C, const FrameL& timeSlices={}, bool forceCopyMeshes=false);
  void setMotion(const uintA& frameIDs, const arr& _motion);
  void setMotion(rai::Configuration& C, const arr& path);

  int view(bool watch=false, const char* _text=0);
  int view_slice(uint t, bool watch=false);
  int view_play(bool watch=true, double delay=1., str saveVideoPath=nullptr); ///< display the trajectory; use "z.vid/" as vid prefix

  rai::Camera& displayCamera();   ///< access to the display camera to change the view
  byteA getRgb();
  floatA getDepth();
  void savePng(str saveVideoPath="z.vid/", int count=-1);

  void raiseWindow();
  void setWindow(const char* window_title, int w=400, int h=400);
  void glDraw(OpenGL&);
  void setCamera(rai::Frame* camFrame);
  void setCameraPose(const arr& pose);
  void focus(const arr& position, double heightAbs=1.);
  arr getCameraPose();

  void setupEventHandler(bool blockDefaultHandler);
  StringA getEvents();
  arr getEventCursor();
  uint getEventCursorObject();
  Mutex& getEventsMutex();

  //mimic a OpenGL, directly calling the same methods in its gl
  void _resetPressedKey();

private://draw data
  arr motion;
  bool abortPlay;
  uint pngCount=0;
  bool drawFrameLines=true;
  shared_ptr<struct ViewerEventHandler> eventHandler;
public:
  int drawSlice;
  String text;
  StringA sliceTexts;
  double phaseOffset=0., phaseFactor=-1.;
  bool nonThreaded=false;
};

struct ConfigurationViewerThread : Thread {
  Var<rai::Configuration> config;
  shared_ptr<ConfigurationViewer> viewer;
  ConfigurationViewerThread(const Var<rai::Configuration>& _config, double beatIntervalSec=-1.);
  ~ConfigurationViewerThread();
  void open();
  void step();
  void close();
};

}
