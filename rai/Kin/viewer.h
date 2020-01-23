#pragma once

#include <Core/util.h>
#include "kin.h"
#include "proxy.h"

namespace rai {

struct ConfigurationViewer : GLDrawer {
  ~ConfigurationViewer();

  int setConfiguration(rai::Configuration& _C, const char* text=0, bool watch=false);
  void setPath(ConfigurationL& Cs, const char* text=0, bool watch=false);
  void setPath(rai::Configuration& _C, const arr& jointPath, const char* text=0, bool watch=false, bool full=true);
  void setPath(const arr& _framePath, const char* text=0, bool watch=false, bool full=true);
  bool playVideo(bool watch=false, double delay=1., const char* saveVideoPath=nullptr); ///< display the trajectory; use "vid/z." as vid prefix
  rai::Camera& displayCamera();   ///< access to the display camera to change the view
  void recopyMeshes(rai::Configuration& _C);

  int update(bool watch=false);
  void glDraw(OpenGL &);
  void ensure_gl();

  //mimic a OpenGL, directly calling the same methods in its gl
  int update(const char* text=nullptr, bool nonThreaded=false);
  int watch(const char* text=nullptr);
  void add(GLDrawer& c);
  void resetPressedKey();

private://draw data
  Configuration C;
  ProxyA proxies;

  ptr<struct OpenGL> gl;
  arr framePath;
  String drawText;
  int drawTimeSlice;
  bool drawFullPath;
  int tprefix;
  bool writeToFiles;
  String text;
};

}
