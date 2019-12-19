#pragma once

#include <Core/util.h>
#include "kin.h"
#include "proxy.h"

namespace rai {

struct ConfigurationViewer : GLDrawer {
  void setConfiguration(rai::Configuration& C, const char* text=0, bool watch=false);
  void setPath(ConfigurationL& Cs, const char* text=0, bool watch=false);
  void setPath(rai::Configuration& C, const arr& jointPath, const char* text=0, bool watch=false, bool full=true);
  void setPath(const arr& _framePath, const char* text=0, bool watch=false, bool full=true);
  bool playVideo(bool watch=false, double delay=1., const char* saveVideoPath=nullptr); ///< display the trajectory; use "vid/z." as vid prefix
  rai::Camera& displayCamera();   ///< access to the display camera to change the view
  void recopyMeshes(rai::Configuration& C);

  int update(bool watch=false);
  void glDraw(OpenGL &);
  void ensure_gl();

private://draw data
  MeshA meshes;
  ProxyA proxies;

  ptr<struct OpenGL> gl;
  arr framePath;
  rai::String drawText;
  int drawTimeSlice;
  bool drawFullPath;
  int tprefix;
  bool writeToFiles;
  rai::String text;
};

}
