/*  ------------------------------------------------------------------
    Copyright (c) 2017 Marc Toussaint
    email: marc.toussaint@informatik.uni-stuttgart.de

    This code is distributed under the MIT License.
    Please see <root-path>/LICENSE for details.
    --------------------------------------------------------------  */

#pragma once

#include <Core/thread.h>

namespace rai { struct Mesh; }
typedef rai::Array<rai::Mesh> MeshA;

struct ImageViewer : Thread {
  struct sImageViewer *s;
  Var<byteA> img;
  bool flipImage = false;
  ImageViewer(const Var<byteA>& _img, double beatIntervalSec=-1.);
  ~ImageViewer();
  void open();
  void step();
  void close();
};

struct ImageViewerFloat : Thread {
  ptr<struct OpenGL> gl;
  Var<floatA> img;
  bool flipImage = false;
  float scale;
  ImageViewerFloat(const Var<floatA>& _img, double beatIntervalSec=-1., float _scale=256.);
  ~ImageViewerFloat();
  void step();
};

struct ImageViewerCallback {
  struct OpenGL *gl=0;
  Var<byteA> img;
  bool flipImage = false;
  ImageViewerCallback(const Var<byteA>& _img);
  ~ImageViewerCallback();
  void call(Var_base *v);
};

struct PointCloudViewer : Thread {
  struct sPointCloudViewer *s;
  Var<arr> pts;
  Var<byteA> rgb;
  PointCloudViewer(const char* pts_name="kinect_points", const char* rgb_name="kinect_rgb");
  PointCloudViewer(const Var<arr>& _pts, const Var<byteA>& _rgb, double beatIntervalSec=-1.);
  ~PointCloudViewer();
  void open();
  void step();
  void close();
};

struct PointCloudViewerCallback {
  struct sPointCloudViewer *s=0;
  Var<arr> pts;
  Var<byteA> rgb;
  PointCloudViewerCallback(const Var<arr>& _pts, const Var<byteA>& _rgb);
  ~PointCloudViewerCallback();
  void call(Var_base *v);
};

struct MeshAViewer : Thread {
  Var<MeshA> meshes;
  MeshA copy;
  struct OpenGL *gl;
  MeshAViewer(const Var<MeshA>& _meshes);
  ~MeshAViewer();
  void open();
  void step();
  void close();
};

struct PlotViewer : Thread, GLDrawer {
  Var<arr> data;
  arr plot, x0;
  uint T=100;
  struct OpenGL *gl;
  PlotViewer(const Var<arr>& _data, double beatIntervalSec=-1.);
  ~PlotViewer();
  void open();
  void step();
  void close();
  void glDraw(struct OpenGL&);
};
