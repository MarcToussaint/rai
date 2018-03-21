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
  ImageViewer(const char* img_name="rgb");
  ~ImageViewer();
  void open();
  void step();
  void close();
};

struct PointCloudViewer : Thread {
  struct sPointCloudViewer *s;
  Var<arr> pts;
  Var<byteA> rgb;
  PointCloudViewer(const char* pts_name="kinect_points", const char* rgb_name="kinect_rgb");
  ~PointCloudViewer();
  void open();
  void step();
  void close();
};

struct MeshAViewer : Thread {
  Var<MeshA> meshes;
  MeshA copy;
  struct OpenGL *gl;
  MeshAViewer(const char* meshes_name="visionDisplay");
  ~MeshAViewer();
  void open();
  void step();
  void close();
};
