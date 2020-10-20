/*  ------------------------------------------------------------------
    Copyright (c) 2011-2020 Marc Toussaint
    email: toussaint@tu-berlin.de

    This code is distributed under the MIT License.
    Please see <root-path>/LICENSE for details.
    --------------------------------------------------------------  */

#pragma once

#ifdef RAI_OPENCV
#  undef COUNT
#  include <opencv2/opencv.hpp>
#  undef MIN
#  undef MAX
#endif

#include "../Core/thread.h"
#include "../Kin/kin.h"
#include "../Core/array.ipp"
#include "../Gui/opengl.h"
#include <map>

//===========================================================================
//
// fwd declarations
//

extern void loadPerception();

//-- Variables
struct Colors;
struct HoughLines;
struct Patching;
struct SURFfeatures;
struct PerceptionOutput;
class AudioPoller_PA;
class AudioWriter_libav;

template<class T>
struct GenericDisplayViewer : Thread {
  OpenGL* gl;
  Var<T> var;
  GenericDisplayViewer(const char* var_name)
    : Thread("GenericDisplayViewer", -1.)
    , gl(nullptr)
    , var(this, var_name, true) {}
  virtual void open() { gl = new OpenGL(STRING("GenericDisplayViewer '"<<var.data->name()<<'\'')); }
  virtual void step() {
    gl->background = var.get()->display;
    if(gl->height!= gl->background.d0 || gl->width!= gl->background.d1)
      gl->resize(gl->background.d1, gl->background.d0);
    gl->update();
  }
  virtual void close() { delete gl; }
};

struct VideoEncoder : Thread {
  unique_ptr<struct sVideoEncoder> self;
  bool is_rgb;
  double fps;
  Var<byteA> img;
  VideoEncoder(const Var<byteA>& _img)
    : Thread("VideoEncoder"), is_rgb(false), fps(30), img(this, _img, true) {}
  virtual ~VideoEncoder() {}

  virtual void open();
  virtual void step();
  virtual void close();
  /// set input packing (default is bgr)
  void set_rgb(bool is_rgb) { this->is_rgb = is_rgb; }
  /// set frames per second -- only effective before open
  void set_fps(double fps) { this->fps = fps; }
};

struct VideoEncoderX264 : Thread {
  unique_ptr<struct sVideoEncoderX> self;
  bool is_rgb;
  double fps;
  Var<byteA> img;
  VideoEncoderX264(const Var<byteA>& _img) : Thread("VideoEncoderX264"), is_rgb(false), img(this, _img, true) {}
  virtual ~VideoEncoderX264() {}

  virtual void open();
  virtual void step();
  virtual void close();
  /// set input packing (default is bgr)
  void set_rgb(bool is_rgb) { this->is_rgb = is_rgb; }
  /// set frames per second -- only effective before open
  void set_fps(double fps) { this->fps = fps; }
};
//===========================================================================
//
// Types
//

struct RigidObjectRepresentation {
  uint found;

  //-- 2d shape
  uint shapeType;
  arr shapeParamsL, shapeParamsR, shapePointsL, shapePointsR;

  //-- 3d information
  arr shapePoints3d;
  arr center3d, orsShapeParams;
  arr diagDiff;

  RigidObjectRepresentation() { found=0; }
};

typedef rai::Array<RigidObjectRepresentation*> RigidObjectRepresentationL;

//===========================================================================
//
// Variables
//

struct ColorChoice {
  byteA rgb;
  byteA hsv;
};

//===========================================================================

struct HoughLines {
#ifdef RAI_OPENCV
  std::vector<cv::Vec4i> lines;
#endif
  byteA display;
};
inline void operator>>(istream& is, HoughLines& hl) {}
inline void operator<<(ostream& os, const HoughLines& hl) {}

//===========================================================================

struct Patching {
  uintA patching;  //for each pixel an integer
  arr pch_cen;     //patch centers
  uintA pch_edges; //patch Delauney edges
  floatA pch_rgb;  //patch mean colors
  byteA display;
};
inline void operator>>(istream& is, Patching& hl) {}
inline void operator<<(ostream& os, const Patching& hl) {}

//===========================================================================

struct SURFfeatures {
#ifdef RAI_OPENCV
  std::vector<cv::KeyPoint> keypoints;
  std::vector<float> descriptors;
#endif
  byteA display;
};
inline void operator>>(istream& is, SURFfeatures& hl) {}
inline void operator<<(ostream& os, const SURFfeatures& hl) {}

//===========================================================================

/** The RigidObjectRepresentation List output of perception */
struct PerceptionOutput {
  rai::Array<RigidObjectRepresentation> objects;
  byteA display;
};
niyPipes(PerceptionOutput);

//-- Module declarations
#if 0
BEGIN_MODULE(ImageViewer)      VAR(byteA, img)       END_MODULE()
BEGIN_MODULE(PointCloudViewer) VAR(arr, pts)         VAR(arr, cols)        END_MODULE()
BEGIN_MODULE(OpencvCamera)     VAR(byteA, rgb)       std::map<int, double> properties; bool set(int prop, double value);  END_MODULE()
BEGIN_MODULE(CvtGray)          VAR(byteA, rgb)       VAR(byteA, gray)      END_MODULE()
BEGIN_MODULE(CvtHsv)           VAR(byteA, rgb)       VAR(byteA, hsv)       END_MODULE()
BEGIN_MODULE(HsvFilter)        VAR(byteA, hsv)       VAR(floatA, evi)      END_MODULE()
BEGIN_MODULE(MotionFilter)     VAR(byteA, rgb)       VAR(byteA, motion)    END_MODULE()
BEGIN_MODULE(DifferenceFilter) VAR(byteA, i1)        VAR(byteA, i2)        VAR(byteA, diffImage) END_MODULE()
BEGIN_MODULE(CannyFilter)      VAR(byteA, grayImage) VAR(byteA, cannyImage)       END_MODULE()
BEGIN_MODULE(Patcher)          VAR(byteA, rgbImage)  VAR(Patching, patchImage)    END_MODULE()
BEGIN_MODULE(SURFer)           VAR(byteA, grayImage) VAR(SURFfeatures, features)  END_MODULE()
BEGIN_MODULE(HoughLineFilter)  VAR(byteA, grayImage) VAR(HoughLines, houghLines)  END_MODULE()
BEGIN_MODULE(ShapeFitter)      VAR(floatA, eviL)     VAR(floatA, eviR)            VAR(PerceptionOutput, perc)      END_MODULE()
BEGIN_MODULE(AudioReader)    AudioPoller_PA* poller; VAR(byteA, pcms16ne2c) END_MODULE()
BEGIN_MODULE(AudioWriter)    AudioWriter_libav* writer; VAR(byteA, pcms16ne2c) END_MODULE()
#else

struct OpencvCamera : Thread {
  unique_ptr<struct sOpencvCamera> self;
  Var<byteA> rgb;
  std::map<int, double> properties; bool set(int prop, double status);
  OpencvCamera(const Var<byteA>& _rgb) : Thread(STRING("OpencvCamera_"<<_rgb.name()), 0.), rgb(this, _rgb) {}
  void open();
  void step();
  void close();
};

struct CvtGray : Thread {
  unique_ptr<struct sCvtGray> self;
  Var<byteA> rgb;
  Var<byteA> gray;
  std::map<int, double> properties; bool set(int prop, double status);
  CvtGray(const Var<byteA>& _rgb, const Var<byteA>& _gray)
    : Thread(STRING("CvtGray_"<<_rgb.name()), -1), rgb(this, _rgb, true), gray(this, _gray) {}
  void open();
  void step();
  void close();
};

struct MotionFilter : Thread {
  unique_ptr<struct sMotionFilter> self;
  Var<byteA> rgb;
  Var<byteA> motion;
  MotionFilter(const Var<byteA>& _rgb, const Var<byteA>& _motion)
    : Thread(STRING("MotionFilter_"<<_rgb.name()), -1), rgb(this, _rgb, true), motion(this, _motion) {}
  void open();
  void step();
  void close();
};

struct DifferenceFilter : Thread {
  unique_ptr<struct sDifferenceFilter> self;
  Var<byteA> i1;
  Var<byteA> i2;
  Var<byteA> diffImage;
  DifferenceFilter(const Var<byteA>& _i1, const Var<byteA>& _i2, const Var<byteA>* _diffImage)
    : Thread(STRING("DifferenceFilter_"<<_i1.name()), -1), i1(this, _i1, true), i2(this, _i2), diffImage(this, _diffImage) {}
  void open();
  void step();
  void close();
};

struct CannyFilter : Thread {
  unique_ptr<struct sCannyFilter> self;
  Var<byteA> grayImage;
  Var<byteA> cannyImage;
  CannyFilter(const Var<byteA>& _grayImage, const Var<byteA>& _cannyImage)
    : Thread(STRING("CannyFilter_"<<_grayImage.name()<<"_" <<_cannyImage.name()), -1),
      grayImage(this, _grayImage, true),
      cannyImage(this, _cannyImage) {}
  void open();
  void step();
  void close();
};

struct Patcher : Thread {
  unique_ptr<struct sPatcher> self;
  Var<byteA> rgbImage;
  Var<Patching> patchImage;
  Patcher(const Var<byteA>& _rgbImage, const Var<Patching>& _patchImage)
    : Thread(STRING("Patcher"<<_rgbImage.name()<<"_" <<_patchImage.name()), -1),
      rgbImage(this, _rgbImage, true),
      patchImage(this, _patchImage) {}
  void open();
  void step();
  void close();
};

//struct AllViewer : Thread {
//  Var<arr> kinect_points;
//  Var<arr> kinect_pointColors;
//  Var<PlaneA> planes_now;

//  rai::Mesh kinect;
//  PlaneA planes_now_copy;
//  OpenGL gl;

//  AllViewer()
//    : Thread("AllViewer", .1),
//      kinect_points(this, "kinect_points"),
//      kinect_pointColors(this, "kinect_pointColors"),
//      planes_now(this, "planes_now"),
//      gl("AllViewer"){}
//  ~AllViewer(){}
//  void open();
//  void step();
//  void close() {}

//};

// macro for a most standard declaration of a module
#define BEGIN_MODULE(name) \
  struct name : Thread { \
    struct s##name *s; \
    name(): Thread(#name), s(nullptr) {} \
    virtual void open(); \
    virtual void step(); \
    virtual void close();

#define END_MODULE() };

//BEGIN_MODULE(ImageViewer)      VAR(byteA, img)       END_MODULE()
//BEGIN_MODULE(PointCloudViewer) VARlisten(arr, kinect_points)         VAR(arr, kinect_pointColors)        END_MODULE()
//BEGIN_MODULE(OpencvCamera)     VAR(byteA, rgb)       std::map<int,double> properties; bool set(int prop, double value);  END_MODULE()
//BEGIN_MODULE(CvtGray)          VAR(byteA, rgb)       VAR(byteA, gray)      END_MODULE()
//BEGIN_MODULE(CvtHsv)           VARlisten(byteA, rgb)       VAR(byteA, hsv)       END_MODULE()
//BEGIN_MODULE(HsvFilter)        VARlisten(byteA, hsv)       VAR(floatA, evi)      END_MODULE()
//BEGIN_MODULE(MotionFilter)     VAR(byteA, rgb)       VAR(byteA, motion)    END_MODULE()
//BEGIN_MODULE(DifferenceFilter) VAR(byteA, i1)        VAR(byteA, i2)        VAR(byteA, diffImage) END_MODULE()
//BEGIN_MODULE(CannyFilter)      VAR(byteA, grayImage) VAR(byteA, cannyImage)       END_MODULE()
//BEGIN_MODULE(Patcher)          VARlisten(byteA, rgbImage)  VAR(Patching, patchImage)    END_MODULE()
//BEGIN_MODULE(SURFer)           VARlisten(byteA, grayImage) VAR(SURFfeatures, features)  END_MODULE()
//BEGIN_MODULE(HoughLineFilter)  VARlisten(byteA, grayImage) VAR(HoughLines, houghLines)  END_MODULE()
//BEGIN_MODULE(ShapeFitter)      VARlisten(floatA, eviL)     VAR(floatA, eviR)            VAR(PerceptionOutput, perc)      END_MODULE()
//BEGIN_MODULE(AudioReader)    AudioPoller_PA *poller; VAR(byteA, pcms16ne2c) END_MODULE()
//BEGIN_MODULE(AudioWriter)    AudioWriter_libav *writer; VAR(byteA, pcms16ne2c) END_MODULE()

#endif

//===========================================================================
//
// PRELIMINARY
//

//BEGIN_MODULE(ColorPicker)
//  VAR(ColorChoice, colorChoice);
//END_MODULE()

//TODO Johannes ProcessL newPointcloudProcesses();
//TODO Johannes VariableL newPointcloudVariables();

//TODO: where should this go? maybe ors?

/*

MEMO on what should/could be elements (Variables) of a perception system

** image level

cam image

grey image
hsv image

hsv-filter-values
hsv-filter-images

background
diff

depth

self-projection-mask
& cropped images

object-projections

Patch

** point cloud level

point cloud

** features level

SURFfeatures

HoughLines

Canny

** primitives level

(RANSAC type methods)

spheres

planes

** rigid body level

list of moving (or non-moving) rigid body hypothesis
(workspace for a tracker -- where does he get the likelihood function from?)

** shape level

list of shapes

*/
