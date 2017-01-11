#ifndef MLR_perception_h
#define MLR_perception_h

#ifdef MLR_OPENCV
#  undef COUNT
#  include <opencv2/opencv.hpp>
#  undef MIN
#  undef MAX
#endif

#include <Core/thread.h>
#include <Kin/kin.h>
#include <Core/array.tpp>
#include <Gui/opengl.h>
#include <map>

#include "plane.h"

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
  OpenGL *gl;
  Access_typed<T> var;
  GenericDisplayViewer(const char* var_name)
    : Thread("GenericDisplayViewer", -1.)
    , gl(NULL)
    , var(this, var_name, true){}
  virtual void open(){ gl = new OpenGL(STRING("ImageViewer '"<<var.data->name()<<'\'')); }
  virtual void step(){
    gl->background = var.get()->display;
    if(gl->height!= gl->background.d0 || gl->width!= gl->background.d1)
      gl->resize(gl->background.d1, gl->background.d0);
    gl->update();
  }
  virtual void close(){ delete gl; }
};

struct VideoEncoder : Thread {
  struct sVideoEncoder *s;
  bool is_rgb;
  double fps;
  ACCESSlisten(byteA, img)
  VideoEncoder() : Thread("VideoEncoder"), is_rgb(false), fps(30) {}
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
   struct sVideoEncoderX264 *s;
   bool is_rgb;
   double fps;
   ACCESSlisten(byteA, img)
   VideoEncoderX264() : Thread("VideoEncoderX264"), is_rgb(false) {}
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

  RigidObjectRepresentation(){ found=0; }
};

typedef mlr::Array<RigidObjectRepresentation*> RigidObjectRepresentationL;


//===========================================================================
//
// Variables
//

struct ColorChoice{
  byteA rgb;
  byteA hsv;
};

//===========================================================================

struct HoughLines {
#ifdef MLR_OPENCV
  std::vector<cv::Vec4i> lines;
#endif
  byteA display;
};
inline void operator>>(istream& is,HoughLines& hl){}
inline void operator<<(ostream& os,const HoughLines& hl){}

//===========================================================================

struct Patching {
  uintA patching;  //for each pixel an integer
  arr pch_cen;     //patch centers
  uintA pch_edges; //patch Delauney edges
  floatA pch_rgb;  //patch mean colors
  byteA display;
};
inline void operator>>(istream& is,Patching& hl){}
inline void operator<<(ostream& os,const Patching& hl){}

//===========================================================================

struct SURFfeatures {
#ifdef MLR_OPENCV
  std::vector<cv::KeyPoint> keypoints;
  std::vector<float> descriptors;
#endif
  byteA display;
};
inline void operator>>(istream& is,SURFfeatures& hl){}
inline void operator<<(ostream& os,const SURFfeatures& hl){}

//===========================================================================

/** The RigidObjectRepresentation List output of perception */
struct PerceptionOutput {
  mlr::Array<RigidObjectRepresentation> objects;
  byteA display;
};
niyPipes(PerceptionOutput);


//-- Module declarations
#if 0
BEGIN_MODULE(ImageViewer)      ACCESS(byteA, img)       END_MODULE()
BEGIN_MODULE(PointCloudViewer) ACCESS(arr, pts)         ACCESS(arr, cols)        END_MODULE()
BEGIN_MODULE(OpencvCamera)     ACCESS(byteA, rgb)       std::map<int,double> properties; bool set(int prop, double value);  END_MODULE()
BEGIN_MODULE(CvtGray)          ACCESS(byteA, rgb)       ACCESS(byteA, gray)      END_MODULE()
BEGIN_MODULE(CvtHsv)           ACCESS(byteA, rgb)       ACCESS(byteA, hsv)       END_MODULE()
BEGIN_MODULE(HsvFilter)        ACCESS(byteA, hsv)       ACCESS(floatA, evi)      END_MODULE()
BEGIN_MODULE(MotionFilter)     ACCESS(byteA, rgb)       ACCESS(byteA, motion)    END_MODULE()
BEGIN_MODULE(DifferenceFilter) ACCESS(byteA, i1)        ACCESS(byteA, i2)        ACCESS(byteA, diffImage) END_MODULE()
BEGIN_MODULE(CannyFilter)      ACCESS(byteA, grayImage) ACCESS(byteA, cannyImage)       END_MODULE()
BEGIN_MODULE(Patcher)          ACCESS(byteA, rgbImage)  ACCESS(Patching, patchImage)    END_MODULE()
BEGIN_MODULE(SURFer)           ACCESS(byteA, grayImage) ACCESS(SURFfeatures, features)  END_MODULE()
BEGIN_MODULE(HoughLineFilter)  ACCESS(byteA, grayImage) ACCESS(HoughLines, houghLines)  END_MODULE()
BEGIN_MODULE(ShapeFitter)      ACCESS(floatA, eviL)     ACCESS(floatA, eviR)            ACCESS(PerceptionOutput, perc)      END_MODULE()
BEGIN_MODULE(AudioReader)    AudioPoller_PA *poller; ACCESS(byteA, pcms16ne2c) END_MODULE()
BEGIN_MODULE(AudioWriter)    AudioWriter_libav *writer; ACCESS(byteA, pcms16ne2c) END_MODULE()
#else

struct ImageViewer : Thread {
  struct sImageViewer *s;
  Access_typed<byteA> img;
  ImageViewer(const char* img_name="rgb") : Thread(STRING("ImageViewer_"<<img_name), -1), img(this, img_name, true){}
  ~ImageViewer(){}
  void open();
  void step();
  void close();
};

struct PointCloudViewer : Thread {
  struct sPointCloudViewer *s;
  Access_typed<arr> pts;
  Access_typed<arr> cols;
  PointCloudViewer(const char* pts_name="kinect_points", const char* cols_name="kinect_pointColors")
    : Thread(STRING("PointCloudViewer_"<<pts_name <<'_' <<cols_name), .1),
      pts(this, pts_name),
      cols(this, cols_name){}
  void open();
  void step();
  void close();
};

struct OpencvCamera : Thread {
  struct sOpencvCamera *s;
  Access_typed<byteA> rgb;
  std::map<int,double> properties; bool set(int prop, double value);
  OpencvCamera(const char* rgb_name="rgb") : Thread(STRING("OpencvCamera_"<<rgb_name), 0.), rgb(this, rgb_name){}
  void open();
  void step();
  void close();
};

struct CvtGray : Thread {
  struct sCvtGray *s;
  Access_typed<byteA> rgb;
  Access_typed<byteA> gray;
  std::map<int,double> properties; bool set(int prop, double value);
  CvtGray(const char* rgb_name="rgb", const char* gray_name="gray")
    : Thread(STRING("CvtGray_"<<rgb_name), -1), rgb(this, rgb_name, true), gray(this, gray_name){}
  void open();
  void step();
  void close();
};

struct MotionFilter : Thread {
  struct sMotionFilter *s;
  Access_typed<byteA> rgb;
  Access_typed<byteA> motion;
  MotionFilter(const char* rgb_name="rgb", const char* motion_name="motion")
    : Thread(STRING("MotionFilter_"<<rgb_name), -1), rgb(this, rgb_name, true), motion(this, motion_name){}
  void open();
  void step();
  void close();
};

struct DifferenceFilter : Thread {
  struct sDifferenceFilter *s;
  Access_typed<byteA> i1;
  Access_typed<byteA> i2;
  Access_typed<byteA> diffImage;
  DifferenceFilter(const char* i1_name="i1", const char* i2_name="i2", const char* diffImage_name="diffImage")
    : Thread(STRING("DifferenceFilter_"<<i1_name), -1), i1(this, i1_name, true), i2(this, i2_name), diffImage(this, diffImage_name){}
  void open();
  void step();
  void close();
};

struct CannyFilter : Thread {
  struct sCannyFilter *s;
  Access_typed<byteA> grayImage;
  Access_typed<byteA> cannyImage;
  CannyFilter(const char* grayImage_name="grayImage", const char* cannyImage_name="cannyImage")
    : Thread(STRING("CannyFilter_"<<grayImage_name<<"_" <<cannyImage_name), -1),
      grayImage(this, grayImage_name, true),
      cannyImage(this, cannyImage_name){}
  void open();
  void step();
  void close();
};

struct Patcher : Thread {
  struct sPatcher *s;
  Access_typed<byteA> rgbImage;
  Access_typed<Patching> patchImage;
  Patcher(const char* rgbImage_name="rgbImage", const char* patchImage_name="patchImage")
    : Thread(STRING("Patcher"<<rgbImage_name<<"_" <<patchImage_name), -1),
      rgbImage(this, rgbImage_name, true),
      patchImage(this, patchImage_name){}
  void open();
  void step();
  void close();
};

struct AllViewer : Thread {
  Access_typed<arr> kinect_points;
  Access_typed<arr> kinect_pointColors;
  Access_typed<PlaneA> planes_now;

  mlr::Mesh kinect;
  PlaneA planes_now_copy;
  OpenGL gl;

  AllViewer()
    : Thread("AllViewer", .1),
      kinect_points(this, "kinect_points"),
      kinect_pointColors(this, "kinect_pointColors"),
      planes_now(this, "planes_now"),
      gl("AllViewer"){}
  ~AllViewer(){}
  void open();
  void step();
  void close() {}

};

//BEGIN_MODULE(ImageViewer)      ACCESS(byteA, img)       END_MODULE()
//BEGIN_MODULE(PointCloudViewer) ACCESSlisten(arr, kinect_points)         ACCESS(arr, kinect_pointColors)        END_MODULE()
//BEGIN_MODULE(OpencvCamera)     ACCESS(byteA, rgb)       std::map<int,double> properties; bool set(int prop, double value);  END_MODULE()
//BEGIN_MODULE(CvtGray)          ACCESS(byteA, rgb)       ACCESS(byteA, gray)      END_MODULE()
BEGIN_MODULE(CvtHsv)           ACCESSlisten(byteA, rgb)       ACCESS(byteA, hsv)       END_MODULE()
BEGIN_MODULE(HsvFilter)        ACCESSlisten(byteA, hsv)       ACCESS(floatA, evi)      END_MODULE()
//BEGIN_MODULE(MotionFilter)     ACCESS(byteA, rgb)       ACCESS(byteA, motion)    END_MODULE()
//BEGIN_MODULE(DifferenceFilter) ACCESS(byteA, i1)        ACCESS(byteA, i2)        ACCESS(byteA, diffImage) END_MODULE()
//BEGIN_MODULE(CannyFilter)      ACCESS(byteA, grayImage) ACCESS(byteA, cannyImage)       END_MODULE()
//BEGIN_MODULE(Patcher)          ACCESSlisten(byteA, rgbImage)  ACCESS(Patching, patchImage)    END_MODULE()
BEGIN_MODULE(SURFer)           ACCESSlisten(byteA, grayImage) ACCESS(SURFfeatures, features)  END_MODULE()
BEGIN_MODULE(HoughLineFilter)  ACCESSlisten(byteA, grayImage) ACCESS(HoughLines, houghLines)  END_MODULE()
BEGIN_MODULE(ShapeFitter)      ACCESSlisten(floatA, eviL)     ACCESS(floatA, eviR)            ACCESS(PerceptionOutput, perc)      END_MODULE()
BEGIN_MODULE(AudioReader)    AudioPoller_PA *poller; ACCESS(byteA, pcms16ne2c) END_MODULE()
BEGIN_MODULE(AudioWriter)    AudioWriter_libav *writer; ACCESS(byteA, pcms16ne2c) END_MODULE()

#endif



//===========================================================================
//
// PRELIMINARY
//

//BEGIN_MODULE(ColorPicker)
//  ACCESS(ColorChoice, colorChoice);
//END_MODULE()


//TODO Johannes ProcessL newPointcloudProcesses();
//TODO Johannes VariableL newPointcloudVariables();

//TODO: where should this go? maybe ors?



#endif //MLR_perception_h





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
