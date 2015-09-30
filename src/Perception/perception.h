#ifndef MT_perception_h
#define MT_perception_h

#ifdef MT_OPENCV
#  undef COUNT
#  include <opencv2/opencv.hpp>
#  undef MIN
#  undef MAX
#endif

#include <Core/module.h>
#include <Ors/ors.h>
#include <Core/array_t.h>
#include <Gui/opengl.h>
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
struct GenericDisplayViewer : Module {
  OpenGL *gl;
  ACCESS(T, var)
  GenericDisplayViewer(): Module("GenericDisplayViewer"), gl(NULL) {} \
  virtual void open(){ gl = new OpenGL(STRING("ImageViewer '"<<var.var->name()<<'\'')); }
  virtual void step(){
    gl->background = var.get()->display;
    if(gl->height!= gl->background.d0 || gl->width!= gl->background.d1)
      gl->resize(gl->background.d1, gl->background.d0);
    gl->update();
  }
  virtual void close(){ delete gl; }
};

struct VideoEncoder : public Module {
  struct sVideoEncoder *s;
  bool is_rgb;
  double fps;
  ACCESS(byteA, img);
  VideoEncoder():is_rgb(false), fps(30) {}
  virtual ~VideoEncoder() {}

  virtual void open();
  virtual void step();
  virtual void close();
  /// set input packing (default is bgr)
  void set_rgb(bool is_rgb) { this->is_rgb = is_rgb; }
  /// set frames per second -- only effective before open
  void set_fps(double fps) { this->fps = fps; }
};

struct VideoEncoderX264 : public Module {
   struct sVideoEncoderX264 *s;
   bool is_rgb;
   double fps;
   ACCESS(byteA, img);
   VideoEncoderX264():is_rgb(false) {}
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

typedef MT::Array<RigidObjectRepresentation*> RigidObjectRepresentationL;


//===========================================================================
//
// Variables
//

struct ColorChoice{
  FIELD(byteA, rgb);
  FIELD(byteA, hsv);
};

//===========================================================================

struct HoughLines {
#ifdef MT_OPENCV
  std::vector<cv::Vec4i> lines;
#endif
  FIELD(byteA, display);
};
inline void operator>>(istream& is,HoughLines& hl){}
inline void operator<<(ostream& os,const HoughLines& hl){}

//===========================================================================

struct Patching {
  uintA patching;  //for each pixel an integer
  arr pch_cen;     //patch centers
  uintA pch_edges; //patch Delauney edges
  floatA pch_rgb;  //patch mean colors
  FIELD(byteA, display);
};
inline void operator>>(istream& is,Patching& hl){}
inline void operator<<(ostream& os,const Patching& hl){}

//===========================================================================

struct SURFfeatures {
#ifdef MT_OPENCV
  std::vector<cv::KeyPoint> keypoints;
  std::vector<float> descriptors;
#endif
  FIELD(byteA, display);
};
inline void operator>>(istream& is,SURFfeatures& hl){}
inline void operator<<(ostream& os,const SURFfeatures& hl){}

//===========================================================================

/** The RigidObjectRepresentation List output of perception */
struct PerceptionOutput {
  MT::Array<RigidObjectRepresentation> objects;
  FIELD(byteA, display);
};
niyPipes(PerceptionOutput);


//-- Module declarations
#if 1
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

struct ImageViewer:Module{
  struct sImageViewer *s;
  Access_typed<byteA> img;
  ImageViewer(const char* img_name="rgb") : Module(STRING("ImageViewer_"<<img_name)), img(this, img_name){}
  void open();
  void step();
  void close();
};

struct OpencvCamera:Module{
  struct sOpencvCamera *s;
  Access_typed<byteA> rgb;
  std::map<int,double> properties; bool set(int prop, double value);
  OpencvCamera(const char* rgb_name="rgb") : Module(STRING("OpencvCamera_"<<rgb_name), NoModuleL, Module::loopFull), rgb(this, rgb_name){}
  void open();
  void step();
  void close();
};

struct CvtGray:Module{
  struct sCvtGray *s;
  Access_typed<byteA> rgb;
  Access_typed<byteA> gray;
  std::map<int,double> properties; bool set(int prop, double value);
  CvtGray(const char* rgb_name="rgb", const char* gray_name="gray")
    : Module(STRING("CvtGray_"<<rgb_name)), rgb(this, rgb_name), gray(this, gray_name){}
  void open();
  void step();
  void close();
};

struct MotionFilter:Module{
  struct sMotionFilter *s;
  Access_typed<byteA> rgb;
  Access_typed<byteA> motion;
  MotionFilter(const char* rgb_name="rgb", const char* motion_name="motion")
    : Module(STRING("MotionFilter_"<<rgb_name)), rgb(this, rgb_name), motion(this, motion_name){}
  void open();
  void step();
  void close();
};

struct DifferenceFilter:Module{
  struct sDifferenceFilter *s;
  Access_typed<byteA> i1;
  Access_typed<byteA> i2;
  Access_typed<byteA> diffImage;
  DifferenceFilter(const char* i1_name="i1", const char* i2_name="i2", const char* diffImage_name="diffImage")
    : Module(STRING("DifferenceFilter_"<<i1_name)), i1(this, i1_name), i2(this, i2_name), diffImage(this, diffImage_name){}
  void open();
  void step();
  void close();
};

struct CannyFilter:Module{
  struct sCannyFilter *s;
  Access_typed<byteA> grayImage;
  Access_typed<byteA> cannyImage;
  CannyFilter(const char* grayImage_name="grayImage", const char* cannyImage_name="cannyImage")
    : Module(STRING("CannyFilter_"<<grayImage_name<<"_" <<cannyImage_name)),
      grayImage(this, grayImage_name),
      cannyImage(this, cannyImage_name){}
  void open();
  void step();
  void close();
};

//BEGIN_MODULE(ImageViewer)      ACCESS(byteA, img)       END_MODULE()
BEGIN_MODULE(PointCloudViewer) ACCESS(arr, pts)         ACCESS(arr, cols)        END_MODULE()
//BEGIN_MODULE(OpencvCamera)     ACCESS(byteA, rgb)       std::map<int,double> properties; bool set(int prop, double value);  END_MODULE()
//BEGIN_MODULE(CvtGray)          ACCESS(byteA, rgb)       ACCESS(byteA, gray)      END_MODULE()
BEGIN_MODULE(CvtHsv)           ACCESS(byteA, rgb)       ACCESS(byteA, hsv)       END_MODULE()
BEGIN_MODULE(HsvFilter)        ACCESS(byteA, hsv)       ACCESS(floatA, evi)      END_MODULE()
//BEGIN_MODULE(MotionFilter)     ACCESS(byteA, rgb)       ACCESS(byteA, motion)    END_MODULE()
//BEGIN_MODULE(DifferenceFilter) ACCESS(byteA, i1)        ACCESS(byteA, i2)        ACCESS(byteA, diffImage) END_MODULE()
//BEGIN_MODULE(CannyFilter)      ACCESS(byteA, grayImage) ACCESS(byteA, cannyImage)       END_MODULE()
BEGIN_MODULE(Patcher)          ACCESS(byteA, rgbImage)  ACCESS(Patching, patchImage)    END_MODULE()
BEGIN_MODULE(SURFer)           ACCESS(byteA, grayImage) ACCESS(SURFfeatures, features)  END_MODULE()
BEGIN_MODULE(HoughLineFilter)  ACCESS(byteA, grayImage) ACCESS(HoughLines, houghLines)  END_MODULE()
BEGIN_MODULE(ShapeFitter)      ACCESS(floatA, eviL)     ACCESS(floatA, eviR)            ACCESS(PerceptionOutput, perc)      END_MODULE()
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



#endif //MT_perception_h





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
