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

//-- Module declarations
BEGIN_MODULE(ImageViewer)      ACCESS(byteA, img)       END_MODULE()
BEGIN_MODULE(VideoEncoder)      ACCESS(byteA, img)       END_MODULE()
BEGIN_MODULE(PointCloudViewer) ACCESS(arr, pts)         ACCESS(arr, cols)        END_MODULE()
BEGIN_MODULE(OpencvCamera)     ACCESS(byteA, rgb)       END_MODULE()
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
niyPipes(PerceptionOutput)





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
//TODO Johannes const int RADIUS = 2;
//TODO Johannes const int HEIGHT = 3;


/*TODO Johannes
struct ObjectBelief {

  ObjectBelief() {
    shapeParams.resize(4);  
  }
  //pose
  // TODO: make pointers
  ors::Vector position;
  ors::Quaternion rotation;

  arr poseCov;

  // primitive shapes
  ors::ShapeType shapeType;
  arr shapeParams;

  // TODO: make pointer, such that the using app does not need to implicitly
  // include half of the PCL?
  //pcl::ModelCoefficients::Ptr pcl_object;

  //pcl::PointCloud<PointT>* pointCloud;
  arr vertices;
  uintA triangles;
};

struct ObjectBeliefSet : Variable {
  FIELD(MT::Array<ObjectBelief*>, objects);
  ObjectBeliefSet(const char *name) : Variable(name) { reg_objects(); }
};
*/

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
