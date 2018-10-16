#pragma once

#include <Kin/kin.h>
#include <Gui/opengl.h>

namespace rai {

struct CameraView : GLDrawer {

  /*! describes a sensor from which we can take 'images' within the simulation (e.g.: kinect, suctionRingView, etc) */
  struct Sensor {
    rai::String name;
    rai::Camera cam;     ///< this includes the transformation X
    uint width=640, height=480;
    byteA backgroundImage;
    rai::Frame *frame=0;
    Sensor(){}
    rai::Transformation& pose(){ return cam.X; }
  };

  //-- description of world configuration
  rai::KinematicWorld K;       //the configuration
  rai::Array<Sensor> sensors;  //the list of sensors

  OpenGL gl;

  //-- run parameter
  bool background=true;

  //-- outputs of image analysis/computation
  byteA image;
  byteA segmentation;
  floatA depth;
  uint16A kinect_depth; // kinect_rgb is same as image
  arr kinect_pcl;
  rai::Mesh surfaceMesh;

  //-- evaluation outputs
  CameraView(const rai::KinematicWorld& _K, bool _background=true);
  ~CameraView(){}

  //-- loading the configuration: the meshes, the robot model, the tote, the sensors; all ends up in K
  Sensor& addSensor(const char* name, const char* frameAttached, uint width, uint height, double focalLength=-1., double orthoAbsHeight=-1., const arr& zRange={}, const char* backgroundImageFile=0){
    Sensor& sen = sensors.append();
    sen.name = name;
    sen.frame = K.getFrameByName(frameAttached);
    rai::Camera& cam = sen.cam;
    sen.width=width;
    sen.height=height;

    cam.setZero();
    if(zRange.N) cam.setZRange(zRange(0), zRange(1));
    if(focalLength>0.) cam.setFocalLength(focalLength);
    if(orthoAbsHeight>0.) cam.setHeightAbs(orthoAbsHeight);

    cam.setWHRatio((double)width/height);
    return sen;
  }

  Sensor& selectSensor(const char* sensorName); //set the OpenGL sensor

  //-- compute/analyze a camera perspective (stored in classes' output fields)
  void computeImageAndDepth(byteA& image, floatA& depth);
  void computeKinectDepth(uint16A& kinect_depth);
  void computePointCloud(byteA& pcl); // point cloud (rgb of every point is given in image)
  void computeSegmentation(byteA& segmentation);     // -> segmentation

  void glDraw(OpenGL &gl);
};

}
