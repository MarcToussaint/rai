#include "kinect2pointCloud.h"

REGISTER_MODULE(Kinect2PointCloud)

const unsigned int image_width = 640; //kinect resolution
const unsigned int image_height = 480; //kinect resolution
const unsigned int depth_size = image_width*image_height;

ors::Camera kinectCam;

void initKinectCam(){
  kinectCam.setPosition(0., 0., 0.);
  kinectCam.focus(0., 0., 1.);
  kinectCam.setZRange(.1, 10.);
  kinectCam.heightAbs=kinectCam.heightAngle=0.;
  kinectCam.focalLength = 580./480.;
}


void Kinect2PointCloud::step(){
  depth = kinect_depth.get();
  rgb = kinect_rgb.get();

  MLR::images2pointcloud(pts, cols, rgb, depth);

  kinect_frame.readAccess();
  if(!kinect_frame().isZero()){
    kinect_frame().applyOnPointArray(pts);
  }
  kinect_frame.deAccess();

  kinect_points.set() = pts;
  kinect_pointColors.set() = cols;
}


void MLR::images2pointcloud(arr& pts, arr& cols, const byteA& rgb, const uint16A& depth){
  depthData2pointCloud(pts, depth);

  if(rgb.N!=3*image_width*image_height){
    MLR_MSG("kinect rgb data has wrong dimension: rgb.dim=" <<rgb.dim());
    return;
  }

  cols.resize(image_width*image_height, 3);
  for(uint i=0;i<rgb.N;i++) cols.elem(i) = (double)rgb.elem(i)/255.;
}

void MLR::depthData2pointCloud(arr& pts, const uint16A& depth){
  if(depth.N!=image_width*image_height){
    MLR_MSG("kinect depth data has wrong dimension: depth.dim=" <<depth.dim());
    return;
  }

  pts.resize(image_width*image_height, 3);

  float constant = 1.0f / 580; //focal length of kinect in pixels
  int centerX = (image_width >> 1);
  int centerY = (image_height >> 1);

  int i = 0;
  for(int y=-centerY+1; y<=centerY; y++) for(int x=-centerX+1; x<=centerX; x++, i++) {
    double d=depth.elem(i);
    if (d!= 0 && d!=2047) { //2^11-1
      double z=(double) d * 0.001;
      pts(i, 0) = z*constant*x;
      pts(i, 1) = z*constant*y;
      pts(i, 2) = z;
    }else{
      pts(i, 0) = 0.;
      pts(i, 1) = 0.;
      pts(i, 2) = -1.;
    }
  }
}
