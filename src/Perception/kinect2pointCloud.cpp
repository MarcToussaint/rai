#include "kinect2pointCloud.h"

Kinect2PointCloud::Kinect2PointCloud() : Thread("Kinect2PointCloud"){
  threadOpen();
}

Kinect2PointCloud::~Kinect2PointCloud(){
  threadClose();
}

void Kinect2PointCloud::step(){
  depth = kinect_depth.get();
  rgb = kinect_rgb.get();

  images2pointcloud(pts, cols, rgb, depth);

  kinect_frame.readAccess();
  if(!kinect_frame().isZero()) kinect_frame().applyOnPointArray(pts);
  kinect_frame.deAccess();

  kinect_points.set() = pts;
  kinect_pointColors.set() = cols;
}


void images2pointcloud(arr& pts, arr& cols, const byteA& rgb, const uint16A& depth){
  CHECK_EQ(rgb.d2, 3, "");
  uint H=rgb.d0, W=rgb.d1;
  CHECK_EQ(H, 480, "");
  CHECK_EQ(W, 640, "");

  CHECK_EQ(depth.d0, H,"");
  CHECK_EQ(depth.d1, W,"");

  depthData2pointCloud(pts, depth);

  cols.resize(W, H, 3);
  for(uint i=0;i<rgb.N;i++) cols.elem(i) = (double)rgb.elem(i)/255.;
}

void depthData2pointCloud(arr& pts, const uint16A& depth){
  uint H=depth.d0, W=depth.d1;
  CHECK_EQ(H, 480, "");
  CHECK_EQ(W, 640, "");

  pts.resize(H*W, 3);

  float constant = 1.0f / 580; //focal length of kinect in pixels
  int centerX = (W >> 1);
  int centerY = (H >> 1);

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

  pts.reshape(H, W, 3);
}
