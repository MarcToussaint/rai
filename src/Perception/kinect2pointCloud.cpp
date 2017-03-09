#include "kinect2pointCloud.h"

Kinect2PointCloud::Kinect2PointCloud()
  : Thread("Kinect2PointCloud"){
  depthShift_dx = mlr::getParameter<int>("kinectDepthPixelShift_x", 0);
  depthShift_dy = mlr::getParameter<int>("kinectDepthPixelShift_y", 0);
  frameShift = mlr::getParameter<arr>("kinectFrameShift", {});
  threadOpen();
}

Kinect2PointCloud::~Kinect2PointCloud(){
  threadClose();
}

void Kinect2PointCloud::step(){
  depth = kinect_depth.get();
  rgb = kinect_rgb.get();

  depthData2pointCloud(pts, depth, depthShift_dx, depthShift_dy);

  frame = kinect_frame.get();
  if(frameShift.N) frame.pos += frameShift;
  if(!frame.isZero()) frame.applyOnPointArray(pts);

  kinect_points.set() = pts;
}

void depthData2pointCloud(arr& pts, const uint16A& depth, int depthShift_dx, int depthShift_dy){
  uint H=depth.d0, W=depth.d1;
  CHECK_EQ(H, 480, "");
  CHECK_EQ(W, 640, "");

  pts.resize(H*W, 3);

  float constant = 1.0f / 580; //focal length of kinect in pixels
  int centerX = (W >> 1);
  int centerY = (H >> 1);

  uint i=0;
  for(int y=-centerY+1; y<=centerY; y++) for(int x=-centerX+1; x<=centerX; x++, i++) {
    int j = i+depthShift_dx+depthShift_dy*depth.d1;
    if(j<0) j=0;
    if(j>=(int)depth.N) j=depth.N-1;
    uint16_t d = depth.elem(j);
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
