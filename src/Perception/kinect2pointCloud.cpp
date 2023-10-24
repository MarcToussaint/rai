/*  ------------------------------------------------------------------
    Copyright (c) 2011-2020 Marc Toussaint
    email: toussaint@tu-berlin.de

    This code is distributed under the MIT License.
    Please see <root-path>/LICENSE for details.
    --------------------------------------------------------------  */

#include "kinect2pointCloud.h"
#include "../Kin/kin.h"

Kinect2PointCloud::Kinect2PointCloud()
  : Thread("Kinect2PointCloud"), kinect_depth(this, true) {
  depthShift_dx = rai::getParameter<int>("kinectDepthPixelShift_x", 0);
  depthShift_dy = rai::getParameter<int>("kinectDepthPixelShift_y", 0);
  frameShift = rai::getParameter<arr>("kinectFrameShift", {});
  threadOpen();
}

Kinect2PointCloud::~Kinect2PointCloud() {
  threadClose();
}

void Kinect2PointCloud::step() {
  depth = kinect_depth.get();
  rgb = kinect_rgb.get();

  depthData2pointCloud(pts, depth, depthShift_dx, depthShift_dy);
//  cout <<depthShift_dx <<' ' <<depthShift_dy <<endl;

  frame = kinect_frame.get(); //this is relative to "/base_link"
  arr basePose = pr2_odom.get();

  if(basePose.N) {
    rai::Transformation base;
    base.pos.set(basePose(0), basePose(1), 0);
    base.rot.setRad(basePose(2), 0, 0, 1);
    frame = base*frame;
  }

  //verbose to compare ros kinect frame with modelWorld..
//  cout <<"KINECT frame=" <<frame <<" -- base pose=" <<basePose <<endl;
//  Var<rai::Configuration> K(nullptr, "modelWorld");
//  rai::Transformation k = K.get()->getShapeByName("endeffKinect")->X;
//  cout <<"ors: frame=" <<k <<" real/k" <<frame/k <<" k/real" <<k/frame <<endl;

  if(frameShift.N) frame.addRelativeTranslation(frameShift(0), frameShift(1), frameShift(2));
  if(!frame.isZero()) frame.applyOnPointArray(pts);

  kinect_points.set() = pts;
}

void depthData2pointCloud(arr& pts, const uint16A& depth, int depthShift_dx, int depthShift_dy) {
  uint H=depth.d0, W=depth.d1;
  CHECK_EQ(H, 480, "");
  CHECK_EQ(W, 640, "");

  pts.resize(H*W, 3);

  //  float constant = 1.0f / 580; //focal length of kinect in pixels
  double focal_x = rai::getParameter<int>("focal_x", 530); // focal length x direction in pixels
  double focal_y = rai::getParameter<int>("focal_y", 510); // focal length y direction in pixels
  int centerX = (W >> 1);
  int centerY = (H >> 1);

  uint i=0;
  for(int y=-centerY+1; y<=centerY; y++) for(int x=-centerX+1; x<=centerX; x++, i++) {
      int j = i+depthShift_dx+depthShift_dy*depth.d1;
      if(j<0) j=0;
      if(j>=(int)depth.N) j=depth.N-1;
      uint16_t d = depth.elem(j);
      if(d!= 0 && d!=2047) {  //2^11-1
        double z=(double) d * 0.001;
        pts(i, 0) = z*(1.0/focal_x)*x;
        pts(i, 1) = z*(1.0/focal_y)*y;
        pts(i, 2) = z;
      } else {
        pts(i, 0) = 0.;
        pts(i, 1) = 0.;
        pts(i, 2) = -1.;
      }
    }

  pts.reshape(H, W, 3);
}
