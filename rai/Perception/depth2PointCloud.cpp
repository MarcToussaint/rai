/*  ------------------------------------------------------------------
    Copyright (c) 2017 Marc Toussaint
    email: marc.toussaint@informatik.uni-stuttgart.de

    This code is distributed under the MIT License.
    Please see <root-path>/LICENSE for details.
    --------------------------------------------------------------  */

#include "depth2PointCloud.h"

Depth2PointCloud::Depth2PointCloud(Var<arr>& _depth, double _forcalLength)
  : Thread("Depth2PointCloud"),
    depth(this, _depth, true),
    focalLength(_forcalLength){
  pose.set()->setZero();
  threadOpen();
}

Depth2PointCloud::~Depth2PointCloud() {
  threadClose();
}

void Depth2PointCloud::step() {
  _depth = depth.get();
  
  depthData2pointCloud(_pts, _depth, focalLength);

  rai::Transformation _pose = pose.get(); //this is relative to "/base_link"
  if(!_pose.isZero()) _pose.applyOnPointArray(_pts);
  
  pts.set() = _pts;
}

void depthData2pointCloud(arr& pts, const arr& depth, double focalLength){
  uint H=depth.d0, W=depth.d1;

  pts.resize(H*W, 3);

  CHECK(focalLength>0, "need a focal length greater zero!(not implemented for ortho yet)");
  int centerX = (W >> 1);
  int centerY = (H >> 1);
  double focal_x = 1./(focalLength*H);
  double focal_y = 1./(focalLength*H);

  uint i=0;
  for(int y=-centerY+1; y<=centerY; y++) for(int x=-centerX+1; x<=centerX; x++, i++) {
    double d = depth.elem(i);
    if(d>=0) {  //2^11-1
      pts(i, 0) = d*focal_x*x;
      pts(i, 1) = -d*focal_y*y;
      pts(i, 2) = -d;
    } else {
      pts(i, 0) = 0.;
      pts(i, 1) = 0.;
      pts(i, 2) = 1.;
    }
  }

  pts.reshape(H, W, 3);

}
