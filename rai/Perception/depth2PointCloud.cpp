/*  ------------------------------------------------------------------
    Copyright (c) 2011-2020 Marc Toussaint
    email: toussaint@tu-berlin.de

    This code is distributed under the MIT License.
    Please see <root-path>/LICENSE for details.
    --------------------------------------------------------------  */

#include "depth2PointCloud.h"

Depth2PointCloud::Depth2PointCloud(Var<floatA>& _depth, float _fx, float _fy, float _px, float _py)
  : Thread("Depth2PointCloud"),
    depth(this, _depth, true),
    fx(_fx), fy(_fy), px(_px), py(_py) {
  pose.set()->setZero();
  threadOpen();
}

Depth2PointCloud::Depth2PointCloud(Var<floatA>& _depth, const arr& Fxypxy)
  : Depth2PointCloud(_depth, Fxypxy(0), Fxypxy(1), Fxypxy(2), Fxypxy(3)) {
}

Depth2PointCloud::~Depth2PointCloud() {
  threadClose();
}

void Depth2PointCloud::step() {
  _depth = depth.get();

  depthData2pointCloud(_points, _depth, fx, fy, px, py);

  rai::Transformation _pose = pose.get(); //this is relative to "/base_link"
  if(!_pose.isZero()) _pose.applyOnPointArray(_points);

  points.set() = _points;
}

void depthData2pointCloud(arr& pts, const floatA& depth, float fx, float fy, float px, float py) {
  uint H=depth.d0, W=depth.d1;

  CHECK(fx>0, "need a focal length greater zero!(not implemented for ortho yet)");
  if(std::isnan(fy)) fy = fx;
  if(std::isnan(px)) px=.5*W;
  if(std::isnan(py)) py=.5*H;

  pts.resize(H*W, 3);
  pts.setZero();
  double* pt=pts.p;
  float* de=depth.p;

  for(uint i=0; i<H; i++) for(uint j=0; j<W; j++) {
      float d = *(de++);
      if(d>=0.) {
        float x=j, y=i;
#if 0 //slow
        pts(k, 0) =  d * (x - px) / fx;
        pts(k, 1) = -d * (y - py) / fy;
        pts(k, 2) = -d;
#else //fast
        *(pt++) =  d * (x - px) / fx;
        *(pt++) = -d * (y - py) / fy;
        *(pt++) = -d;
#endif
      } else {
        pt += 3;
      }
    }

  pts.reshape(H, W, 3);

}

void depthData2pointCloud(arr& pts, const floatA& depth, const arr& Fxypxy) {
  depthData2pointCloud(pts, depth, Fxypxy.elem(0), Fxypxy.elem(1), Fxypxy.elem(2), Fxypxy.elem(3));
}

void depthData2point(double* pt, double* fxypxy) {
  pt[0] =  pt[2] * (pt[0] - fxypxy[2]) / fxypxy[0];
  pt[1] = -pt[2] * (pt[1] - fxypxy[3]) / fxypxy[1];
  pt[2] = -pt[2];
}

void depthData2point(arr& pt, const arr& Fxypxy) {
  CHECK_EQ(pt.N, 3, "need a 3D point");
  CHECK_EQ(Fxypxy.N, 4, "need 4 intrinsic parameters");
  depthData2point(pt.p, Fxypxy.p);
}

