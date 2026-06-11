/*  ------------------------------------------------------------------
    Copyright (c) 2011-2024 Marc Toussaint
    email: toussaint@tu-berlin.de

    This code is distributed under the MIT License.
    Please see <root-path>/LICENSE for details.
    --------------------------------------------------------------  */

#include "CtrlMsgs.h"

#include "../Geo/depth2PointCloud.h"

void rai::CameraAbstraction::getPointCloud(byteA& img, arr& pts, bool globalCoordinates) {
  floatA dep = depth.get();
  img = image.get();
  depthData2pointCloud(pts, dep, getFxycxy());
  if(globalCoordinates) {
    rai::Transformation pose=getPose();
    if(!pose.isZero()) {
      pose.applyOnPointArray(pts);
    }
  }
}
