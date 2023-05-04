#include "CtrlMsgs.h"

#include "../Geo/depth2PointCloud.h"

void rai::CameraAbstraction::getPointCloud(byteA& image, arr& pts, bool globalCoordinates) {
  floatA depth;
  getImageAndDepth(image, depth);
  depthData2pointCloud(pts, depth, getFxypxy());
  if(globalCoordinates){
    rai::Transformation pose=getPose();
    if(!pose.isZero()){
      pose.applyOnPointArray(pts);
    }
  }
}
