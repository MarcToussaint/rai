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

void rai::CtrlCmdMsg::setConst(const arr& q, bool floating, bool damping){
  auto zref = std::dynamic_pointer_cast<rai::ConstCtrlReference>(ref);
  if(!zref){
    ref = make_shared<rai::ConstCtrlReference>();
    zref = std::dynamic_pointer_cast<rai::ConstCtrlReference>(ref);
    CHECK(zref, "this is not a spline reference!")
  }
  if(floating){
    zref->setPositionReference({});
    if(damping) zref->setVelocityReference({0.}); //{0.}: have a Kd with zero vel ref;
    else zref->setVelocityReference({}); //{}: have no Kd term at all; {1.} have a Kd term with velRef=velTrue (and friction compensation!)
  }else{
    zref->setPositionReference(q);
    zref->setVelocityReference({0.});
  }
}
