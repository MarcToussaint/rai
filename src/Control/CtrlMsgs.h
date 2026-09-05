/*  ------------------------------------------------------------------
    Copyright (c) 2011-2024 Marc Toussaint
    email: toussaint@tu-berlin.de

    This code is distributed under the MIT License.
    Please see <root-path>/LICENSE for details.
    --------------------------------------------------------------  */

#pragma once

#include "../Core/array.h"
#include "../Geo/geo.h"
#include "../Algo/SplineCtrlFeed.h"

namespace rai {

enum class ControlType { configRefs, projectedAcc };

//The control message send to the robot
struct CtrlCmdMsg {
  ControlType controlType=ControlType::configRefs;
  std::shared_ptr<ReferenceFeed> ref; // joint space references
//  arr q_ref, qDot_ref, qDDot_ref; // joint space references
  arr u_b; // open-loop/feed-forward torque term
  arr Kp, Kd; // gain matrices
  arr P_compliance;
  void setConst(const arr& q, bool floating, bool damping){
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
};

// The state message comming back from the robot
struct CtrlStateMsg {
  double ctrlTime=0.;
  int stall=0; //now many iterations should we stall (not increment ctrl time)
  arr q, qDot; // actual joint state
  arr tauExternalIntegral; // external torques
  int tauExternalCount=0;
  void init(const arr& q0) { q=q0; qDot.resize(q.N).setZero(); tauExternalIntegral.resize(q.N).setZero(); tauExternalCount=0; }
};

struct RobotAbstraction {
  Var<rai::CtrlCmdMsg>& cmd;
  Var<rai::CtrlStateMsg>& state;
  int writeData=0;
  RobotAbstraction(Var<rai::CtrlCmdMsg>& cmd, Var<rai::CtrlStateMsg>& state) : cmd(cmd), state(state) {}
  virtual ~RobotAbstraction() {}
};

struct GripperAbstraction {
  virtual void open(double width=1.,
                    double speed=1.) = 0;
  virtual void close(double force=.0, //relative to [min,max]
                     double width=.2, //relative to [min,max]
                     double speed=.2) = 0; //relative to [min,max]
  virtual void closeGrasp(const char* objName, double force=.0, double width=.2, double speed=.2) { close(force, width, speed); }
  virtual double pos() = 0;
  virtual bool isDone() = 0;
};

struct CameraAbstraction {
  rai::String camera_name;
  Var<byteA> image;
  Var<floatA> depth;
  virtual arr getFxycxy() { return arr{}; } //intrinsics
  virtual rai::Transformation getPose() { LOG(-2) <<"not implemented for this camera!"; return 0; } //extrinsics
  void getPointCloud(byteA& image, arr& pts, bool globalCoordinates);
};

} //namespace
