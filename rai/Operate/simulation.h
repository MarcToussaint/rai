#pragma once

#include <Gui/opengl.h>
#include <Geo/mesh.h>
#include <Kin/kin.h>
#include <Kin/frame.h>
#include <iomanip>

//=============================================================================

struct ContactEntry{
  rai::String objA, objB;
  double d;
  void write(ostream& os) const{ os <<"Contact " <<objA <<"--" <<objB <<" distance=" <<d; }
};

typedef rai::Array<ContactEntry> ContactA;

struct PlanSkeletonEntry{
  StringA symbols;
  double timeStart=-1.;
  double timeTo=-1.;
  PlanSkeletonEntry(){}
  PlanSkeletonEntry(StringA symbols,double _timeFrom,double _timeTo):symbols(symbols), timeStart(_timeFrom), timeTo(_timeTo){}
  void write(ostream& os) const{ symbols.write(os," ",NULL,"()"); os <<" from " <<timeStart <<" to " <<timeTo; }
};
stdOutPipe(PlanSkeletonEntry)

struct Plan{
  arr robot_joint_path;
  arr tau;
  rai::Array<PlanSkeletonEntry> skeleton;
};

//=============================================================================

struct Simulation : GLDrawer{
  struct Simulation_self *self=0;
  rai::KinematicWorld K;

  Simulation(const rai::KinematicWorld& _K, double dt=.01);
  ~Simulation();

  //-- stepping physics
  void stepPhysics(uint k=1);
  void stepKin();

  //--
  void setJointState(const StringA& joints, const arr& q_ref);
  void setJointStateSafe(arr q_ref, StringA& jointsInLimit, StringA& collisionPairs);

  //-- store and restore simulator state
  rai::KinematicWorld getState();
  void restoreState(rai::KinematicWorld& K);

  //-- the narrow action interface
  StringA getRobotJoints(); //info on the joints; the plan needs to have same dimensionality
  void setUsedRobotJoints(const StringA& joints);
  void exec(const Plan& plan, bool append=true);
  void exec(const arr& robot_joint_path, const arr& tau, bool append=true);
  void exec(const StringA& command);
  void stop(bool hard=false);

  //-- feedback from action execution
  double getTimeToGo();
  double trackingError();

  //-- sensor readouts
  void setCamera(const char* sensorName);
  void setVisualObjects(const StringA& visualObjects, bool allButGiven=false);
  void getCamera(byteA& image=NoByteA,
                 arr& realDepth=NoArr, // z coordinate in meters, for each pixel
                 arr& pointCloud=NoArr, // x,y,z coordinates in meters, for each pixel
                 uint16A& kinectDepth=NoUint16A); // simulated kinect signal; kinectRgb is same as image
  void getSegmentation(byteA& segmentation); //obj ID, for each pixel
  arr  getFTsensor(const char* sensorFrameName);

  //-- true state info
  arr getJointState();
  arr getFrameState();
  arr getObjectPoses(const StringA& objects={});
  StringA getJointNames();
  StringA getObjectNames();
  ContactA getContactInfo(double margin=.1);

  void glDraw(OpenGL &);
};

