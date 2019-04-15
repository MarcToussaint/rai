#include "robotInterface.h"
#ifdef RAI_ROS
#  include <RosCom/roscom.h>
#  include <RosCom/baxter.h>
#endif
#include "splineRunner.h"

//#include "SimulationThread_self.h"
extern bool Geo_mesh_drawColors;

struct sRobotInterface : Thread, GLDrawer{
  BaxterInterface baxter;
  arr q0, q_ref;
  StringA jointNames;
  rai::KinematicWorld K_ref, K_baxter;
  OpenGL gl;
  bool useBaxter=false;

  SplineRunner spline;
  double dt; // time stepping interval

  sRobotInterface(const rai::KinematicWorld& _K, double _dt)
    : Thread("RobotInterface", _dt),
      K_ref(_K),
      dt(_dt){

    useBaxter = rai::getParameter<bool>("useRos",false);

    gl.add(*this);
    gl.camera.setDefault();

    q0 = _K.getJointState();
    jointNames = _K.getJointNames();
    if(useBaxter) K_baxter = K_ref;
    threadLoop();
  }

  ~sRobotInterface(){
    threadClose();
  }

  void step(){
    {
      auto lock = stepMutex(RAI_HERE);

      //read out the new reference
      arr q_spline = spline.run(dt);

      if(q_spline.N) q_ref = q_spline;
      //otherwise q_ref is just the last spline point..

      if(q_ref.N){
        if(useBaxter) baxter.send_q(q_ref);
        {
          auto lock = gl.dataLock(RAI_HERE);
          K_ref.setJointState(q_ref);
        }
      }
    }


    if(!(step_count%10)){
      {
      }
      gl.update(STRING("step=" <<step_count <<" phase=" <<spline.phase <<" timeToGo=" <<spline.timeToGo() <<" #ref=" <<spline.refSpline.points.d0));
    }

  }

  void glDraw(OpenGL& gl){
    //        auto lock = stepMutex(RAI_HERE);
    glStandardScene(NULL);
    K_ref.glDraw(gl);
//    auto lock = gl.dataLock(RAI_HERE);
    K_baxter.setJointState(baxter.get_q());
    Geo_mesh_drawColors=false;
    glColor(.8, .2, .2, .5);
    gl.drawMode_idColor = true;
    if(useBaxter) K_baxter.glDraw(gl);
    Geo_mesh_drawColors=true;
    gl.drawMode_idColor = false;
  }


};


RobotInterface::RobotInterface(const rai::KinematicWorld& _K, double dt) {
  s = make_shared<sRobotInterface>(_K, dt);
}

RobotInterface::~RobotInterface(){
}


/** This is the core method to send motion to the robot. It
    constructs a spline from the given path and times (could
    be just a single next pose and a time), and then iterates
    a "spline runner", which evaluates the spline in dt intervals
    and sends the poses to the robot (position control).

    This command is non-blocking! You can continue computing
    novel things while the robot still moves.

    You can append, meaning that the spline is extended with the
    novel poses.

    To interrupt a running motion, send and empty path with
    append=false. */
void RobotInterface::move(const arr& path, const arr& times, bool append){
  auto lock = s->stepMutex(RAI_HERE);
  arr _path = path.ref();
  if(_path.nd==1) _path.reshape(1,_path.N);
  s->spline.set(_path, times, getJointPositions(), append);
}

void RobotInterface::move(const arrA& poses, const arr& times, bool append){
  arr path(poses.N, poses(0).N);
  for(uint i=0;i<path.d0;i++) path[i] = poses(i);
  move(path, times, append);
}

double RobotInterface::timeToGo(){
  auto lock = s->stepMutex(RAI_HERE);
  return s->spline.timeToGo();
}

//void RobotInterface::execGripper(const rai::String& gripper, double position, double force){
//  auto lock = stepMutex(RAI_HERE);
//  if(gripper=="pr2R"){
//    //  komo->addObjective(0., 0., OT_eq, FS_accumulatedCollisions, {}, 1e0);
//    //open gripper
//    //  komo->addObjective(0.,0., OT_sos, FS_qItself, {"r_gripper_joint"}, 1e1, {.08} );
//    //  komo->addObjective(0.,0., OT_sos, FS_qItself, {"r_gripper_l_finger_joint"}, 1e1, {.8} );

//    SIM.setUsedRobotJoints({"r_gripper_joint", "r_gripper_l_finger_joint"});
//    SIM.exec({1,2, {position, position*10.}}, {1.}, true);
//    return;
//  }
//  if(gripper=="pandaL"){
//    SIM.setUsedRobotJoints({"L_panda_finger_joint1"});
//    SIM.exec(arr(1,1, {position}), {1.}, true);
//    return;
//  }
//  NIY
//}

arr RobotInterface::getHomePose(){ return s->q0; }

const StringA& RobotInterface::getJointNames(){
  return s->jointNames;
}

arr RobotInterface::getJointPositions(const StringA& joints){
  auto lock = s->stepMutex(RAI_HERE);
  if(s->useBaxter) return s->baxter.get_q();
  return s->K_ref.getJointState();
}

void RobotInterface::sync(rai::KinematicWorld& K){
  auto lock = s->stepMutex(RAI_HERE);
  K.setJointState(getJointPositions(), s->jointNames);
}

