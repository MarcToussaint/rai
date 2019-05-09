#include <RosCom/roscom.h>
#include <RosCom/baxter.h>

#include <Gui/opengl.h>

#include "robotOperation.h"
#include "splineRunner.h"


//#include "SimulationThread_self.h"
extern bool Geo_mesh_drawColors;

struct sRobotOperation : Thread, GLDrawer{
  BaxterInterface baxter;
  arr q0, q_ref;
  StringA jointNames;
  rai::KinematicWorld K_ref, K_baxter;
  OpenGL gl;
  bool useBaxter=false;
  bool sendToBaxter=false;

  SplineRunner spline;
  double dt; // time stepping interval

  sRobotOperation(const rai::KinematicWorld& _K, double _dt, bool useRosDefault)
    : Thread("RobotInterface", _dt),
      baxter(useRosDefault),
      K_ref(_K),
      dt(_dt){

    useBaxter = rai::getParameter<bool>("useRos", useRosDefault);
    sendToBaxter = useBaxter;

    gl.add(*this);
    gl.camera.setDefault();

    q0 = _K.getJointState();
    jointNames = _K.getJointNames();
    if(useBaxter) K_baxter = K_ref;
    threadLoop();
  }

  ~sRobotOperation(){
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
        if(useBaxter && sendToBaxter) baxter.send_q(q_ref);
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

//      cout <<"ONLINE! " <<baxter.get_q() <<endl;
    }

  }

  void glDraw(OpenGL& gl){
    //        auto lock = stepMutex(RAI_HERE);
    glStandardScene(NULL);
    K_ref.glDraw(gl);
//    auto lock = gl.dataLock(RAI_HERE);
    if(useBaxter){
      arr q_real = baxter.get_q();
      if(q_real.N == K_baxter.getJointStateDimension()){
        K_baxter.setJointState(q_real);
      }
      Geo_mesh_drawColors=false;
      glColor(.8, .2, .2, .5);
      gl.drawMode_idColor = true;
      K_baxter.glDraw(gl);
      Geo_mesh_drawColors=true;
      gl.drawMode_idColor = false;
    }
  }


};


RobotOperation::RobotOperation(const rai::KinematicWorld& _K, double dt, const char* rosNodeName) {
  if(rosNodeName && strlen(rosNodeName)>3){
      rosCheckInit(rosNodeName);
      s = make_shared<sRobotOperation>(_K, dt, true);
  }else{
      s = make_shared<sRobotOperation>(_K, dt, false);
  }

}

RobotOperation::~RobotOperation(){
}

void RobotOperation::sendToReal(bool activate){
  s->sendToBaxter = activate;
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
void RobotOperation::move(const arr& path, const arr& times, bool append){
  cout <<"SENDING MOTION: " <<path <<endl <<times <<endl;
  auto lock = s->stepMutex(RAI_HERE);
  arr _path = path.ref();
  if(_path.nd==1) _path.reshape(1,_path.N);
  s->spline.set(_path, times, getJointPositions(), append);
}

void RobotOperation::move(const arrA& poses, const arr& times, bool append){
  arr path(poses.N, poses(0).N);
  for(uint i=0;i<path.d0;i++) path[i] = poses(i);
  move(path, times, append);
}

double RobotOperation::timeToGo(){
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

arr RobotOperation::getHomePose(){ return s->q0; }

const StringA& RobotOperation::getJointNames(){
  return s->jointNames;
}

arr RobotOperation::getJointPositions(const StringA& joints){
  auto lock = s->stepMutex(RAI_HERE);
  if(s->useBaxter) return s->baxter.get_q();
  return s->K_ref.getJointState();
}

bool RobotOperation::getGripperGrabbed(const std::string& whichArm){
  // if(s->useBaxter) 
  return s->baxter.get_grabbed(whichArm);
}

bool RobotOperation::getGripperOpened(const std::string& whichArm){
  // if(s->useBaxter) 
  return s->baxter.get_opened(whichArm);
}

void RobotOperation::sync(rai::KinematicWorld& K){
  auto lock = s->stepMutex(RAI_HERE);
  K.setJointState(getJointPositions(), s->jointNames);
}

