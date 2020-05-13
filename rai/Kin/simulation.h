/*  ------------------------------------------------------------------
    Copyright (c) 2019 Marc Toussaint
    email: marc.toussaint@informatik.uni-stuttgart.de

    This code is distributed under the MIT License.
    Please see <root-path>/LICENSE for details.
    --------------------------------------------------------------  */

#include "kin.h"
#include "cameraview.h"

namespace rai {

struct SimulationState;
struct SimulationImp;

struct Simulation {
  enum SimulatorEngine { _physx, _bullet, _kinematic };
  enum ControlMode { _none, _position, _velocity, _acceleration };
  enum ImpType { _closeGripper, _openGripper, _depthNoise, _rgbNoise, _adversarialDropper, _objectImpulses };

  std::unique_ptr<struct Simulation_self> self;

  Configuration& C;
  double time;
  SimulatorEngine engine;
  Array<ptr<SimulationImp>> imps; ///< list of (adversarial) imps doing things/perturbations/noise in addition to clean physics engine
  int verbose;
  FrameL grasps;

  Simulation(Configuration& _C, SimulatorEngine _engine, int _verbose=1);
  ~Simulation();


  //== controller interface

  //-- send a low-level control and step the simulation
  void step(const arr& u_control, double tau=.01, ControlMode u_mode = _velocity);

  //-- send a gripper command
  void openGripper(const char* gripperFrameName, double width=.075, double speed=.2);
  void closeGripper(const char* gripperFrameName, double width=.05, double speed=.1, double force=20.);

  //-- get state information
  const arr& get_q(){ return C.getJointState(); }
  const arr& get_qDot();
  double getGripperWidth(const char* gripperFrameName);
  bool getGripperIsGrasping(const char* gripperFrameName);

  //-- get sensor information
  void getImageAndDepth(byteA& image, floatA& depth); ///< use this during stepping
  void getSegmentation(byteA& segmentation);
  CameraView& cameraview(); ///< use this if you want to initialize the sensor, etc
  rai::CameraView::Sensor& addSensor(const char* frameAttached){ return cameraview().addSensor(frameAttached); }


  //== perturbation/adversarial interface

  //what are really the fundamental perturbations? Their (reactive?) management should be realized by 'agents'. We need a method to add purturbation agents.
  void addImp(ImpType type, const StringA& frames, const arr& parameters);

  //displace object



  //== management interface

  //-- store and reset the state of the simulation
  ptr<SimulationState> getState();
  void restoreState(const ptr<SimulationState>& state);
  void setState(const arr& frameState, const arr& frameVelocities=NoArr);
  void pushConfigurationToSimulator(const arr& frameVelocities=NoArr);

};

}
