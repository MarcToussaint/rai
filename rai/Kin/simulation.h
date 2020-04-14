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

struct Simulation {
  enum SimulatorEngine { _physx, _bullet, _kinematic };
  enum ControlMode { _none, _position, _velocity, _acceleration };
  std::unique_ptr<struct Simulation_self> self;

  Configuration& C;
  double time=0.;
  SimulatorEngine engine;
  bool display;

  Simulation(Configuration& _C, SimulatorEngine _engine, bool _display=true);
  ~Simulation();


  //== controller interface

  //-- send a low-level control and step the simulation
  void step(const arr& u_control, double tau=.01, ControlMode u_mode = _velocity);

  //-- send a gripper command
  void openGripper(const char* gripperFrameName, double width=.075, double speed=.2);
  void closeGripper(const char* gripperFrameName, double width=.05, double speed=.1, double force=20.);

  //-- get state information
  const arr& qdot();
  double getGripperWidth(const char* gripperFrameName);
  bool getGripperIsGrasped(const char* gripperFrameName);

  //-- get sensor information
  CameraView& cameraview(); ///< use this if you want to initialize the sensor, etc
  void getImageAndDepth(byteA& image, floatA& depth); ///< use this during stepping
  void getSegmentation(byteA& segmentation);
  //use C.evalFeature to retrieve any geometric feature


  //== perturbation/adversarial interface

  //what are really the fundamental perturbations? Their (reactive?) management should be realized by 'agents'. We need a method to add purturbation agents.

  //displace object



  //== management interface

  //-- store and reset the state of the simulation
  ptr<SimulationState> getState();
  void setState(const arr& frameState, const arr& frameVelocities=NoArr);
  void resetToPreviousState(const ptr<SimulationState>& state);
  void pushConfigurationToSimulator(const arr& frameVelocities=NoArr);

};

}
