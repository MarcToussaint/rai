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
  enum ControlMode { _position, _velocity, _acceleration };
  std::unique_ptr<struct Simulation_self> self;

  Configuration& C;
  double time=0.;
  SimulatorEngine engine;
  bool display;

  Simulation(Configuration& _C, SimulatorEngine _engine, bool _display=true);
  ~Simulation();

  void step(const arr& u_control, double tau=.01, ControlMode u_mode = _velocity);

  //-- store and reset the state of the simulation
  ptr<SimulationState> getState();
  void setState(const arr& frameState, const arr& frameVelocities=NoArr);
  void resetToPreviousState(const ptr<SimulationState>& state);
  void pushConfigurationToSimulator(const arr& frameVelocities=NoArr);

  //-- state information
  const arr& qdot();

  //-- sensor information
  CameraView& cameraview(); ///< use this if you want to initialize the sensor, etc
  void getImageAndDepth(byteA& image, floatA& depth); ///< use this during stepping
  void getSegmentation(byteA& segmentation);
  //use C.evalFeature to retrieve any geometric feature

};

}
