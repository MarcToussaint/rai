#ifdef RAI_PYBIND

#include "ry-Simulation.h"
#include "types.h"

#include "../Kin/simulation.h"

namespace rai{
  struct SimulationState {
    arr frameState;
    arr frameVels;

    SimulationState(const arr& _frameState, const arr& _frameVels) : frameState(_frameState), frameVels(_frameVels) {}
  };
}

void init_Simulation(pybind11::module &m) {
  pybind11::class_<ry::RySimulation>(m, "RySimulation")

  .def("step", [](ry::RySimulation& self, const std::vector<double>& u_control, double tau=.01, rai::Simulation::ControlMode u_mode) {
    arr u = conv_stdvec2arr(u_control);
    auto lock = self.config->set();
    self.sim->step(u, tau, u_mode);
  },
  "",
    pybind11::arg("u_control"),
    pybind11::arg("tau") = .01,
    pybind11::arg("u_mode") = rai::Simulation::_velocity
  )

  .def("openGripper", [](ry::RySimulation& self, const char* gripperFrameName, double width, double speed) {
     auto lock = self.config->set();
     self.sim->openGripper(gripperFrameName, width, speed);
  }, "",
    pybind11::arg("gripperFrameName"),
    pybind11::arg("width") = .075,
    pybind11::arg("speed") = .2
  )

  .def("closeGripper", [](ry::RySimulation& self, const char* gripperFrameName, double width, double speed, double force) {
    auto lock = self.config->set();
    self.sim->closeGripper(gripperFrameName, width, speed, force);
  }, "",
    pybind11::arg("gripperFrameName"),
    pybind11::arg("width") = .05,
    pybind11::arg("speed") = 1.,
    pybind11::arg("force") = 20.
  )



  .def("get_qDot", [](ry::RySimulation& self) {
    arr qdot = self.sim->qdot();
    return pybind11::array(qdot.dim(), qdot.p);
  })

  .def("getGripperWidth", [](ry::RySimulation& self, const char* gripperFrameName) {
    return self.sim->getGripperWidth(gripperFrameName);
  }, "",
    pybind11::arg("gripperFrameName")
  )

  .def("getGripperIsGrasped", [](ry::RySimulation& self, const char* gripperFrameName) {
    return self.sim->getGripperIsGrasped(gripperFrameName);
  }, "",
    pybind11::arg("gripperFrameName")
  )

  .def("getImageAndDepth", [](ry::RySimulation& self) {
    byteA rgb;
    floatA depth;
    self.sim->getImageAndDepth(rgb, depth);
    return pybind11::make_tuple(pybind11::array_t<byte>(rgb.dim(), rgb.p),
                                pybind11::array_t<float>(depth.dim(), depth.p));
  })



  .def("getState", [](ry::RySimulation& self) {
    auto lock = self.config->set();
    return self.sim->getState();
   })

  .def("getState", [](ry::RySimulation& self, const ptr<rai::SimulationState>& state) {
    auto lock = self.config->set();
    return self.sim->restoreState(state);
   })

  .def("setState", [](ry::RySimulation& self, const pybind11::array& frameState, const pybind11::array& frameVelocities) {
    arr X = numpy2arr(frameState);
    X.reshape(X.N/7, 7);
    arr V = numpy2arr(frameVelocities);
    V.reshape(V.N/6, 2, 3);
    auto lock = self.config->set();
    return self.sim->setState(X, V);
   }, "",
     pybind11::arg("frameState"),
     pybind11::arg("frameVelocities") = std::vector<double>()
   )

  .def("pushConfigurationToSimulator", [](ry::RySimulation& self, const std::vector<double>& frameVelocities) {
    auto lock = self.config->set();
    return self.sim->pushConfigurationToSimulator(conv_stdvec2arr(frameVelocities));
  }, "after you modified the configuration (from which you derived the RySimulation), use this to push the modified configuration back into the simulation engine.\n Optionally: with frameVelocities",
    pybind11::arg("frameVelocities") = std::vector<double>()
  )




//  .def("getSegmentation", [](ry::RySimulation& self) {
//    byteA seg;
//    self.sim->getSegmentation(seg);
//    return pybind11::array_t<byte>(seg.dim(), seg.p);
//  })

  ;
}

#endif
