#ifdef RAI_PYBIND

#include "ry-Simulation.h"
//#include "ry-Config.h"
#include "types.h"

#include <Kin/simulation.h>

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
    self.sim->step(u, tau, u_mode);
  },
  "",
  pybind11::arg("u_control"),
      pybind11::arg("tau") = .01,
      pybind11::arg("u_mode") = rai::Simulation::_velocity
                                )
  .def("setState", [](ry::RySimulation& self, const ptr<rai::SimulationState>& state) {
    return self.sim->setState(state);
   })

  .def("getState", [](ry::RySimulation& self) {
    return self.sim->getState();
  })


  .def("get_qDot", [](ry::RySimulation& self) {
    arr qdot = self.sim->qdot();
    return pybind11::array(qdot.dim(), qdot.p);
  })

  .def("getImageAndDepth", [](ry::RySimulation& self) {
    byteA rgb;
    floatA depth;
    self.sim->getImageAndDepth(rgb, depth);
    return pybind11::make_tuple(pybind11::array_t<byte>(rgb.dim(), rgb.p),
                                pybind11::array_t<float>(depth.dim(), depth.p));
  })

//  .def("getSegmentation", [](ry::RySimulation& self) {
//    byteA seg;
//    self.sim->getSegmentation(seg);
//    return pybind11::array_t<byte>(seg.dim(), seg.p);
//  })

  ;
}

#endif
