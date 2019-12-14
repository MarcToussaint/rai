#ifdef RAI_PYBIND

#include "ry-Operate.h"
#include "ry-Config.h"
#include "types.h"

#include <Operate/robotOperation.h>

void init_Operate(pybind11::module &m) {
pybind11::class_<ry::RyOperate>(m, "RyOperate")
.def("move", [](ry::RyOperate& self, const std::vector<std::vector<double>>& poses, const std::vector<double>& times, bool append) {
  arr path(poses.size(), poses[0].size());
  for(uint i=0; i<path.d0; i++) path[i] = conv_stdvec2arr(poses[i]);
  self.R->move(path, conv_stdvec2arr(times), append);
})

.def("move", [](ry::RyOperate& self, const pybind11::array& path, const std::vector<double>& times, bool append) {
  arr _path = numpy2arr(path);
  self.R->move(_path, conv_stdvec2arr(times), append);
})

.def("moveHard", [](ry::RyOperate& self, const pybind11::array& pose) {
  arr _pose = numpy2arr(pose);
  self.R->moveHard(_pose);
})

.def("timeToGo", [](ry::RyOperate& self) {
  return self.R->timeToGo();
})

.def("wait", [](ry::RyOperate& self) {
  return self.R->wait();
})

.def("getJointPositions", [](ry::RyOperate& self, ry::Config& C) {
  arr q = self.R->getJointPositions();
  return pybind11::array(q.dim(), q.p);
})

.def("getGripperGrabbed", [](ry::RyOperate& self, const std::string& whichArm) {
  return self.R->getGripperGrabbed(whichArm);
})

.def("getGripperOpened", [](ry::RyOperate& self, const std::string& whichArm) {
  return self.R->getGripperOpened(whichArm);
})

.def("sendToReal", [](ry::RyOperate& self, bool activate) {
  self.R->sendToReal(activate);
})

.def("sync", [](ry::RyOperate& self, ry::Config& C) {
  self.R->sync(C.set());
})
;
}

#endif
