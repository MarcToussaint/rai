#ifdef RAI_PYBIND

#include "ry-Frame.h"
#include "types.h"

#include "../Core/thread.h"
#include "../Kin/kin.h"
#include "../Kin/frame.h"

void init_Frame(pybind11::module &m) {
pybind11::class_<ry::RyFrame>(m, "Frame")
.def("setPointCloud", [](ry::RyFrame& self, const pybind11::array& points, const pybind11::array_t<byte>& colors) {
  arr _points = numpy2arr<double>(points);
  byteA _colors = numpy2arr<byte>(colors);
  WToken<rai::Configuration> token(*self.config, &self.config->data);
  self.frame->setPointCloud(_points, _colors);
})

.def("setShape", [](ry::RyFrame& self, rai::ShapeType shape, const std::vector<double>& size) {
  WToken<rai::Configuration> token(*self.config, &self.config->data);
  self.frame->setShape(shape, size);
},
pybind11::arg("type"),
pybind11::arg("size")
    )

.def("setColor", [](ry::RyFrame& self, const std::vector<double>& color) {
  WToken<rai::Configuration> token(*self.config, &self.config->data);
  self.frame->setColor(color);
})

.def("setPose", [](ry::RyFrame& self, const std::string& pose) {
  WToken<rai::Configuration> token(*self.config, &self.config->data);
  self.frame->set_X()->setText(pose.c_str());
})

.def("setPosition", [](ry::RyFrame& self, const std::vector<double>& pos) {
  WToken<rai::Configuration> token(*self.config, &self.config->data);
  self.frame->setPosition(pos);
})

.def("setQuaternion", [](ry::RyFrame& self, const std::vector<double>& quat) {
  WToken<rai::Configuration> token(*self.config, &self.config->data);
  self.frame->setQuaternion(quat);
})

.def("setRelativePose", [](ry::RyFrame& self, const std::string& pose) {
  WToken<rai::Configuration> token(*self.config, &self.config->data);
  self.frame->set_Q()->setText(pose.c_str());
})

.def("setRelativePosition", [](ry::RyFrame& self, const std::vector<double>& pos) {
  WToken<rai::Configuration> token(*self.config, &self.config->data);
  self.frame->setRelativePosition(pos);
})

.def("setRelativeQuaternion", [](ry::RyFrame& self, const std::vector<double>& quat) {
  WToken<rai::Configuration> token(*self.config, &self.config->data);
  self.frame->setRelativeQuaternion(quat);
})

.def("setJoint", [](ry::RyFrame& self, rai::JointType jointType) {
  WToken<rai::Configuration> token(*self.config, &self.config->data);
  self.frame->setJoint(jointType);
})

.def("setContact", [](ry::RyFrame& self, int cont) {
  WToken<rai::Configuration> token(*self.config, &self.config->data);
  self.frame->setContact(cont);
})

.def("setMass", [](ry::RyFrame& self, double mass) {
  WToken<rai::Configuration> token(*self.config, &self.config->data);
  self.frame->setMass(mass);
})

.def("addAttribute", [](ry::RyFrame& self, const char* key, double value) {
  WToken<rai::Configuration> token(*self.config, &self.config->data);
  self.frame->addAttribute(key, value);
})

.def("getPosition", [](ry::RyFrame& self) {
  RToken<rai::Configuration> token(*self.config, &self.config->data);
  arr x = self.frame->getPosition();
  return pybind11::array_t<double>(x.dim(), x.p);
})

.def("getQuaternion", [](ry::RyFrame& self) {
  RToken<rai::Configuration> token(*self.config, &self.config->data);
  arr x = self.frame->getQuaternion();
  return pybind11::array_t<double>(x.dim(), x.p);
})

.def("getRotationMatrix", [](ry::RyFrame& self) {
  RToken<rai::Configuration> token(*self.config, &self.config->data);
  arr x = self.frame->getRotationMatrix();
  return pybind11::array_t<double>(x.dim(), x.p);
})

.def("getRelativePosition", [](ry::RyFrame& self) {
  RToken<rai::Configuration> token(*self.config, &self.config->data);
  arr x = self.frame->getRelativePosition();
  return pybind11::array_t<double>(x.dim(), x.p);
})

.def("getRelativeQuaternion", [](ry::RyFrame& self) {
  RToken<rai::Configuration> token(*self.config, &self.config->data);
  arr x = self.frame->getRelativeQuaternion();
  return pybind11::array_t<double>(x.dim(), x.p);
})

.def("getSize", [](ry::RyFrame& self) {
  RToken<rai::Configuration> token(*self.config, &self.config->data);
  arr x = self.frame->getSize();
  return pybind11::array_t<double>(x.dim(), x.p);
})

.def("getMeshPoints", [](ry::RyFrame& self) {
  RToken<rai::Configuration> token(*self.config, &self.config->data);
  arr x = self.frame->getMeshPoints();
  return pybind11::array_t<double>(x.dim(), x.p);
})

.def("info", [](ry::RyFrame& self) {
  rai::Graph G;
  WToken<rai::Configuration> token(*self.config, &self.config->data);
  G.newNode<rai::String>("name", {}, self.frame->name);
  G.newNode<int>("ID", {}, self.frame->ID);
  self.frame->write(G);
  if(!G["X"]) G.newNode<arr>("X", {}, self.frame->ensure_X().getArr7d());
  return graph2dict(G);
})

.def("setMeshAsLines", [](ry::RyFrame& self, const std::vector<double>& lines) {
  CHECK(self.frame, "this is not a valid frame");
  CHECK(self.frame->shape, "this frame is not a mesh!");
  CHECK_EQ(self.frame->shape->type(), rai::ST_mesh, "this frame is not a mesh!");
  uint n = lines.size()/3;
  self.frame->shape->mesh().V = lines;
  self.frame->shape->mesh().V.reshape(n, 3);
  uintA& T = self.frame->shape->mesh().T;
  T.resize(n/2, 2);
  for(uint i=0; i<T.d0; i++) {
    T(i, 0) = 2*i;
    T(i, 1) = 2*i+1;
  }
})
;
}

#endif
