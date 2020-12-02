/*  ------------------------------------------------------------------
    Copyright (c) 2011-2020 Marc Toussaint
    email: toussaint@tu-berlin.de

    This code is distributed under the MIT License.
    Please see <root-path>/LICENSE for details.
    --------------------------------------------------------------  */

#ifdef RAI_PYBIND

#include "ry-Frame.h"
#include "types.h"

#include "../Core/thread.h"
#include "../Kin/kin.h"
#include "../Kin/frame.h"
#include "../Kin/viewer.h"

void checkView(shared_ptr<rai::Frame>& self, bool recopyMeshes=false){
  if(self->C.hasView()){
    if(recopyMeshes) self->C.gl()->recopyMeshes(self->C);
    self->C.watch();
  }
}

void init_Frame(pybind11::module& m) {
//  pybind11::class_<ry::RyFrame>(m, "Frame")
  pybind11::class_<rai::Frame, shared_ptr<rai::Frame>>(m, "Frame")

  .def("setPointCloud", [](shared_ptr<rai::Frame>& self, const pybind11::array& points, const pybind11::array_t<byte>& colors) {
    arr _points = numpy2arr<double>(points);
    byteA _colors = numpy2arr<byte>(colors);
    //WToken<rai::Configuration> token(*self.config, &self.config->data);
    self->setPointCloud(_points, _colors);
    checkView(self, true);
  })

  .def("setShape", [](shared_ptr<rai::Frame>& self, rai::ShapeType shape, const std::vector<double>& size) {
    //WToken<rai::Configuration> token(*self.config, &self.config->data);
    self->setShape(shape, arr(size, true));
    checkView(self, true);
  },
  pybind11::arg("type"),
  pybind11::arg("size")
      )

  .def("setColor", [](shared_ptr<rai::Frame>& self, const std::vector<double>& color) {
    //WToken<rai::Configuration> token(*self.config, &self.config->data);
    self->setColor( arr(color, true) );
    checkView(self, true);
  })

  .def("setPose", [](shared_ptr<rai::Frame>& self, const std::string& pose) {
    //WToken<rai::Configuration> token(*self.config, &self.config->data);
    self->set_X()->setText(pose.c_str());
    checkView(self);
  })

  .def("setPosition", [](shared_ptr<rai::Frame>& self, const std::vector<double>& pos) {
    //WToken<rai::Configuration> token(*self.config, &self.config->data);
    self->setPosition( arr(pos, true) );
    checkView(self);
  })

  .def("setQuaternion", [](shared_ptr<rai::Frame>& self, const std::vector<double>& quat) {
    //WToken<rai::Configuration> token(*self.config, &self.config->data);
    self->setQuaternion( arr(quat, true) );
    checkView(self);
  })

  .def("setRelativePose", [](shared_ptr<rai::Frame>& self, const std::string& pose) {
    //WToken<rai::Configuration> token(*self.config, &self.config->data);
    self->set_Q()->setText(pose.c_str());
    checkView(self);
  })

  .def("setRelativePosition", [](shared_ptr<rai::Frame>& self, const std::vector<double>& pos) {
    //WToken<rai::Configuration> token(*self.config, &self.config->data);
    self->setRelativePosition( arr(pos, true) );
    checkView(self);
  })

  .def("setRelativeQuaternion", [](shared_ptr<rai::Frame>& self, const std::vector<double>& quat) {
    //WToken<rai::Configuration> token(*self.config, &self.config->data);
    self->setRelativeQuaternion( arr(quat, true) );
    checkView(self);
  })

  .def("setJoint", [](shared_ptr<rai::Frame>& self, rai::JointType jointType) {
    //WToken<rai::Configuration> token(*self.config, &self.config->data);
    self->setJoint(jointType);
    checkView(self);
  })

  .def("setJointState", [](shared_ptr<rai::Frame>& self, const std::vector<double>& q) {
    //WToken<rai::Configuration> token(*self.config, &self.config->data);
    self->setJointState( arr(q, true) );
    checkView(self);
  })

  .def("setContact", [](shared_ptr<rai::Frame>& self, int cont) {
    //WToken<rai::Configuration> token(*self.config, &self.config->data);
    self->setContact(cont);
    checkView(self);
  })

  .def("setMass", [](shared_ptr<rai::Frame>& self, double mass) {
    //WToken<rai::Configuration> token(*self.config, &self.config->data);
    self->setMass(mass);
    checkView(self);
  })

  .def("addAttribute", [](shared_ptr<rai::Frame>& self, const char* key, double value) {
    //WToken<rai::Configuration> token(*self.config, &self.config->data);
    self->addAttribute(key, value);
  })

  .def("getName", [](shared_ptr<rai::Frame>& self) {
    std::string str(self->name.p);
    return str;
  })

  .def("getPosition", [](shared_ptr<rai::Frame>& self) {
    //RToken<rai::Configuration> token(*self.config, &self.config->data);
    arr x = self->getPosition();
    return pybind11::array_t<double>(x.dim(), x.p);
  })

  .def("getQuaternion", [](shared_ptr<rai::Frame>& self) {
    //RToken<rai::Configuration> token(*self.config, &self.config->data);
    arr x = self->getQuaternion();
    return pybind11::array_t<double>(x.dim(), x.p);
  })

  .def("getRotationMatrix", [](shared_ptr<rai::Frame>& self) {
    //RToken<rai::Configuration> token(*self.config, &self.config->data);
    arr x = self->getRotationMatrix();
    return pybind11::array_t<double>(x.dim(), x.p);
  })

  .def("getRelativePosition", [](shared_ptr<rai::Frame>& self) {
    //RToken<rai::Configuration> token(*self.config, &self.config->data);
    arr x = self->getRelativePosition();
    return pybind11::array_t<double>(x.dim(), x.p);
  })

  .def("getRelativeQuaternion", [](shared_ptr<rai::Frame>& self) {
    //RToken<rai::Configuration> token(*self.config, &self.config->data);
    arr x = self->getRelativeQuaternion();
    return pybind11::array_t<double>(x.dim(), x.p);
  })

  .def("getJointState", [](shared_ptr<rai::Frame>& self) {
    //RToken<rai::Configuration> token(*self.config, &self.config->data);
    arr x = self->getJointState();
    return pybind11::array_t<double>(x.dim(), x.p);
  })

  .def("getSize", [](shared_ptr<rai::Frame>& self) {
    //RToken<rai::Configuration> token(*self.config, &self.config->data);
    arr x = self->getSize();
    return pybind11::array_t<double>(x.dim(), x.p);
  })

  .def("getMeshPoints", &rai::Frame::getMeshPoints)
  .def("getMeshTriangles", &rai::Frame::getMeshTriangles)

  .def("info", [](shared_ptr<rai::Frame>& self) {
    rai::Graph G;
    //WToken<rai::Configuration> token(*self.config, &self.config->data);
    G.newNode<rai::String>("name", {}, self->name);
    G.newNode<int>("ID", {}, self->ID);
    self->write(G);
    if(!G["X"]) G.newNode<arr>("X", {}, self->ensure_X().getArr7d());
    return graph2dict(G);
  })

  .def("setMeshAsLines", [](shared_ptr<rai::Frame>& self, const std::vector<double>& lines) {
//    CHECK(self.frame, "this is not a valid frame");
    CHECK(self->shape, "this frame is not a mesh!");
    CHECK_EQ(self->shape->type(), rai::ST_mesh, "this frame is not a mesh!");
    uint n = lines.size()/3;
    self->shape->mesh().V = lines;
    self->shape->mesh().V.reshape(n, 3);
    uintA& T = self->shape->mesh().T;
    T.resize(n/2, 2);
    for(uint i=0; i<T.d0; i++) {
      T(i, 0) = 2*i;
      T(i, 1) = 2*i+1;
    }
    checkView(self, true);
  })
  ;
}

#endif
