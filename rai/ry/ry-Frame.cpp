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
#include "../Gui/opengl.h"

//void checkView(shared_ptr<rai::Frame>& self, bool recopyMeshes=false){
//  if(self->C.hasView()){
//    if(recopyMeshes) self->C.gl()->recopyMeshes(self->C);
//    self->C.view();
//  }
//}

void init_Frame(pybind11::module& m) {
  pybind11::class_<rai::Frame, shared_ptr<rai::Frame>>(m, "Frame", "todo doc")

    .def("setColor", &rai::Frame::setColor )

    .def("setPose",  [](shared_ptr<rai::Frame>& self, const char* pose){
       self->setPose(rai::Transformation(pose));
     })
    .def("setPosition", &rai::Frame::setPosition )
    .def("setQuaternion", &rai::Frame::setQuaternion )
    .def("setRelativePose", [](shared_ptr<rai::Frame>& self, const char* pose){
       self->setRelativePose(rai::Transformation(pose));
     })
    .def("setRelativePosition", &rai::Frame::setRelativePosition )
    .def("setRelativeQuaternion", &rai::Frame::setRelativeQuaternion )
    .def("setJoint", &rai::Frame::setJoint )
    .def("setJointState", &rai::Frame::setJointState )
    .def("setContact", &rai::Frame::setContact )
    .def("setMass", &rai::Frame::setMass )
    .def("setPointCloud", [](std::shared_ptr<rai::Frame>& self, const pybind11::array& points, const pybind11::array_t<byte>& colors) {
	arr _points = numpy2arr<double>(points);
	byteA _colors = numpy2arr<byte>(colors);
	if(self->C.viewer()->gl){
	  auto mux = self->C.gl().dataLock(RAI_HERE);
	  self->setPointCloud(_points, _colors);
	}else{
	  self->setPointCloud(_points, _colors);
	}
     }, "", pybind11::arg("points"), pybind11::arg("colors") = pybind11::array_t<byte>{} )

    .def("setShape", &rai::Frame::setShape, "", pybind11::arg("type"), pybind11::arg("size") )

    .def("setParent", &rai::Frame::setParent )
    .def("unLink", &rai::Frame::unLink )

    
    .def("addAttribute", &rai::Frame::addAttribute )
    .def("addAttributes",  [](shared_ptr<rai::Frame>& self, const pybind11::dict& D){
      if(!self->ats) self->ats = make_shared<rai::Graph>();
      self->ats->copy(dict2graph(D), true);
     }, "add/set attributes for the frame")

    .def("getAttributes", [](shared_ptr<rai::Frame>& self){
      if(!self->ats) self->ats = make_shared<rai::Graph>();
      return graph2dict(*self->ats);
     }, "get frame attributes")

    .def_readwrite("name", &rai::Frame::name )

    .def("getPosition", &rai::Frame::getPosition )
    .def("getQuaternion", &rai::Frame::getQuaternion )
    .def("getRotationMatrix", &rai::Frame::getRotationMatrix )
    .def("getRelativePosition", &rai::Frame::getRelativePosition )
    .def("getRelativeQuaternion", &rai::Frame::getRelativeQuaternion )
    .def("getJointState", &rai::Frame::getJointState )
    .def("getSize", &rai::Frame::getSize )
    .def("getMeshPoints", &rai::Frame::getMeshPoints )
    .def("getMeshTriangles", &rai::Frame::getMeshTriangles )

    .def("info", [](shared_ptr<rai::Frame>& self) {
	rai::Graph G;
	G.add<rai::String>("name", self->name);
	G.add<int>("ID", self->ID);
	self->write(G);
	if(!G["X"]) G.add<arr>("X", self->ensure_X().getArr7d());
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
      })
    ;

  //===========================================================================

#undef ENUMVAL
#define ENUMVAL(pre, x) .value(#x, pre##_##x)

  pybind11::enum_<rai::JointType>(m, "JT")

  ENUMVAL(rai::JT,hingeX) ENUMVAL(rai::JT,hingeY) ENUMVAL(rai::JT,hingeZ) ENUMVAL(rai::JT,transX) ENUMVAL(rai::JT,transY) ENUMVAL(rai::JT,transZ) ENUMVAL(rai::JT,transXY) ENUMVAL(rai::JT,trans3) ENUMVAL(rai::JT,transXYPhi) ENUMVAL(rai::JT,transYPhi) ENUMVAL(rai::JT,universal) ENUMVAL(rai::JT,rigid) ENUMVAL(rai::JT,quatBall) ENUMVAL(rai::JT,phiTransXY) ENUMVAL(rai::JT,XBall) ENUMVAL(rai::JT,free) ENUMVAL(rai::JT,generic) ENUMVAL(rai::JT,tau)
  .export_values();

}

#endif
