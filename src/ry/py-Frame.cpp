/*  ------------------------------------------------------------------
    Copyright (c) 2011-2024 Marc Toussaint
    email: toussaint@tu-berlin.de

    This code is distributed under the MIT License.
    Please see <root-path>/LICENSE for details.
    --------------------------------------------------------------  */

#ifdef RAI_PYBIND

#include "py-Frame.h"
#include "types.h"

#include "../Core/thread.h"
#include "../Kin/kin.h"
#include "../Kin/frame.h"
#include "../Kin/viewer.h"
#include "../Gui/opengl.h"
#include "../KOMO/skeletonSymbol.h"
#include "../Kin/simulation.h"

//void checkView(shared_ptr<rai::Frame>& self, bool recopyMeshes=false){
//  if(self->C.hasView()){
//    if(recopyMeshes) self->C.gl()->recopyMeshes(self->C);
//    self->C.view();
//  }
//}

void init_Frame(pybind11::module& m) {
  pybind11::class_<rai::Frame, shared_ptr<rai::Frame>>(m, "Frame", "A (coordinate) frame of a configuration, which can have a parent, and associated shape, joint, and/or inertia")

  .def("setPose",  [](shared_ptr<rai::Frame>& self, const char* pose) { self->setPose(rai::Transformation(pose)); }, "")
  .def("setPosition", &rai::Frame::setPosition, "")
  .def("setQuaternion", &rai::Frame::setQuaternion, "")
  .def("setRelativePose", [](shared_ptr<rai::Frame>& self, const char* pose) { self->setRelativePose(rai::Transformation(pose)); }, "")
  .def("setRelativePosition", &rai::Frame::setRelativePosition, "")
  .def("setRelativeQuaternion", &rai::Frame::setRelativeQuaternion, "")
  .def("setJoint", &rai::Frame::setJoint, "", pybind11::arg("jointType"), pybind11::arg("limits")=arr{})
  .def("setJointState", &rai::Frame::setJointState, "")
  .def("setContact", &rai::Frame::setContact, "")
  .def("setMass", &rai::Frame::setMass, "")
  .def("setColor", &rai::Frame::setColor, "")
  .def("setShape", &rai::Frame::setShape, "", pybind11::arg("type"), pybind11::arg("size"))
  .def("setMesh", &rai::Frame::setMesh,
       "attach a mesh shape",
       pybind11::arg("vertices"), pybind11::arg("triangles"), pybind11::arg("colors")=byteA{}, pybind11::arg("cvxParts")=uintA{})
  .def("setPointCloud", &rai::Frame::setPointCloud,
       "attach a point cloud shape",
       pybind11::arg("points"), pybind11::arg("colors") = byteA{}, pybind11::arg("normals")=arr{})
  .def("setConvexMesh", &rai::Frame::setConvexMesh,
       "attach a convex mesh as shape",
       pybind11::arg("points"), pybind11::arg("colors") = byteA{}, pybind11::arg("radius")=0.)
  .def("setDensity", &rai::Frame::setDensity, "", pybind11::arg("data"), pybind11::arg("size"))
  .def("setImplicitSurface", &rai::Frame::setImplicitSurface, "", pybind11::arg("data"), pybind11::arg("size"), pybind11::arg("blur"), pybind11::arg("resample")=-1.)

  .def("setParent", &rai::Frame::setParent, "", pybind11::arg("parent"), pybind11::arg("keepAbsolutePose_and_adaptRelativePose") = false, pybind11::arg("checkForLoop") = false)
  .def("unLink", &rai::Frame::unLink, "")
  .def("makeRoot", &rai::Frame::makeRoot, "", pybind11::arg("untilPartBreak"))

  .def("setAttribute", &rai::Frame::setAttribute, "")
  .def("addAttributes",  [](shared_ptr<rai::Frame>& self, const pybind11::dict& D) {
    if(!self->ats) self->ats = make_shared<rai::Graph>();
    self->ats->copy(dict2graph(D), true);
  }, "add/set attributes for the frame")

  .def("getAttributes", [](shared_ptr<rai::Frame>& self) {
    if(!self->ats) self->ats = make_shared<rai::Graph>();
    return graph2dict(*self->ats);
  }, "get frame attributes")

  .def_readwrite("name", &rai::Frame::name)

  .def("getParent", [](shared_ptr<rai::Frame>& self) { return shared_ptr<rai::Frame>(self->parent, &null_deleter); }, "")
  .def("getPose", &rai::Frame::getPose, "")
  .def("getPosition", &rai::Frame::getPosition, "")
  .def("getQuaternion", &rai::Frame::getQuaternion, "")
  .def("getRotationMatrix", [](shared_ptr<rai::Frame>& self) { return self->ensure_X().rot.getArr(); }, "")
  .def("getTransform", &rai::Frame::getTransform, "")
  .def("getRelativePosition", &rai::Frame::getRelativePosition, "")
  .def("getRelativeQuaternion", &rai::Frame::getRelativeQuaternion, "")
  .def("getRelativeTransform", &rai::Frame::getRelativeTransform, "")
  .def("getJointState", &rai::Frame::getJointState, "")
  .def("getSize", &rai::Frame::getSize, "")
  .def("getShapeType", &rai::Frame::getShapeType, "")
  .def("getMeshPoints", &rai::Frame::getMeshPoints, "")
  .def("getMeshTriangles", &rai::Frame::getMeshTriangles, "")
  .def("getMesh", [](shared_ptr<rai::Frame>& self){
      return pybind11::make_tuple(Array2numpy<double>(self->getMeshPoints()),
                                  Array2numpy<uint>(self->getMeshTriangles()),
                                  Array2numpy<byte>(self->getMeshColors()) );
  }, "")

  .def("info", [](shared_ptr<rai::Frame>& self) {
    rai::Graph G;
    G.add<str>("name", self->name);
    G.add<int>("ID", self->ID);
    self->write(G);
    if(!G["X"]) G.add<arr>("X", self->ensure_X().getArr7d());
    return graph2dict(G);
  }, "")

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
  }, "")
  ;

}

//===========================================================================

void init_enums(pybind11::module& m) {

#undef ENUMVAL
//#define ENUMVAL(x) m.attr(#x) = pybind11::int_(int(rai::x));
#define ENUMVAL(x) .value(#x, rai::x)

  pybind11::enum_<rai::ArgWord>(m, "ArgWord", "[todo: replace by str]")
  ENUMVAL(_left)
  ENUMVAL(_right)
  ENUMVAL(_sequence)
  ENUMVAL(_path)
  .export_values();

#undef ENUMVAL
#define ENUMVAL(pre, x) .value(#x, pre##_##x)

  pybind11::enum_<rai::JointType>(m, "JT")
  ENUMVAL(rai::JT, none) ENUMVAL(rai::JT, hingeX) ENUMVAL(rai::JT, hingeY) ENUMVAL(rai::JT, hingeZ) ENUMVAL(rai::JT, transX) ENUMVAL(rai::JT, transY) ENUMVAL(rai::JT, transZ) ENUMVAL(rai::JT, transXY) ENUMVAL(rai::JT, trans3) ENUMVAL(rai::JT, transXYPhi) ENUMVAL(rai::JT, transYPhi) ENUMVAL(rai::JT, universal) ENUMVAL(rai::JT, rigid) ENUMVAL(rai::JT, quatBall) ENUMVAL(rai::JT, phiTransXY) ENUMVAL(rai::JT, XBall) ENUMVAL(rai::JT, free) ENUMVAL(rai::JT, generic) ENUMVAL(rai::JT, tau)
  ;

  pybind11::enum_<rai::ShapeType>(m, "ST")
  ENUMVAL(rai::ST, none)
  ENUMVAL(rai::ST, box)
  ENUMVAL(rai::ST, sphere)
  ENUMVAL(rai::ST, capsule)
  ENUMVAL(rai::ST, mesh)
  ENUMVAL(rai::ST, cylinder)
  ENUMVAL(rai::ST, marker)
  ENUMVAL(rai::ST, pointCloud)
  ENUMVAL(rai::ST, ssCvx)
  ENUMVAL(rai::ST, ssBox)
  ENUMVAL(rai::ST, ssCylinder)
  ENUMVAL(rai::ST, ssBoxElip)
  ENUMVAL(rai::ST, quad)
  ENUMVAL(rai::ST, camera)
  ENUMVAL(rai::ST, sdf)
  ;

  pybind11::enum_<FeatureSymbol>(m, "FS")
  ENUMVAL(FS, position)
  ENUMVAL(FS, positionDiff)
  ENUMVAL(FS, positionRel)
  ENUMVAL(FS, quaternion)
  ENUMVAL(FS, quaternionDiff)
  ENUMVAL(FS, quaternionRel)
  ENUMVAL(FS, pose)
  ENUMVAL(FS, poseDiff)
  ENUMVAL(FS, poseRel)
  ENUMVAL(FS, vectorX)
  ENUMVAL(FS, vectorXDiff)
  ENUMVAL(FS, vectorXRel)
  ENUMVAL(FS, vectorY)
  ENUMVAL(FS, vectorYDiff)
  ENUMVAL(FS, vectorYRel)
  ENUMVAL(FS, vectorZ)
  ENUMVAL(FS, vectorZDiff)
  ENUMVAL(FS, vectorZRel)
  ENUMVAL(FS, scalarProductXX)
  ENUMVAL(FS, scalarProductXY)
  ENUMVAL(FS, scalarProductXZ)
  ENUMVAL(FS, scalarProductYX)
  ENUMVAL(FS, scalarProductYY)
  ENUMVAL(FS, scalarProductYZ)
  ENUMVAL(FS, scalarProductZZ)
  ENUMVAL(FS, gazeAt)

  ENUMVAL(FS, angularVel)

  ENUMVAL(FS, accumulatedCollisions)
  ENUMVAL(FS, jointLimits)
  ENUMVAL(FS, distance)
  ENUMVAL(FS, negDistance)
  ENUMVAL(FS, oppose)

  ENUMVAL(FS, qItself)
  ENUMVAL(FS, jointState)

  ENUMVAL(FS, aboveBox)
  ENUMVAL(FS, insideBox)

  ENUMVAL(FS, pairCollision_negScalar)
  ENUMVAL(FS, pairCollision_vector)
  ENUMVAL(FS, pairCollision_normal)
  ENUMVAL(FS, pairCollision_p1)
  ENUMVAL(FS, pairCollision_p2)

  ENUMVAL(FS, standingAbove)

  ENUMVAL(FS, physics)
  ENUMVAL(FS, contactConstraints)
  ENUMVAL(FS, energy)

  ENUMVAL(FS, transAccelerations)
  ENUMVAL(FS, transVelocities)
  ;

  pybind11::enum_<rai::SkeletonSymbol>(m, "SY")
  ENUMVAL(rai::SY, touch) ENUMVAL(rai::SY, above) ENUMVAL(rai::SY, inside) ENUMVAL(rai::SY, oppose) ENUMVAL(rai::SY, restingOn)
  ENUMVAL(rai::SY, poseEq) ENUMVAL(rai::SY, positionEq) ENUMVAL(rai::SY, stableRelPose) ENUMVAL(rai::SY, stablePose)
  ENUMVAL(rai::SY, stable) ENUMVAL(rai::SY, stableOn) ENUMVAL(rai::SY, dynamic) ENUMVAL(rai::SY, dynamicOn) ENUMVAL(rai::SY, dynamicTrans) ENUMVAL(rai::SY, quasiStatic) ENUMVAL(rai::SY, quasiStaticOn) ENUMVAL(rai::SY, downUp) ENUMVAL(rai::SY, stableZero)
  ENUMVAL(rai::SY, contact) ENUMVAL(rai::SY, contactStick) ENUMVAL(rai::SY, contactComplementary) ENUMVAL(rai::SY, bounce) ENUMVAL(rai::SY, push)
  ENUMVAL(rai::SY, magic) ENUMVAL(rai::SY, magicTrans)
  ENUMVAL(rai::SY, pushAndPlace)
  ENUMVAL(rai::SY, topBoxGrasp) ENUMVAL(rai::SY, topBoxPlace)
  ENUMVAL(rai::SY, dampMotion)
  ENUMVAL(rai::SY, identical)
  ENUMVAL(rai::SY, alignByInt)
  ENUMVAL(rai::SY, makeFree) ENUMVAL(rai::SY, forceBalance)
  ENUMVAL(rai::SY, relPosY)
  ENUMVAL(rai::SY, touchBoxNormalX) ENUMVAL(rai::SY, touchBoxNormalY) ENUMVAL(rai::SY, touchBoxNormalZ)
  ENUMVAL(rai::SY, boxGraspX) ENUMVAL(rai::SY, boxGraspY) ENUMVAL(rai::SY, boxGraspZ)
  ENUMVAL(rai::SY, lift)
  ENUMVAL(rai::SY, stableYPhi)
  ENUMVAL(rai::SY, stableOnX)
  ENUMVAL(rai::SY, stableOnY)
  ENUMVAL(rai::SY, end)
  ;

#undef ENUMVAL
#define ENUMVAL(x) .value(#x, rai::Simulation::_##x)

  pybind11::enum_<rai::Simulation::Engine>(m, "SimulationEngine")
  ENUMVAL(physx)
  ENUMVAL(bullet)
  ENUMVAL(kinematic)
  ;

  pybind11::enum_<rai::Simulation::ControlMode>(m, "ControlMode")
  ENUMVAL(none)
  ENUMVAL(position)
  ENUMVAL(velocity)
  ENUMVAL(acceleration)
  ENUMVAL(spline)
  ;

}

#endif
