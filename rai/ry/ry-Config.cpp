/*  ------------------------------------------------------------------
    Copyright (c) 2011-2020 Marc Toussaint
    email: toussaint@tu-berlin.de

    This code is distributed under the MIT License.
    Please see <root-path>/LICENSE for details.
    --------------------------------------------------------------  */

#ifdef RAI_PYBIND

#include "ry-Config.h"
#include "ry-Feature.h"
#include "ry-Frame.h"
#include "ry-KOMO.h"
#include "ry-LGP_Tree.h"
#include "ry-Bullet.h"
#include "ry-PhysX.h"
#include "ry-Operate.h"
#include "ry-Simulation.h"

#include "types.h"

#include "../Kin/kin.h"
#include "../Kin/forceExchange.h"
#include "../Kin/kin_bullet.h"
#include "../Kin/kin_physx.h"
#include "../Operate/robotOperation.h"
#include "../Kin/proxy.h"
#include "../Kin/viewer.h"
#include "../Kin/cameraview.h"
#include "../Kin/simulation.h"
#include "../Gui/viewer.h"
#include "../LGP/LGP_tree.h"

void checkView(shared_ptr<rai::Configuration>& self){ if(self->hasView()) self->watch(); }
void null_deleter(rai::Frame*){}

void init_Config(pybind11::module& m) {
  pybind11::class_<rai::Configuration, shared_ptr<rai::Configuration>>(m, "Config", "Core data structure to represent a kinematic configuration.")

  .def(pybind11::init<>(), "initializes to an empty configuration, with no frames")

  .def("clear", [](shared_ptr<rai::Configuration>& self) {
    self->clear();
    checkView(self);
  }, "clear all frames and additional data; becomes the empty configuration, with no frames")

  .def("copy", [](shared_ptr<rai::Configuration>& self, shared_ptr<rai::Configuration>& C2) {
    self->copy(*C2);
    checkView(self);
  },
  "make C a (deep) copy of the given C2",
  pybind11::arg("C2")
      )

//-- setup/edit the configuration

  .def("addFile", [](shared_ptr<rai::Configuration>& self, const std::string& fileName) {
    self->addFile(fileName.c_str());
    checkView(self);
  },
  "add the contents of the file to C",
  pybind11::arg("file_name")
      )

  .def("addFrame", [](shared_ptr<rai::Configuration>& self, const std::string& name, const std::string& parent, const std::string& args) {
    rai::Frame* f = self->addFrame(name.c_str(), parent.c_str(), args.c_str());
    checkView(self);
    return shared_ptr<rai::Frame>(f, &null_deleter ); //giving it a non-sense deleter!
  },
  "add a new frame to C; optionally make this a child to the given parent; use the Frame methods to set properties of the new frame",
  pybind11::arg("name"),
  pybind11::arg("parent") = std::string(),
  pybind11::arg("args") = std::string()
      )

  .def("addObject", [](shared_ptr<rai::Configuration>& self, const std::string& name, const std::string& parent,
                       rai::ShapeType shape,
                       const std::vector<double>& size,
                       const std::vector<double>& color,
                       const std::vector<double>& pos,
  const std::vector<double>& quat) {
    rai::Frame *f = self->addFrame(name.c_str(), parent.c_str());
    if(f->parent) f->setJoint(rai::JT_rigid);
    f->setShape(shape, arr(size, true));
    f->setContact(-1);
    if(color.size()) f->setColor(arr(color, true));
    if(f->parent) {
      if(pos.size()) f->setRelativePosition(arr(pos,true));
      if(quat.size()) f->setRelativeQuaternion(arr(quat, true));
    } else {
      if(pos.size()) f->setPosition(arr(pos,true));
      if(quat.size()) f->setQuaternion(arr(quat, true));
    }

    checkView(self);
    return shared_ptr<rai::Frame>(f, &null_deleter ); //giving it a non-sense deleter!
  }, "TODO remove! use addFrame only",
  pybind11::arg("name"),
  pybind11::arg("parent") = std::string(),
  pybind11::arg("shape"),
  pybind11::arg("size") = std::vector<double>(),
  pybind11::arg("color") = std::vector<double>(),
  pybind11::arg("pos") = std::vector<double>(),
  pybind11::arg("quat") = std::vector<double>()
      )

  .def("addConfigurationCopy", [](shared_ptr<rai::Configuration>& self, shared_ptr<rai::Configuration>& other){
    self->addConfiguration(*other);
  }, "")

  .def("getFrame", [](shared_ptr<rai::Configuration>& self, const std::string& frameName) {
    rai::Frame *f = self->getFrame(frameName.c_str(), true);
    return shared_ptr<rai::Frame>(f, &null_deleter ); //giving it a non-sense deleter!
  },
  "get access to a frame by name; use the Frame methods to set/get frame properties",
  pybind11::arg("frameName")
      )

  .def("frame", [](shared_ptr<rai::Configuration>& self, const std::string& frameName) {
    rai::Frame *f = self->getFrame(frameName.c_str(), true);
    return shared_ptr<rai::Frame>(f, &null_deleter ); //giving it a non-sense deleter!
  },
  "get access to a frame by name; use the Frame methods to set/get frame properties",
  pybind11::arg("frameName")
      )

  .def("frames", [](shared_ptr<rai::Configuration>& self) {
    std::vector<shared_ptr<rai::Frame>> F;
    for(rai::Frame *f:self->frames) F.push_back(shared_ptr<rai::Frame>(f, &null_deleter)); //giving it a non-sense deleter!
    return F;
  } )

  .def("delFrame", [](shared_ptr<rai::Configuration>& self, const std::string& frameName) {
    rai::Frame* p = self->getFrame(frameName.c_str(), true);
    if(p) delete p;
    checkView(self);
  },
  "destroy and remove a frame from C",
  pybind11::arg("frameName")
      )

  .def("getJointNames", [](shared_ptr<rai::Configuration>& self) {
    return I_conv(self->getJointNames());
  },
  "get the list of joint names"
      )

  .def("getJointDimension", [](shared_ptr<rai::Configuration>& self) {
    return self->getJointStateDimension();
  },
  "get the total number of degrees of freedom"
      )

  .def("getJointState", [](shared_ptr<rai::Configuration>& self, const uintA& joints) {
    arr q;
    if(joints.N) q = self->getJointState(joints);
    else q = self->getJointState();
    return q;
//    return pybind11::array(q.dim(), q.p);
  },
  "get the joint state as a numpy vector, optionally only for a subset of joints specified as list of joint names",
  pybind11::arg("joints") = ry::I_StringA()
      )

  .def("setJointState", [](shared_ptr<rai::Configuration>& self, const std::vector<double>& q, const uintA& joints) {
    if(joints.N) {
      self->setJointState(arr(q, true), joints);
    } else {
      self->setJointState(arr(q, true));
    }
    checkView(self);
  },
  "set the joint state, optionally only for a subset of joints specified as list of frameIDs",
  pybind11::arg("q"),
  pybind11::arg("joints") = ry::I_StringA()
      )

      .def("setJointState", [](shared_ptr<rai::Configuration>& self, const arr& q, const ry::I_StringA& joints) {
        self->setJointState(q, self->getFrames(I_conv(joints)));
        checkView(self);
      },
      "set the joint state, optionally only for a subset of joints specified as list of joint names",
  pybind11::arg("q"),
  pybind11::arg("joints")
  )

  .def("setJointStateSlice", [](shared_ptr<rai::Configuration>& self, const std::vector<double>& q, uint t) {
    self->setJointStateSlice(arr(q, true), t);
    checkView(self);
  }, "")

  .def("getFrameNames", [](shared_ptr<rai::Configuration>& self) {
    return I_conv(self->getFrameNames());
  },
  "get the list of frame names"
      )

  .def("getFrameDimension", [](shared_ptr<rai::Configuration>& self) {
    return self->frames.N;
  },
  "get the total number of frames"
      )

  .def("getFrameState", [](shared_ptr<rai::Configuration>& self) {
    arr X = self->getFrameState();
    return pybind11::array(X.dim(), X.p);
  },
  "get the frame state as a n-times-7 numpy matrix, with a 7D pose per frame"
      )

  .def("getFrameState", [](shared_ptr<rai::Configuration>& self, const char* frame) {
    arr X;
    rai::Frame* f = self->getFrame(frame, true);
    if(f) X = f->ensure_X().getArr7d();
    return pybind11::array(X.dim(), X.p);
  }, "TODO remove -> use individual frame!")

  .def("setFrameState", [](shared_ptr<rai::Configuration>& self, const std::vector<double>& X, const ry::I_StringA& frames) {
    arr _X (X, true);
    _X.reshape(_X.N/7, 7);
    if(frames.size()){
      self->setFrameState(_X, self->getFrames(I_conv(frames)));
    }else{
      self->setFrameState(_X);
    }
    checkView(self);
  },
  "set the frame state, optionally only for a subset of frames specified as list of frame names",
  pybind11::arg("X"),
  pybind11::arg("frames") = ry::I_StringA()
      )

  .def("setFrameState", [](shared_ptr<rai::Configuration>& self, const pybind11::array& X, const ry::I_StringA& frames) {
    arr _X = numpy2arr<double>(X);
    _X.reshape(_X.N/7, 7);
    if(frames.size()){
      self->setFrameState(_X, self->getFrames(I_conv(frames)));
    }else{
      self->setFrameState(_X);
    }
    checkView(self);
  },
  "set the frame state, optionally only for a subset of frames specified as list of frame names",
  pybind11::arg("X"),
  pybind11::arg("frames") = ry::I_StringA()
      )

  .def("feature", [](shared_ptr<rai::Configuration>& self, FeatureSymbol featureSymbol, const ry::I_StringA& frameNames, const std::vector<double>& scale, const std::vector<double>& target, int order) {
    return symbols2feature(featureSymbol, I_conv(frameNames), *self, arr(scale, true), arr(target, true), order);
  },
  "create a feature (a differentiable map from joint state to a vector space), as they're typically used for IK or optimization. See the dedicated tutorial for details. \
featureSymbol defines which mapping this is (position, vectors, collision distance, etc). \
many mapping refer to one or several frames, which need to be specified using frameNames",
  pybind11::arg("featureSymbol"),
  pybind11::arg("frameNames")=ry::I_StringA(),
    pybind11::arg("scale")=std::vector<double>(),
    pybind11::arg("target")=std::vector<double>(),
    pybind11::arg("order")=0
    )

  .def("evalFeature", [](shared_ptr<rai::Configuration>& self, FeatureSymbol fs, const ry::I_StringA& frames) {
    arr y, J;
    self->evalFeature(y, J, fs, I_conv(frames));
    return pybind11::make_tuple(pybind11::array(y.dim(), y.p), pybind11::array(J.dim(), J.p));
  }, "TODO remove -> use feature directly"
      )

  .def("selectJoints", [](shared_ptr<rai::Configuration>& self, const ry::I_StringA& jointNames, bool notThose) {
    // TODO: this is joint groups
    // TODO: maybe call joint groups just joints and joints DOFs
    self->selectJointsByName(I_conv(jointNames), notThose);
  },
  "redefine what are considered the DOFs of this configuration: only joints listed in jointNames are considered \
part of the joint state and define the number of DOFs",
  pybind11::arg("jointNames"),
  pybind11::arg("notThose") = false
      )

  .def("selectJointsByTag", [](shared_ptr<rai::Configuration>& self, const ry::I_StringA& jointGroups) {
    self->selectJointsByGroup(I_conv(jointGroups));
    self->ensure_q();
  },
  "redefine what are considered the DOFs of this configuration: only joint that have a tag listed in jointGroups are considered \
part of the joint state and define the number of DOFs",
  pybind11::arg("jointGroups")
      )

  .def("makeObjectsFree", [](shared_ptr<rai::Configuration>& self, const ry::I_StringA& objs) {
    self->makeObjectsFree(I_conv(objs));
    checkView(self);
  }, "TODO remove -> to frame")

  .def("makeObjectsConvex", [](shared_ptr<rai::Configuration>& self) {
    makeConvexHulls(self->frames);
    checkView(self);
  },
  "remake all meshes associated with all frames to become their convex hull"
      )

  .def("attach", [](shared_ptr<rai::Configuration>& self, const std::string& frame1, const std::string& frame2) {
    self->attach(frame1.c_str(), frame2.c_str());
    checkView(self);
  },
  "change the configuration by creating a rigid joint from frame1 to frame2, adopting their current \
relative pose. This also breaks the first joint that is parental to frame2 and reverses the \
topological order from frame2 to the broken joint"
      )

  .def("computeCollisions", [](shared_ptr<rai::Configuration>& self) {
    self->stepSwift();
    checkView(self);
  },
  "call the broadphase collision engine (SWIFT++ or FCL) to generate the list of collisions (or near proximities) \
between all frame shapes that have the collision tag set non-zero"
      )

  .def("getCollisions", [](shared_ptr<rai::Configuration>& self, double belowMargin) {
    pybind11::list ret;
    for(const rai::Proxy& p: self->proxies) {
      if(!p.collision)((rai::Proxy*)&p)->calc_coll();
      if(p.d>belowMargin) continue;
      pybind11::tuple tuple(3);
      tuple[0] = p.a->name.p;
      tuple[1] = p.b->name.p;
      tuple[2] = p.d;
//      tuple[3] = p.posA;
//      tuple[4] = p.posB;
      ret.append(tuple) ;
    }
    return ret;
  },
  "return the results of collision computations: a list of 3 tuples with (frame1, frame2, distance). \
Optionally report only on distances below a margin \
To get really precise distances and penetrations use the FS.distance feature with the two frame names",
  pybind11::arg("belowMargin") = 1.
      )

  .def("view", [](shared_ptr<rai::Configuration>& self) {
    self->gl()->setConfiguration(*self);
  },
  "create a viewer for this configuration. Optionally, specify a frame that is the origin of the viewer camera")

  .def("view_recopyMeshes", [](shared_ptr<rai::Configuration>& self) {
    self->gl()->recopyMeshes(*self);
  })

  .def("view_playVideo", [](shared_ptr<rai::Configuration>& self, double delay, const char* saveVideoPath) {
    self->gl()->playVideo(false, delay, saveVideoPath);
  }, "",
  pybind11::arg("delay")=double(1.),
  pybind11::arg("saveVideoPath")=nullptr
  )

  .def("view_getScreenshot", [](shared_ptr<rai::Configuration>& self) {
    byteA rgb = self->gl()->getScreenshot();
   return pybind11::array_t<byte>(rgb.dim(), rgb.p);
  })

  .def("view_close", [](shared_ptr<rai::Configuration>& self) {
    self->gl().reset();
  }, "close the view")

  .def("cameraView", [](shared_ptr<rai::Configuration>& self) {
    ry::RyCameraView view;
    view.cam = make_shared<rai::CameraView>(*self, true, 0);
    return view;
  },
  "create an offscreen renderer for this configuration"
      )

  .def("edit", [](shared_ptr<rai::Configuration>& self, const char* fileName) {
    rai::Configuration K;
    editConfiguration(fileName, K);
    self->copy(K);
    checkView(self);
  },
  "launch a viewer that listents (inode) to changes of a file (made by you in an editor), and \
reloads, displays and animates the configuration whenever the file is changed"
      )

  .def("komo_IK", [](shared_ptr<rai::Configuration>& self, bool useSwift) {
//  ry::RyKOMO komo;
    auto komo = make_shared<KOMO>();
    komo->setModel(*self, useSwift);
    komo->setIKOpt();
    return komo;
  },
  "create KOMO solver configured to IK, useSwift determine whether for each \
query the broadphase collision computations are done. (Necessary only when generic \
FS.accumulatedCollision feature is needed. The explicit distance feature is independent \
from broadphase collision computation)",
  pybind11::arg("useSwift")
      )

  .def("komo_CGO", [](shared_ptr<rai::Configuration>& self, uint numConfigs, bool useSwift) {
    CHECK_GE(numConfigs, 1, "");
    auto komo = make_shared<KOMO>();
    komo->setModel(*self, useSwift);
    komo->setDiscreteOpt(numConfigs);
    return komo;
  },
  "create KOMO solver configured for dense graph optimization, \
numConfig gives the number of configurations optimized over, \
useSwift determine whether for each \
query the broadphase collision computations are done. (Necessary only when generic \
FS.accumulatedCollision feature is needed. The explicit distance feature is independent \
from broadphase collision computation)",
  pybind11::arg("numConfigs"),
  pybind11::arg("useSwift")
      )

  .def("komo_path",  [](shared_ptr<rai::Configuration>& self, double phases, uint stepsPerPhase, double timePerPhase, bool useSwift) {
    auto komo = make_shared<KOMO>();
    komo->setModel(*self, useSwift);
    komo->setTiming(phases, stepsPerPhase, timePerPhase);
    komo->add_qControlObjective({}, 2, 1.);
    return komo;
  },
  "create KOMO solver configured for sparse path optimization",
  pybind11::arg("phases"),
  pybind11::arg("stepsPerPhase")=20,
  pybind11::arg("timePerPhase")=5.,
  pybind11::arg("useSwift")
      )

  .def("komo",  [](shared_ptr<rai::Configuration>& self, double phases, uint stepsPerPhase, double timePerPhase, uint k_order, bool useSwift) {
    auto komo = make_shared<KOMO>();
    komo->setModel(*self, useSwift);
    komo->setTiming(phases, stepsPerPhase, timePerPhase, k_order);
    return komo;
  },
  "create KOMO solver configured for sparse path optimization without control objective",
  pybind11::arg("phases"),
  pybind11::arg("stepsPerPhase")=20,
  pybind11::arg("timePerPhase")=5.,
  pybind11::arg("k_order")=2,
  pybind11::arg("useSwift")
      )

  /*
  .def("lgp", [](shared_ptr<rai::Configuration>& self, const std::string& folFileName) {
    ry::RyLGP_Tree lgp;
    lgp.lgp = make_shared<LGP_Tree_Thread>(*self, folFileName.c_str());
    return lgp;
  },
  "create an LGP solver"
      )

  .def("bullet", [](shared_ptr<rai::Configuration>& self) {
    return make_shared<BulletInterface>(*self);
  },
  "create a Bullet engine for physical simulation from the configuration: The configuration \
  is being exported into a bullet instance, which can be stepped forward, and the result syced back to this configuration"
      )

  .def("physx", [](shared_ptr<rai::Configuration>& self) {
    return make_shared<PhysXInterface>(*self);
  },
  "create a PhysX engine for physical simulation from the configuration: The configuration \
  is being exported into a bullet instance, which can be stepped forward, and the result syced back to this configuration"
      )
  */

  .def("simulation", [](shared_ptr<rai::Configuration>& self, rai::Simulation::SimulatorEngine engine, int verbose) {
    return make_shared<rai::Simulation>(*self, engine, verbose);
  },
  "create a generic Simulation engine, which can internally call PhysX, Bullet, or just kinematics to forward simulate, \
allows you to control robot motors by position, velocity, or accelerations, \
    and allows you go query camera images and depth",
  pybind11::arg("engine"),
  pybind11::arg("verbose")
      )

//.def("operate", [](shared_ptr<rai::Configuration>& self, const char* rosNodeName) {
//  ry::RyOperate op;
//  op.R = make_shared<RobotOperation>(*self, .01, rosNodeName);
//  return op;
//},
//"create a module (including ROS node) to sync this configuration both ways (reading state, and controlling) to a real robot",
//pybind11::arg("rosNodeName")
//    )

  .def("sortFrames", [](shared_ptr<rai::Configuration>& self) {
    self->sortFrames();
    checkView(self);
  }, "resort the internal order of frames according to the tree topology. This is important before saving the configuration.")

  .def("equationOfMotion", [](shared_ptr<rai::Configuration>& self, std::vector<double>& qdot, bool gravity) {
    arr M, F;
    self->equationOfMotion(M, F, arr(qdot, true), gravity);
    return pybind11::make_tuple(pybind11::array(M.dim(), M.p), pybind11::array(F.dim(), F.p));
  }, "",
  pybind11::arg("qdot"),
  pybind11::arg("gravity"))

  .def("stepDynamics", [](shared_ptr<rai::Configuration>& self, std::vector<double>& qdot, std::vector<double>& u_control, double tau, double dynamicNoise, bool gravity) {
    arr _qdot(qdot, false);
    self->stepDynamics(_qdot, arr(u_control, true), tau, dynamicNoise, gravity);
    checkView(self);
    return pybind11::array(_qdot.dim(), _qdot.p);
  }, "",
  pybind11::arg("qdot"),
  pybind11::arg("u_control"),
  pybind11::arg("tau"),
  pybind11::arg("dynamicNoise"),
  pybind11::arg("gravity"))

  ;

//===========================================================================

//  pybind11::class_<ry::ConfigViewer>(m, "ConfigViewer");
  pybind11::class_<ImageViewerCallback, shared_ptr<ImageViewerCallback>>(m, "ImageViewer");
  pybind11::class_<PointCloudViewerCallback, shared_ptr<PointCloudViewerCallback>>(m, "PointCloudViewer");


//===========================================================================

  pybind11::class_<ry::RyCameraView>(m, "CameraView")
  .def("updateConfig", [](ry::RyCameraView& self, shared_ptr<rai::Configuration>& config) {
    if(config->frames.N!= self.cam->C.frames.N) {
      self.cam->C.copy(*config);
    } else {
      self.cam->C.setFrameState(config->getFrameState());
    }
  })

  .def("addSensor", [](ry::RyCameraView& self, const char* name, const char* frameAttached, uint width, uint height, double focalLength, double orthoAbsHeight, const std::vector<double>& zRange, const std::string& backgroundImageFile) {
    self.cam->addSensor(name, frameAttached, width, height, focalLength, orthoAbsHeight, arr(zRange, true), backgroundImageFile.c_str());
  }, "",
  pybind11::arg("name"),
  pybind11::arg("frameAttached"),
  pybind11::arg("width"),
  pybind11::arg("height"),
  pybind11::arg("focalLength") = -1.,
  pybind11::arg("orthoAbsHeight") = -1.,
  pybind11::arg("zRange") = std::vector<double>(),
  pybind11::arg("backgroundImageFile") = std::string())

  .def("selectSensor", [](ry::RyCameraView& self, const char* sensorName) {
    self.cam->selectSensor(sensorName);
  }, "",
  pybind11::arg("name"))

  .def("computeImageAndDepth", [](ry::RyCameraView& self, bool visualsOnly) {
    auto imageSet = self.image.set();
    auto depthSet = self.depth.set();
    if(visualsOnly) self.cam->renderMode = rai::CameraView::visuals;
    else self.cam->renderMode = rai::CameraView::all;
    self.cam->computeImageAndDepth(imageSet, depthSet);
    pybind11::tuple ret(2);
    ret[0] = pybind11::array(imageSet->dim(), imageSet->p);
    ret[1] = pybind11::array(depthSet->dim(), depthSet->p);
    return ret;
  },
  pybind11::arg("visualsOnly")=true
      )

  .def("computePointCloud", [](ry::RyCameraView& self, const pybind11::array& depth, bool globalCoordinates) {
    arr _depth = numpy2arr<double>(depth);
    floatA __depth; copy(__depth, _depth);
    auto ptsSet = self.pts.set();
    self.cam->computePointCloud(ptsSet, __depth, globalCoordinates);
    return pybind11::array(ptsSet->dim(), ptsSet->p);
  }, "",
  pybind11::arg("depth"),
  pybind11::arg("globalCoordinates") = true)

  .def("computeSegmentation", [](ry::RyCameraView& self) {
    auto segSet = self.segmentation.set();
    self.cam->computeSegmentation(segSet);
    return pybind11::array(segSet->dim(), segSet->p);
  })

  .def("pointCloudViewer", [](ry::RyCameraView& self) {
    return make_shared<PointCloudViewerCallback>(self.pts, self.image);
  })

  .def("imageViewer", [](ry::RyCameraView& self) {
    return make_shared<ImageViewerCallback>(self.image);
  })

  .def("segmentationViewer", [](ry::RyCameraView& self) {
    return make_shared<ImageViewerCallback>(self.segmentation);
  })
  ;

#define ENUMVAL(pre, x) .value(#x, pre##_##x)

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
  .export_values();

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
  ENUMVAL(FS, oppose)

  ENUMVAL(FS, qItself)

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
  .export_values();

#undef ENUMVAL
#define ENUMVAL(x) .value(#x, rai::Simulation::_##x)

  pybind11::enum_<rai::Simulation::SimulatorEngine>(m, "SimulatorEngine")
  ENUMVAL(physx)
  ENUMVAL(bullet)
  ENUMVAL(kinematic)
  .export_values();

  pybind11::enum_<rai::Simulation::ControlMode>(m, "ControlMode")
  ENUMVAL(none)
  ENUMVAL(position)
  ENUMVAL(velocity)
  ENUMVAL(acceleration)
  .export_values();

  pybind11::enum_<rai::Simulation::ImpType>(m, "ImpType")
  ENUMVAL(closeGripper)
  ENUMVAL(openGripper)
  ENUMVAL(depthNoise)
  ENUMVAL(rgbNoise)
  ENUMVAL(adversarialDropper)
  ENUMVAL(objectImpulses)
  .export_values();

}

#endif
