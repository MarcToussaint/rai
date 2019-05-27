#ifdef RAI_PYBIND

#include "ry.h"

#include <Core/graph.h>
#include <Kin/frame.h>
#include <Kin/kin.h>
#include <Kin/kin_bullet.h>
#include <Kin/kin_physx.h>
#include <Perception/depth2PointCloud.h>

#include <pybind11/pybind11.h>
#include <pybind11/stl.h>
#include <pybind11/numpy.h>

namespace py = pybind11;

py::dict graph2dict(const Graph& G){
  py::dict dict;
  for(Node *n:G){
    rai::String key;
    if(n->keys.N) key=n->keys.last();
    else key <<n->index;

    //-- write value
    if(n->isGraph()) {
      dict[key.p] = graph2dict(n->get<Graph>());
    } else if(n->isOfType<rai::String>()) {
      dict[key.p] = n->get<rai::String>().p;
    } else if(n->isOfType<arr>()) {
      dict[key.p] = conv_arr2stdvec( n->get<arr>() );
    } else if(n->isOfType<intA>()) {
      dict[key.p] = conv_arr2stdvec( n->get<intA>() );
    } else if(n->isOfType<uintA>()) {
      dict[key.p] = conv_arr2stdvec( n->get<uintA>() );
    } else if(n->isOfType<boolA>()) {
      dict[key.p] = conv_arr2stdvec( n->get<boolA>() );
    } else if(n->isOfType<double>()) {
      dict[key.p] = n->get<double>();
    } else if(n->isOfType<int>()) {
      dict[key.p] = n->get<int>();
    } else if(n->isOfType<uint>()) {
      dict[key.p] = n->get<uint>();
    } else if(n->isOfType<bool>()) {
      dict[key.p] = n->get<bool>();
    } else if(n->isOfType<rai::Enum<rai::ShapeType>>()) {
      dict[key.p] = n->get<rai::Enum<rai::ShapeType>>().name();
    } else {
      LOG(-1) <<"can't convert node of type " <<n->type.name() <<" to dictionary";
    }
  }
  return dict;
}

py::list graph2list(const Graph& G){
  py::list list;
  for(Node *n:G){
    //-- write value
    if(n->isGraph()) {
      list.append( graph2dict(n->get<Graph>()) );
    } else if(n->isOfType<rai::String>()) {
      list.append( n->get<rai::String>().p );
    } else if(n->isOfType<arr>()) {
      list.append( conv_arr2stdvec( n->get<arr>() ) );
    } else if(n->isOfType<double>()) {
      list.append( n->get<double>() );
    } else if(n->isOfType<int>()) {
      list.append( n->get<int>() );
    } else if(n->isOfType<uint>()) {
      list.append( n->get<uint>() );
    } else if(n->isOfType<bool>()) {
      list.append( n->get<bool>() );
    } else {
    }

  }
  return list;
}

Skeleton list2skeleton(const py::list& L){
  Skeleton S;
  for(uint i=0;i<L.size();i+=3){
    std::vector<double> when = L[i].cast<std::vector<double> >();
    SkeletonSymbol symbol = L[i+1].cast<SkeletonSymbol>();
    ry::I_StringA frames = L[i+2].cast<ry::I_StringA>();
    S.append(SkeletonEntry(when[0], when[1], symbol, I_conv(frames)));
  }
  return S;
}

py::tuple uintA2tuple(const uintA& tup){
  py::tuple tuple;
  for(uint i=0;i<tup.N;i++) tuple[i] = tup(i);
  return tuple;
}

arr numpy2arr(const pybind11::array& X){
  arr Y;
  uintA dim(X.ndim());
  for(uint i=0;i<dim.N;i++) dim(i)=X.shape()[i];
  Y.resize(dim);
  auto ref = X.unchecked<double>();
  if (Y.nd==1) {
    for(uint i=0;i<Y.d0;i++) Y(i) = ref(i);
    return Y;
  } else if(Y.nd==2){
    for(uint i=0;i<Y.d0;i++) for(uint j=0;j<Y.d1;j++) Y(i,j) = ref(i,j);
    return Y;
  } else if(Y.nd==3){
    for(uint i=0;i<Y.d0;i++) for(uint j=0;j<Y.d1;j++) for(uint k=0;k<Y.d2;k++) Y(i,j,k) = ref(i,j,k);
    return Y;
  }
  NIY;
  return Y;
}

byteA numpy2arr(const pybind11::array_t<byte>& X){
  byteA Y;
  uintA dim(X.ndim());
  for(uint i=0;i<dim.N;i++) dim(i)=X.shape()[i];
  Y.resize(dim);
  auto ref = X.unchecked<>();
  if (Y.nd==1) {
    for(uint i=0;i<Y.d0;i++) Y(i) = ref(i);
    return Y;
  } else if(Y.nd==2){
    for(uint i=0;i<Y.d0;i++) for(uint j=0;j<Y.d1;j++) Y(i,j) = ref(i,j);
    return Y;
  } else if(Y.nd==3){
    for(uint i=0;i<Y.d0;i++) for(uint j=0;j<Y.d1;j++) for(uint k=0;k<Y.d2;k++) Y(i,j,k) = ref(i,j,k);
    return Y;
  }
  NIY;
  return Y;
}

arr vecvec2arr(const std::vector<std::vector<double>>& X){
  CHECK(X.size()>0, "");
  arr Y(X.size(), X[0].size());
  for(uint i=0;i<Y.d0;i++) for(uint j=0;j<Y.d1;j++) Y(i,j) = X[i][j];
  return Y;
}

#define METHOD_set(method) .def(#method, [](ry::Config& self) { self.set()->method(); } )
#define METHOD_set1(method, arg1) .def(#method, [](ry::Config& self) { self.set()->method(arg1); } )


PYBIND11_MODULE(libry, m) {
  m.doc() = "rai bindings";

  //===========================================================================
  //
  // Config
  //

  py::class_<ry::Config>(m, "Config", "This is a class docstring")
      .def(py::init<>())

  METHOD_set(clear)

  .def("copy", [](ry::Config& self, ry::Config& C2) {
    self.set()->copy(C2.get());
  },
  "make C a (deep) copy of the given C2",
  py::arg("C2")
  )

  //-- setup/edit the configuration

  .def("addFile", [](ry::Config& self, const std::string& fileName) {
    self.set()->addFile(fileName.c_str());
  },
  "add the contents of the file to C",
  py::arg("file_name")
  )

  .def("addFrame", [](ry::Config& self, const std::string& name, const std::string& parent, const std::string& args) {
    ry::RyFrame f;
    f.config = self.data;
    f.frame = self.set()->addFrame(name.c_str(), parent.c_str(), args.c_str());
    return f;
  },
  "add a new frame to C; optionally make this a child to the given parent",
  py::arg("name"),
      py::arg("parent") = std::string(),
      py::arg("args") = std::string()
  )

  .def("frame", [](ry::Config& self, const std::string& frameName){
    ry::RyFrame f;
    f.config = self.data;
    f.frame = self.get()->getFrameByName(frameName.c_str(), true);
    return f;
  },
  "get access to a frame by name",
  py::arg("frameName")
  )

  .def("setFrameRelativePose", [](ry::Config& self, const std::string& frame, const std::vector<double>& x) {
      auto Kset = self.set();
      rai::Frame *f = Kset->getFrameByName(frame.c_str(), true);
      f->Q.set(conv_stdvec2arr(x));
      Kset->calc_fwdPropagateFrames();
  }, "TODO remove -> use frame" )

  .def("delFrame", [](ry::Config& self, const std::string& frameName) {
    auto Kset = self.set();
    rai::Frame *p = Kset->getFrameByName(frameName.c_str(), true);
    if(p) delete p;
  },
  "destroy and remove a frame from C",
  py::arg("frameName")
  )

  .def("addObject", [](ry::Config& self, const std::string& name, const std::string& parent,
       rai::ShapeType shape,
       const std::vector<double>& size,
       const std::vector<double>& color,
       const std::vector<double>& pos,
       const std::vector<double>& quat){
    auto Kset = self.set();
    ry::RyFrame f;
    f.config = self.data;
    f.frame = Kset->addObject(name.c_str(), parent.c_str(), shape, conv_stdvec2arr(size), conv_stdvec2arr(color), conv_stdvec2arr(pos), conv_stdvec2arr(quat));
//    f->name = name;
//    if(parent.size()){
//      rai::Frame *p = Kset->getFrameByName(parent.c_str());
//      if(p) f->linkFrom(p);
//    }
//    if(pos.size()) f->Q.pos.set(pos);
//    if(quat.size()) f->Q.rot.set(quat);
//    if(rot.size()) f->Q.addRelativeRotationDeg(rot[0], rot[1], rot[2], rot[3]);
//    if(f->parent){
//      f->X = f->parent->X * f->Q;
//    }else{
//      f->X = f->Q;
//    }
    return f;
  }, "TODO remove! use addFrame only",
    py::arg("name"),
    py::arg("parent") = std::string(),
    py::arg("shape"),
    py::arg("size") = std::vector<double>(),
    py::arg("color") = std::vector<double>(),
    py::arg("pos") = std::vector<double>(),
    py::arg("quat") = std::vector<double>()
  )


  .def("getJointNames", [](ry::Config& self) {
    return I_conv(self.get()->getJointNames());
  },
    "get the list of joint names"
  )

  .def("getJointDimension", [](ry::Config& self) {
    return self.get()->getJointStateDimension();
  },
    "get the total number of degrees of freedom"
  )

  .def("getJointState", [](ry::Config& self, const ry::I_StringA& joints) {
    arr q;
    if(joints.size()) q = self.get()->getJointState(I_conv(joints));
    else q = self.get()->getJointState();
    return pybind11::array(q.dim(), q.p);
  },
    "get the joint state as a numpy vector, optionally only for a subset of joints specified as list of joint names",
    py::arg("joints") = ry::I_StringA()
  )

  .def("setJointState", [](ry::Config& self, const std::vector<double>& q, const ry::I_StringA& joints){
    arr _q = conv_stdvec2arr(q);
    if(joints.size()){
      self.set()->setJointState(_q, I_conv(joints));
    }else{
      self.set()->setJointState(_q);
    }
  },
    "set the joint state, optionally only for a subset of joints specified as list of joint names",
    py::arg("q"),
    py::arg("joints") = ry::I_StringA()
  )

  .def("getFrameNames", [](ry::Config& self){
    return I_conv(self.get()->getFrameNames());
  },
    "get the list of frame names"
  )

  .def("getJointDimension", [](ry::Config& self) {
    return self.get()->frames.N;
  },
    "get the total number of frames"
  )

  .def("getFrameState", [](ry::Config& self){
    arr X = self.get()->getFrameState();
    return pybind11::array(X.dim(), X.p);
  },
    "get the frame state as a n-times-7 numpy matrix, with a 7D pose per frame"
  )

  .def("getFrameState", [](ry::Config& self, const char* frame){
    arr X;
    auto Kget = self.get();
    rai::Frame *f = Kget->getFrameByName(frame, true);
    if(f) X = f->X.getArr7d();
    return pybind11::array(X.dim(), X.p);
  }, "TODO remove -> use individual frame!" )

  .def("setFrameState", [](ry::Config& self, const std::vector<double>& X, const ry::I_StringA& frames, bool calc_q_from_X){
    arr _X = conv_stdvec2arr(X);
    _X.reshape(_X.N/7, 7);
    self.set()->setFrameState(_X, I_conv(frames), calc_q_from_X);
  },
    "set the frame state, optionally only for a subset of frames specified as list of frame names. \
 By default this also computes and sets the consistent joint state based on the relative poses.\
 Setting calc_q_from_x to false will not compute the joint state and leave the configuration in an inconsistent state!",
    py::arg("X"),
    py::arg("frames") = ry::I_StringA(),
    py::arg("calc_q_from_X") = true
  )

  .def("setFrameState", [](ry::Config& self, const pybind11::array& X, const ry::I_StringA& frames, bool calc_q_from_X){
    arr _X = numpy2arr(X);
    _X.reshape(_X.N/7, 7);
    self.set()->setFrameState(_X, I_conv(frames), calc_q_from_X);
 },
 "set the frame state, optionally only for a subset of frames specified as list of frame names. \
By default this also computes and sets the consistent joint state based on the relative poses.\
Setting calc_q_from_x to false will not compute the joint state and leave the configuration in an inconsistent state!",
    py::arg("X"),
    py::arg("frames") = ry::I_StringA(),
    py::arg("calc_q_from_X") = true
  )

  .def("feature", [](ry::Config& self, FeatureSymbol featureSymbol, const ry::I_StringA& frameNames) {
    ry::RyFeature F;
    F.feature = symbols2feature(featureSymbol, I_conv(frameNames), self.get());
    return F;
  },
  "create a feature (a differentiable map from joint state to a vector space), as they're typically used for IK or optimization. See the dedicated tutorial for details.\
featureSymbol defines which mapping this is (position, vectors, collision distance, etc).\
many mapping refer to one or several frames, which need to be specified using frameNames",
py::arg("featureSymbol"),
    py::arg("frameNames"))

  .def("evalFeature", [](ry::Config& self, FeatureSymbol fs, const ry::I_StringA& frames) {
    arr y,J;
    self.get()->evalFeature(y, J, fs, I_conv(frames));
    return pybind11::make_tuple(pybind11::array(y.dim(), y.p), pybind11::array(J.dim(), J.p));
  }, "TODO remove -> use feature directly"
  )

  .def("selectJoints", [](ry::Config& self, const ry::I_StringA& jointNames){
    // TODO: this is joint groups
    // TODO: maybe call joint groups just joints and joints DOFs
    self.set()->selectJointsByName(I_conv(jointNames));
  },
  "redefine what are considered the DOFs of this configuration: only joint listed in jointNames are considered\
  part of the joint state and define the number of DOFs",
    py::arg("jointNames")
  )

  .def("selectJointsByTag", [](ry::Config& self, const ry::I_StringA& jointGroups){
    auto Kset = self.set();
    Kset->selectJointsByGroup(I_conv(jointGroups), true, true);
    Kset->calc_q();
  },
  "redefine what are considered the DOFs of this configuration: only joint that have a tag listed in jointGroups are considered\
  part of the joint state and define the number of DOFs",
    py::arg("jointGroups")
  )

  .def("makeObjectsFree", [](ry::Config& self, const ry::I_StringA& objs){
    self.set()->makeObjectsFree(I_conv(objs));
  }, "TODO remove -> to frame" )

  .def("makeObjectsConvex", [](ry::Config& self){
      makeConvexHulls(self.set()->frames);
  },
  "remake all meshes associated with all frames to become their convex hull"
  )

  .def("attach", [](ry::Config& self, const std::string& frame1, const std::string& frame2){
      auto Kset = self.set();
      Kset->attach(frame1.c_str(), frame2.c_str());
  },
  "change the configuration by creating a rigid joint from frame1 to frame2, adopting their current\
  relative pose. This also breaks the first joint joints that is parental to frame2 and reverses the\
  topological order from frame2 to the broken joint"
  )

  .def("computeCollisions", [](ry::Config& self){
    self.set()->stepSwift();
  },
  "call the broadphase collision engine (SWIFT++) to generate the list of collisions (or near proximities)\
  between all frame shapes that have the collision tag set non-zero"
  )

  .def("getCollisions", [](ry::Config& self, double belowMargin){
    py::list ret;
    auto Kget = self.get();
    for(const rai::Proxy& p: Kget->proxies) {
      if(!p.coll)((rai::Proxy*)&p)->calc_coll(Kget);
      if(p.d>belowMargin) continue;
      pybind11::tuple tuple(3);
      tuple[0] = p.a->name.p;
      tuple[1] = p.b->name.p;
      tuple[2] = p.d;
//      tuple[3] = p.posA;
//      tuple[4] = p.posB;
      ret.append( tuple ) ;
    }
    return ret;
  },
  "return the results of collision computations: a list of 3 tuples with (frame1, frame2, distance).\
  Optionally report only on distances below a margin\
  To get really precise distances and penetrations use the FS.distance feature with the two frame names",
  py::arg("belowMargin") = 1.
  )

  .def("getFrameBox", [](ry::Config& self, const std::string& framename){
    auto Kget = self.get();
    rai::Frame *f = Kget->getFrameByName(framename.c_str(), true);
    rai::Shape *s = f->shape;
    CHECK(s, "frame " <<f->name <<" does not have a shape");
    CHECK(s->type() == rai::ST_ssBox || s->type() == rai::ST_box,
            "frame " <<f->name <<" needs to be a box");
    arr range = s->size();
    return pybind11::array(range.dim(), range.p);
  }, "TODO remove -> frame.getShape" )

  .def("view", [](ry::Config& self, const std::string& frame){
    ry::ConfigViewer view;
    view.view = make_shared<KinViewer>(self, -1, rai::String(frame));
    return view;
  },
  "create a viewer for this configuration. Optionally, specify a frame that is the origin of the viewer camera",
  py::arg("frame")="")

  .def("cameraView", [](ry::Config& self){
    ry::RyCameraView view;
    view.cam = make_shared<rai::CameraView>(self.get(), true, 0);
    return view;
   },
  "create an offscreen renderer for this configuration"
  )

  .def("edit", [](ry::Config& self, const char* fileName){
    rai::KinematicWorld K;
    editConfiguration(fileName, K);
    self.set() = K;
  },
  "launch a viewer that listents (inode) to changes of a file (made by you in an editor), and\
  reloads, displays and animates the configuration whenever the file is changed"
  )

  .def("komo_IK", [](ry::Config& self, bool useSwift){
    ry::RyKOMO komo;
    komo.komo = make_shared<KOMO>(self.get(), useSwift);
    komo.config.set() = komo.komo->world;
    komo.komo->setIKOpt();
    return komo;
  },
  "create KOMO solver configured to IK, useSwift determine whether for each\
  query the broadphase collision computations are done. (Necessary only when generic\
  FS.accumulatedCollision feature is needed. The explicit distance feature is independent\
  from broadphase collision computation)",
  py::arg("useSwift")
  )

  .def("komo_CGO", [](ry::Config& self, uint numConfigs, bool useSwift){
    CHECK_GE(numConfigs, 1, "");
    ry::RyKOMO komo;
    komo.komo = make_shared<KOMO>(self.get(), useSwift);
    komo.config.set() = komo.komo->world;
    komo.komo->setDiscreteOpt(numConfigs);
    return komo;
  },
  "create KOMO solver configured for dense graph optimization,\
  numConfig gives the number of configurations optimized over,\
  useSwift determine whether for each\
  query the broadphase collision computations are done. (Necessary only when generic\
  FS.accumulatedCollision feature is needed. The explicit distance feature is independent\
  from broadphase collision computation)",
  py::arg("numConfigs"),
  py::arg("useSwift")
  )

  .def("komo_path",  [](ry::Config& self, double phases, uint stepsPerPhase, double timePerPhase, bool useSwift){
    ry::RyKOMO komo;
    komo.komo = make_shared<KOMO>(self.get(), useSwift);
    komo.config.set() = komo.komo->world;
    komo.komo->setPathOpt(phases, stepsPerPhase, timePerPhase);
    return komo;
  },
  "create KOMO solver configured for sparse path optimization",
  py::arg("phases"),
  py::arg("stepsPerPhase")=20,
  py::arg("timePerPhase")=5.,
  py::arg("useSwift")
  )

  .def("lgp", [](ry::Config& self, const std::string& folFileName){
    ry::RyLGP_Tree lgp;
    lgp.lgp = make_shared<LGP_Tree_Thread>(self.get(), folFileName.c_str());
    return lgp;
  },
  "create an LGP solver"
  )

  .def("bullet", [](ry::Config& self){
    ry::RyBullet bullet;
    bullet.bullet = make_shared<BulletInterface>(self.get());
    return bullet;
  },
  "create a Bullet engine for physical simulation from the configuration: The configuration\
  is being exported into a bullet instance, which can be stepped forward, and the result syced back to this configuration"
  )

  .def("physx", [](ry::Config& self){
    ry::RyPhysX physx;
    physx.physx = make_shared<PhysXInterface>(self.set());
    return physx;
  },
  "create a PhysX engine for physical simulation from the configuration: The configuration\
  is being exported into a bullet instance, which can be stepped forward, and the result syced back to this configuration"
  )

  .def("operate", [](ry::Config& self, const char* rosNodeName){
    ry::RyOperate op;
    op.R = make_shared<RobotOperation>(self.get(), .01, rosNodeName);
    return op;
  },
  "create a module (including ROS node) to sync this configuration both ways (reading state, and controlling) to a real robot",
  py::arg("rosNodeName")
  )

  .def("sortFrames", [](ry::Config& self){
    self.set()->sortFrames();
  })
    
  .def("equationOfMotion", [](ry::Config& self, bool gravity){
    arr M, F;
    self.set()->equationOfMotion(M, F, gravity);
    return pybind11::make_tuple(pybind11::array(M.dim(), M.p), pybind11::array(F.dim(), F.p));
  }, "",
    py::arg("gravity"))

  .def("stepDynamics", [](ry::Config& self, std::vector<double>& u_control, double tau, double dynamicNoise, bool gravity){
    arr _u = conv_stdvec2arr(u_control);
    self.set()->stepDynamics(_u, tau, dynamicNoise, gravity);
  }, "",
    py::arg("u_control"),
    py::arg("tau"),
    py::arg("dynamicNoise"),
    py::arg("gravity"))

 .def("getJointState_qdot", [](ry::Config& self, const ry::I_StringA& joints) {
   arr q, qdot;
   self.get()->getJointState(q, qdot);
   return pybind11::make_tuple(pybind11::array(q.dim(), q.p), pybind11::array(qdot.dim(), qdot.p));
  }, "",
  py::arg("joints") = ry::I_StringA() )
    
  .def("setJointState_qdot", [](ry::Config& self, const std::vector<double>& q, const std::vector<double>& qdot){
    arr _q = conv_stdvec2arr(q);
    arr _qdot = conv_stdvec2arr(qdot);
    self.set()->setJointState(_q, _qdot);
  }, "",
    py::arg("q"),
    py::arg("qdot") )
    
  ;

//  py::class_<ry::Display>(m, "Display")
//      .def("update", (void (ry::Display::*)(bool)) &ry::Display::update)
//      .def("update", (void (ry::Display::*)(std::string, bool)) &ry::Display::update);

  //===========================================================================

  py::class_<ry::ConfigViewer>(m, "ConfigViewer");
  py::class_<ry::PathViewer>(m, "PathViewer");
  py::class_<ry::PointCloudViewer>(m, "PointCloudViewer");
  py::class_<ry::ImageViewer>(m, "ImageViewer");

  //===========================================================================

  py::class_<ry::RyCameraView>(m, "CameraView")
  .def("updateConfig", [](ry::RyCameraView& self, ry::Config& config){
    auto Cget = config.get();
    if(Cget->frames.N!= self.cam->K.frames.N){
      self.cam->K.copy(Cget);
    }else{
      self.cam->K.setFrameState(Cget->getFrameState());
    }
  } )

  .def("addSensor", [](ry::RyCameraView& self, const char* name, const char* frameAttached, uint width, uint height, double focalLength, double orthoAbsHeight, const std::vector<double>& zRange, const std::string& backgroundImageFile){
    self.cam->addSensor(name, frameAttached, width, height, focalLength, orthoAbsHeight, arr(zRange), backgroundImageFile.c_str());
  }, "",
      py::arg("name"),
      py::arg("frameAttached"),
      py::arg("width"),
      py::arg("height"),
      py::arg("focalLength") = -1.,
      py::arg("orthoAbsHeight") = -1.,
      py::arg("zRange") = std::vector<double>(),
      py::arg("backgroundImageFile") = std::string() )

   .def("selectSensor", [](ry::RyCameraView& self, const char* sensorName){
        self.cam->selectSensor(sensorName);
      }, "",
      py::arg("name") )

   .def("computeImageAndDepth", [](ry::RyCameraView& self, bool visualsOnly){
      auto imageSet = self.image.set();
      auto depthSet = self.depth.set();
      if(visualsOnly) self.cam->renderMode = CameraView::visuals;
      else self.cam->renderMode = CameraView::all;
      self.cam->computeImageAndDepth(imageSet, depthSet);
      pybind11::tuple ret(2);
      ret[0] = pybind11::array(imageSet->dim(), imageSet->p);
      ret[1] = pybind11::array(depthSet->dim(), depthSet->p);
      return ret;
    },
      py::arg("visualsOnly")=true
    )

    .def("computePointCloud", [](ry::RyCameraView& self, const pybind11::array& depth, bool globalCoordinates){
      arr _depth = numpy2arr(depth);
      floatA __depth; copy(__depth, _depth);
      auto ptsSet = self.pts.set();
      self.cam->computePointCloud(ptsSet, __depth, globalCoordinates);
      return pybind11::array(ptsSet->dim(), ptsSet->p);
    }, "",
      py::arg("depth"),
      py::arg("globalCoordinates") = true )

   .def("computeSegmentation", [](ry::RyCameraView& self){
      auto segSet = self.segmentation.set();
      self.cam->computeSegmentation(segSet);
      return pybind11::array(segSet->dim(), segSet->p);
    } )

    .def("pointCloudViewer", [](ry::RyCameraView& self){
      ry::PointCloudViewer ret;
      ret.view = make_shared<PointCloudViewer>(self.pts, self.image);
      return ret;
    } )

    .def("imageViewer", [](ry::RyCameraView& self){
      ry::ImageViewer ret;
      ret.view = make_shared<ImageViewer>(self.image);
      return ret;
    } )

    .def("segmentationViewer", [](ry::RyCameraView& self){
      ry::ImageViewer ret;
      ret.view = make_shared<ImageViewer>(self.segmentation);
      return ret;
    } )

      //-- displays
//      void watch_PCL(const arr& pts, const byteA& rgb);
      ;

  //===========================================================================

  py::class_<ry::RyFeature>(m, "Feature")
  .def("eval", [](ry::RyFeature& self, ry::Config& K){
    arr y,J;
    self.feature->__phi(y, J, K.get());
    pybind11::tuple ret(2);
    ret[0] = pybind11::array(y.dim(), y.p);
    ret[1] = pybind11::array(J.dim(), J.p);
    return ret;
  } )
  .def("eval", [](ry::RyFeature& self, pybind11::tuple& Kpytuple){
    WorldL Ktuple;
    for(uint i=0;i<Kpytuple.size();i++){
      ry::Config& K = Kpytuple[i].cast<ry::Config&>();
      Ktuple.append(&K.set()());
    }

    arr y, J;
    self.feature->order=Ktuple.N-1;
    self.feature->__phi(y, J, Ktuple);
    cout <<"THERE!!" <<J.dim() <<endl;
    pybind11::tuple ret(2);
    ret[0] = pybind11::array(y.dim(), y.p);
    ret[1] = pybind11::array(J.dim(), J.p);
    return ret;
  } )
  .def("description", [](ry::RyFeature& self, ry::Config& K){
    std::string s = self.feature->shortTag(K.get()).p;
    return s;
  } )
  ;

  //===========================================================================

  py::class_<ry::RyFrame>(m, "Frame")
  .def("setPointCloud", [](ry::RyFrame& self, const pybind11::array& points, const pybind11::array_t<byte>& colors){
    arr _points = numpy2arr(points);
    byteA _colors = numpy2arr(colors);
    WToken<rai::KinematicWorld> token(*self.config, &self.config->data);
    self.frame->setPointCloud(_points, _colors);
  } )

  .def("setShape", [](ry::RyFrame& self, rai::ShapeType shape, const std::vector<double>& size){
    WToken<rai::KinematicWorld> token(*self.config, &self.config->data);
    self.frame->setShape(shape, size);
  },
  py::arg("type"),
  py::arg("size")
  )

  .def("setColor", [](ry::RyFrame& self, const std::vector<double>& color){
    WToken<rai::KinematicWorld> token(*self.config, &self.config->data);
    self.frame->setColor(color);
  } )

  .def("setPose", [](ry::RyFrame& self, const std::string& pose){
    WToken<rai::KinematicWorld> token(*self.config, &self.config->data);
    self.frame->X.setText(pose.c_str());
  } )

  .def("setPosition", [](ry::RyFrame& self, const std::vector<double>& pos){
    WToken<rai::KinematicWorld> token(*self.config, &self.config->data);
    self.frame->setPosition(pos);
  } )

  .def("setQuaternion", [](ry::RyFrame& self, const std::vector<double>& quat){
    WToken<rai::KinematicWorld> token(*self.config, &self.config->data);
    self.frame->setQuaternion(quat);
  } )

  .def("setRelativePose", [](ry::RyFrame& self, const std::string& pose){
    WToken<rai::KinematicWorld> token(*self.config, &self.config->data);
    self.frame->Q.setText(pose.c_str());
    self.frame->calc_X_from_parent();
  } )

  .def("setRelativePosition", [](ry::RyFrame& self, const std::vector<double>& pos){
    WToken<rai::KinematicWorld> token(*self.config, &self.config->data);
    self.frame->setRelativePosition(pos);
  } )

  .def("setRelativeQuaternion", [](ry::RyFrame& self, const std::vector<double>& quat){
    WToken<rai::KinematicWorld> token(*self.config, &self.config->data);
    self.frame->setRelativeQuaternion(quat);
  } )

  .def("setJoint", [](ry::RyFrame& self, rai::JointType jointType){
    WToken<rai::KinematicWorld> token(*self.config, &self.config->data);
    self.frame->setJoint(jointType);
  } )

  .def("setContact", [](ry::RyFrame& self, int cont){
    WToken<rai::KinematicWorld> token(*self.config, &self.config->data);
    self.frame->setContact(cont);
  } )


  .def("getPosition", [](ry::RyFrame& self){
    RToken<rai::KinematicWorld> token(*self.config, &self.config->data);
    arr x = self.frame->getPosition();
    return pybind11::array_t<double>(x.dim(), x.p);
  } )

  .def("getQuaternion", [](ry::RyFrame& self){
    RToken<rai::KinematicWorld> token(*self.config, &self.config->data);
    arr x = self.frame->getQuaternion();
    return pybind11::array_t<double>(x.dim(), x.p);
  } )

  .def("getRotationMatrix", [](ry::RyFrame& self){
    RToken<rai::KinematicWorld> token(*self.config, &self.config->data);
    arr x = self.frame->getRotationMatrix();
    return pybind11::array_t<double>(x.dim(), x.p);
  } )

  .def("getRelativePosition", [](ry::RyFrame& self){
    RToken<rai::KinematicWorld> token(*self.config, &self.config->data);
    arr x = self.frame->getRelativePosition();
    return pybind11::array_t<double>(x.dim(), x.p);
  } )

  .def("getRelativeQuaternion", [](ry::RyFrame& self){
    RToken<rai::KinematicWorld> token(*self.config, &self.config->data);
    arr x = self.frame->getRelativeQuaternion();
    return pybind11::array_t<double>(x.dim(), x.p);
  } )

  .def("getMeshPoints", [](ry::RyFrame& self){
    RToken<rai::KinematicWorld> token(*self.config, &self.config->data);
    arr x = self.frame->getMeshPoints();
    return pybind11::array_t<double>(x.dim(), x.p);
  } )

  .def("info", [](ry::RyFrame& self){
    Graph G;
    WToken<rai::KinematicWorld> token(*self.config, &self.config->data);
    G.newNode<rai::String>({"name"}, {}, self.frame->name);
    G.newNode<int>({"ID"}, {}, self.frame->ID);
    self.frame->write(G);
    if(!G["X"]) G.newNode<arr>({"X"}, {}, self.frame->X.getArr7d());
    return graph2dict(G);
  } )

  .def("setMeshAsLines", [](ry::RyFrame& self, const std::vector<double>& lines){
      CHECK(self.frame, "this is not a valid frame");
      CHECK(self.frame->shape, "this frame is not a mesh!");
      CHECK_EQ(self.frame->shape->type(), rai::ST_mesh, "this frame is not a mesh!");
      uint n = lines.size()/3;
      self.frame->shape->mesh().V = lines;
      self.frame->shape->mesh().V.reshape(n, 3);
      uintA& T = self.frame->shape->mesh().T;
      T.resize(n/2, 2);
      for(uint i=0;i<T.d0;i++){
	T(i,0) = 2*i;
	T(i,1) = 2*i+1;
      }
  } )
  ;

  //===========================================================================
  //
  // KOMO
  //

  py::class_<ry::RyKOMO>(m, "KOMOpy")
  .def("makeObjectsFree", [](ry::RyKOMO& self, const ry::I_StringA& objs){
    self.komo->world.makeObjectsFree(I_conv(objs));
  } )

  .def("activateCollisionPairs", [](ry::RyKOMO& self, const std::vector<std::pair<std::string, std::string> >& collision_pairs){
    for (const auto&  pair : collision_pairs) {
      self.komo->activateCollisions(rai::String(pair.first), rai::String(pair.second));
    }
  } )

  .def("deactivateCollisionPairs", [](ry::RyKOMO& self, const std::vector<std::pair<std::string, std::string> >& collision_pairs){
    for (const auto&  pair : collision_pairs) {
      self.komo->deactivateCollisions(rai::String(pair.first), rai::String(pair.second));
    }
  } )

  .def("addTimeOptimization", [](ry::RyKOMO& self){
    self.komo->setTimeOptimization();
  } )

  .def("clearObjectives", [](ry::RyKOMO& self){
    self.komo->clearObjectives();
  } )

  .def("addSwitch_magic", [](ry::RyKOMO& self, double time, const char* from, const char* to){
      self.komo->addSwitch_magic(time, time, from, to, 0.);
  } )

  .def("addSwitch_dynamicTrans", [](ry::RyKOMO& self, double startTime, double endTime, const char* from, const char* to){
      self.komo->addSwitch_dynamicTrans(startTime, endTime, from, to);
  } )

  .def("addInteraction_elasticBounce", [](ry::RyKOMO& self, double time, const char* from, const char* to, double elasticity, double stickiness){
      self.komo->addContact_elasticBounce(time, from, to, elasticity, stickiness);
  }, "", py::arg("time"),
      py::arg("from"),
      py::arg("to"),
      py::arg("elasticity") = .8,
      py::arg("stickiness") = 0. )

  .def("addObjective", [](ry::RyKOMO& self, const std::vector<double>& time, const ObjectiveType& type, const FeatureSymbol& feature, const ry::I_StringA& frames, const std::vector<double> scale, const std::vector<std::vector<double>> scaleTrans, const std::vector<double>& target, int order){
    arr _scale;
    if(scale.size()) _scale=conv_stdvec2arr(scale);
    if(scaleTrans.size()) _scale=vecvec2arr(scaleTrans);
    self.komo->addObjective(arr(time), type, feature, I_conv(frames), _scale, arr(target), order);
  },"", py::arg("time")=std::vector<double>(),
      py::arg("type"),
      py::arg("feature"),
      py::arg("frames")=ry::I_StringA(),
      py::arg("scale")=std::vector<double>(),
      py::arg("scaleTrans")=std::vector<std::vector<double>>(),
      py::arg("target")=std::vector<double>(),
      py::arg("order")=-1 )

  .def("add_StableRelativePose", [](ry::RyKOMO& self, const std::vector<int>& confs, const char* gripper, const char* object){
      for(uint i=1;i<confs.size();i++)
        self.komo->addObjective(ARR(confs[0], confs[i]), OT_eq, FS_poseDiff, {gripper, object});
      //  for(uint i=0;i<confs.size();i++) self.self->configurations(self.self->k_order+confs[i]) -> makeObjectsFree({object});
      self.komo->world.makeObjectsFree({object});
  },"", py::arg("confs"),
      py::arg("gripper"),
      py::arg("object") )

  .def("add_StablePose", [](ry::RyKOMO& self, const std::vector<int>& confs, const char* object){
    for(uint i=1;i<confs.size();i++)
      self.komo->addObjective(ARR(confs[0], confs[i]), OT_eq, FS_pose, {object});
    //  for(uint i=0;i<confs.size();i++) self.self->configurations(self.self->k_order+confs[i]) -> makeObjectsFree({object});
    self.komo->world.makeObjectsFree({object});
  },"", py::arg("confs"),
      py::arg("object") )

  .def("add_grasp", [](ry::RyKOMO& self, int conf, const char* gripper, const char* object){
    self.komo->addObjective(ARR(conf), OT_eq, FS_distance, {gripper, object});
  } )

  .def("add_place", [](ry::RyKOMO& self, int conf, const char* object, const char* table){
    self.komo->addObjective(ARR(conf), OT_ineq, FS_aboveBox, {table, object});
    self.komo->addObjective(ARR(conf), OT_eq, FS_standingAbove, {table, object});
    self.komo->addObjective(ARR(conf), OT_sos, FS_vectorZ, {object}, {}, {0.,0.,1.});
  } )

  .def("add_resting", [](ry::RyKOMO& self, int conf1, int conf2, const char* object){
    self.komo->addObjective(ARR(conf1, conf2), OT_eq, FS_pose, {object});
  } )

  .def("add_restingRelative", [](ry::RyKOMO& self, int conf1, int conf2, const char* object, const char* tableOrGripper){
    self.komo->addObjective(ARR(conf1, conf2), OT_eq, FS_poseDiff, {tableOrGripper, object});
  } )

  .def("addSkeleton", [](ry::RyKOMO& self, const py::list& L){
    Skeleton S = list2skeleton(L);
    cout <<"SKELETON: " <<S <<endl;
    self.komo->setSkeleton(S);
//    skeleton2Bound(*self.komo, BD_path, S, self.komo->world, self.komo->world, false);
  } )

  .def("addSkeletonBound", [](ry::RyKOMO& self, const py::list& L, BoundType boundType, bool collisions){
    Skeleton S = list2skeleton(L);
    cout <<"SKELETON: " <<S <<endl;
//    self.komo->setSkeleton(S);
    skeleton2Bound(*self.komo, boundType, S, self.komo->world, self.komo->world, collisions);
  } )

  //-- run

  .def("optimize", [](ry::RyKOMO& self, bool reinitialize_randomly){
    self.komo->optimize(reinitialize_randomly);
    self.path.set() = self.komo->getPath_frames();
  } )

  //-- reinitialize with configuration
  .def("setConfigurations", [](ry::RyKOMO& self, ry::Config& C){
    for(rai::KinematicWorld *c:self.komo->configurations){
      c->setFrameState(C.get()->getFrameState());
    }
    self.komo->reset(0.);
  } )

  //-- read out

  .def("getT", [](ry::RyKOMO& self){
    return self.komo->T;
  } )

  .def("getConfiguration", [](ry::RyKOMO& self, int t){
    arr X = self.komo->configurations(t+self.komo->k_order)->getFrameState();
    return pybind11::array(X.dim(), X.p);
  } )

  .def("getPathFrames", [](ry::RyKOMO& self, const ry::I_StringA& frames){
    arr X = self.komo->getPath_frames(I_conv(frames));
    return pybind11::array(X.dim(), X.p);
  } )

  .def("getPathTau", [](ry::RyKOMO& self){
    arr X = self.komo->getPath_tau();
    return pybind11::array(X.dim(), X.p);
  } )

  .def("getForceInteractions", [](ry::RyKOMO& self){
    Graph G = self.komo->getContacts();
    return graph2list(G);
  } )

  .def("getReport", [](ry::RyKOMO& self){
    Graph G = self.komo->getProblemGraph(true);
    return graph2list(G);
  } )

  .def("getConstraintViolations", [](ry::RyKOMO& self){
    Graph R = self.komo->getReport(false);
    return R.get<double>("constraints");
  } )

  .def("getCosts", [](ry::RyKOMO& self){
    Graph R = self.komo->getReport(false);
    return R.get<double>("sqrCosts");
  } )

  //-- display

  .def("view", [](ry::RyKOMO& self){
    ry::PathViewer view;
    view.view = make_shared<KinPoseViewer>(self.config, self.path, .1);
    return view;
  } )

  .def("display", [](ry::RyKOMO& self){
    self.komo->displayPath(false, true);
  } )

  .def("displayTrajectory", [](ry::RyKOMO& self){
    rai::system("mkdir -p z.vid");
    self.komo->displayTrajectory(1., false, false, "z.vid/");
  } )
  ;

  //===========================================================================

  py::class_<ry::RyLGP_Tree>(m, "LGP_Tree")
  .def("walkToNode", [](ry::RyLGP_Tree& self, const char* seq){
    self.lgp->walkToNode(seq);
  } )

  .def("walkToRoot", [](ry::RyLGP_Tree& self){
    self.lgp->focusNode = self.lgp->root;
  } )

  .def("walkToParent", [](ry::RyLGP_Tree& self){
    self.lgp->focusNode = self.lgp->focusNode->parent;
  } )

  .def("walkToDecision", [](ry::RyLGP_Tree& self, uint decision){
    LGP_Node* focusNode = self.lgp->focusNode;
    if(!focusNode->isExpanded) focusNode->expand();
    self.lgp->focusNode = focusNode->children(decision);
  } )

  .def("getDecisions", [](ry::RyLGP_Tree& self){
    LGP_Node* focusNode = self.lgp->focusNode;
    if(!focusNode->isExpanded) focusNode->expand();
    StringA decisions(focusNode->children.N);
    uint c=0;
    for(LGP_Node* a:focusNode->children) {
      decisions(c++) <<*a->decision;
    }
    return I_conv(decisions);
  } )

  .def("nodeInfo", [](ry::RyLGP_Tree& self){
    Graph G = self.lgp->focusNode->getInfo();
    LOG(0) <<G;
    return graph2dict(G);
  } )

  .def("viewTree", [](ry::RyLGP_Tree& self){
    self.lgp->displayTreeUsingDot();
  } )

  .def("optBound", [](ry::RyLGP_Tree& self, BoundType bound, bool collisions){
    self.lgp->focusNode->optBound(bound, collisions);
    if(bound == BD_seqPath){
      self.lgp->focusNode->komoProblem(bound)->displayTrajectory(.02, false, false);
    }else{
      self.lgp->focusNode->komoProblem(bound)->displayTrajectory(.1, false, false);
    }
  } )

  .def("getKOMOforBound", [](ry::RyLGP_Tree& self, BoundType bound){
    return ry::RyKOMO( self.lgp->focusNode->komoProblem(bound) );
  } )

  .def("addTerminalRule", [](ry::RyLGP_Tree& self, const char* precondition){
    self.lgp->fol.addTerminalRule(precondition);
  } )

  .def("run", [](ry::RyLGP_Tree& self, int verbose){
    self.lgp->displayBound = BD_seqPath;
    self.lgp->LGP_Tree::verbose=verbose;
    self.lgp->threadLoop();
  } )

  .def("stop", [](ry::RyLGP_Tree& self){
    self.lgp->threadStop();
  } )

  .def("numSolutions", [](ry::RyLGP_Tree& self){
    return self.lgp->numSolutions();
  } )

  .def("getReport", [](ry::RyLGP_Tree& self, uint solution, BoundType bound){
    Graph G = self.lgp->getReport(solution, bound);
    return graph2list(G);
  } )

  .def("getKOMO", [](ry::RyLGP_Tree& self, uint solution, BoundType bound){
    const auto& komo = self.lgp->getKOMO(solution, bound);
    return ry::RyKOMO(komo);
  } )

  ;

  //===========================================================================

  py::class_<ry::RyBullet>(m, "RyBullet")
  .def("step", [](ry::RyBullet& self){
    self.bullet->step();
  } )

  .def("step", [](ry::RyBullet& self, ry::Config& C){
    self.bullet->pushKinematicStates(C.get()->frames);
    self.bullet->step();
    self.bullet->pullDynamicStates(C.set()->frames);
  } )

  .def("getState", [](ry::RyBullet& self, ry::Config& C){
    arr V;
    self.bullet->pullDynamicStates(C.set()->frames, V);
    return pybind11::array(V.dim(), V.p);
  } )

  .def("setState", [](ry::RyBullet& self, ry::Config& C, const pybind11::array& velocities){
    self.bullet->pushFullState(C.get()->frames, numpy2arr(velocities));
  } )

  ;

  //===========================================================================

  py::class_<ry::RyPhysX>(m, "RyPhysX")
  .def("step", [](ry::RyPhysX& self){
    self.physx->step();
  } )

  .def("step", [](ry::RyPhysX& self, ry::Config& C){
    self.physx->pushKinematicStates(C.get()->frames);
    self.physx->step();
    self.physx->pullDynamicStates(C.set()->frames);
  } )

  .def("getState", [](ry::RyPhysX& self, ry::Config& C){
    arr V;
    self.physx->pullDynamicStates(C.set()->frames, V);
    return pybind11::array(V.dim(), V.p);
  } )

  .def("setState", [](ry::RyPhysX& self, ry::Config& C, const pybind11::array& velocities){
    self.physx->pushFullState(C.get()->frames, numpy2arr(velocities));
  } )

  ;

  //===========================================================================

  py::class_<ry::RyOperate>(m, "RyOperate")
  .def("move", [](ry::RyOperate& self, const std::vector<std::vector<double>>& poses, const std::vector<double>& times, bool append){
    arr path(poses.size(), poses[0].size());
    for(uint i=0;i<path.d0;i++) path[i] = conv_stdvec2arr(poses[i]);
    self.R->move(path, conv_stdvec2arr(times), append);
  } )

  .def("move", [](ry::RyOperate& self, const pybind11::array& path, const std::vector<double>& times, bool append){
    arr _path = numpy2arr(path);
    self.R->move(_path, conv_stdvec2arr(times), append);
  } )

  .def("moveHard", [](ry::RyOperate& self, const pybind11::array& pose){
    arr _pose = numpy2arr(pose);
    self.R->moveHard(_pose);
  } )

  .def("timeToGo", [](ry::RyOperate& self){
    return self.R->timeToGo();
  } )

  .def("wait", [](ry::RyOperate& self){
    return self.R->wait();
  } )

  .def("getJointPositions", [](ry::RyOperate& self, ry::Config& C){
    arr q = self.R->getJointPositions();
    return pybind11::array(q.dim(), q.p);
  } )

  .def("getGripperGrabbed", [](ry::RyOperate& self, const std::string& whichArm){
    return self.R->getGripperGrabbed(whichArm);
  } )

  .def("getGripperOpened", [](ry::RyOperate& self, const std::string& whichArm){
    return self.R->getGripperOpened(whichArm);
  } )

  .def("sendToReal", [](ry::RyOperate& self, bool activate){
    self.R->sendToReal(activate);
  } )

  .def("sync", [](ry::RyOperate& self, ry::Config& C){
    self.R->sync(C.set());
  } )

  ;

  //===========================================================================

  py::class_<ry::RyCamera>(m, "Camera")
  .def(py::init<const char*, const char*, const char*, bool>()
       , "", py::arg("rosNodeName"),
             py::arg("rgb_topic"),
             py::arg("depth_topic"),
             py::arg("useUint") = false)

  .def("getRgb", [](ry::RyCamera& self){
    byteA rgb = self.rgb.get();
    return pybind11::array_t<byte>(rgb.dim(), rgb.p);
  } )

  .def("getDepth", [](ry::RyCamera& self){
    floatA depth = self.depth.get();
    return pybind11::array_t<float>(depth.dim(), depth.p);
  } )

  .def("getPoints", [](ry::RyCamera& self, const std::vector<double>& Fxypxy){
    floatA _depth = self.depth.get();
    arr _points;
    CHECK_EQ(Fxypxy.size(), 4, "I need 4 intrinsic calibration parameters")
    depthData2pointCloud(_points, _depth, Fxypxy[0], Fxypxy[1], Fxypxy[2], Fxypxy[3]);
    return pybind11::array_t<double>(_points.dim(), _points.p);
  } )

  .def("transform_image2world", [](ry::RyCamera& self, const std::vector<double>& pt, const char* cameraFrame, const std::vector<double>& Fxypxy){
    NIY
//    arr _pt = pt;
//    depthData2point(_pt, conv_stdvec2arr(Fxypxy)); //transforms the point to camera xyz coordinates

//    pcl->X.applyOnPoint(pt); //transforms into world coordinates

//    return pybind11::array_t<double>(_pt.dim(), _pt.p);
  } )

  ;

  //===========================================================================

#define ENUMVAL(pre, x) .value(#x, pre##_##x)

  py::enum_<ObjectiveType>(m, "OT")
      ENUMVAL(OT,none)
      ENUMVAL(OT,f)
      ENUMVAL(OT,sos)
      ENUMVAL(OT,ineq)
      ENUMVAL(OT,eq)
      .export_values();

  py::enum_<rai::ShapeType>(m, "ST")
      ENUMVAL(rai::ST,none)
      ENUMVAL(rai::ST,box)
      ENUMVAL(rai::ST,sphere)
      ENUMVAL(rai::ST,capsule)
      ENUMVAL(rai::ST,mesh)
      ENUMVAL(rai::ST,cylinder)
      ENUMVAL(rai::ST,marker)
      ENUMVAL(rai::ST,pointCloud)
      ENUMVAL(rai::ST,ssCvx)
      ENUMVAL(rai::ST,ssBox)
      .export_values();

  py::enum_<BoundType>(m, "BT")
      ENUMVAL(BD,all)
      ENUMVAL(BD,symbolic)
      ENUMVAL(BD,pose)
      ENUMVAL(BD,seq)
      ENUMVAL(BD,path)
      ENUMVAL(BD,seqPath)
      ENUMVAL(BD,max)
      .export_values();

  py::enum_<SkeletonSymbol>(m, "SY")
      ENUMVAL(SY,touch)
      ENUMVAL(SY,above)
      ENUMVAL(SY,inside)
      ENUMVAL(SY,impulse)
      ENUMVAL(SY,stable)
      ENUMVAL(SY,stableOn)
      ENUMVAL(SY,dynamic)
      ENUMVAL(SY,dynamicOn)
      ENUMVAL(SY,dynamicTrans)
      ENUMVAL(SY,liftDownUp)

      ENUMVAL(SY,contact)
      ENUMVAL(SY,bounce)

      ENUMVAL(SY,magic)

      ENUMVAL(SY,push)
      ENUMVAL(SY,graspSlide)
      .export_values();

  py::enum_<FeatureSymbol>(m, "FS")
      ENUMVAL(FS,none)
      ENUMVAL(FS,position)
      ENUMVAL(FS,positionDiff)
      ENUMVAL(FS,positionRel)
      ENUMVAL(FS,quaternion)
      ENUMVAL(FS,quaternionDiff)
      ENUMVAL(FS,quaternionRel)
      ENUMVAL(FS,pose)
      ENUMVAL(FS,poseDiff)
      ENUMVAL(FS,poseRel)
      ENUMVAL(FS,vectorX)
      ENUMVAL(FS,vectorXDiff)
      ENUMVAL(FS,vectorXRel)
      ENUMVAL(FS,vectorY)
      ENUMVAL(FS,vectorYDiff)
      ENUMVAL(FS,vectorYRel)
      ENUMVAL(FS,vectorZ)
      ENUMVAL(FS,vectorZDiff)
      ENUMVAL(FS,vectorZRel)
      ENUMVAL(FS,scalarProductXX)
      ENUMVAL(FS,scalarProductXY)
      ENUMVAL(FS,scalarProductXZ)
      ENUMVAL(FS,scalarProductYX)
      ENUMVAL(FS,scalarProductYY)
      ENUMVAL(FS,scalarProductYZ)
      ENUMVAL(FS,scalarProductZZ)
      ENUMVAL(FS,gazeAt)

      ENUMVAL(FS,accumulatedCollisions)
      ENUMVAL(FS,jointLimits)
      ENUMVAL(FS,distance)

      ENUMVAL(FS,qItself)

      ENUMVAL(FS,aboveBox)
      ENUMVAL(FS,insideBox)

      ENUMVAL(FS,standingAbove)

      ENUMVAL(FS,physics)
      ENUMVAL(FS,contactConstraints)
      ENUMVAL(FS,energy)

      ENUMVAL(FS,transAccelerations)
      ENUMVAL(FS,transVelocities)
      .export_values();

#undef ENUMVAL

}

#endif
