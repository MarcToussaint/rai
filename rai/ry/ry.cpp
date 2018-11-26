#ifdef RAI_PYBIND

#include "ry.h"
#include "lgp-py.h"

#include <Core/graph.h>
#include <Kin/frame.h>
#include <Kin/kin.h>

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

  //===========================================================================
  //
  // Config
  //

  py::class_<ry::Config>(m, "Config")
      .def(py::init<>())

  METHOD_set(clear)

  .def("copy", [](ry::Config& self, ry::Config& K2) {
    self.set()->copy(K2.get());
  } )

  //-- setup/edit the configuration

  .def("addFile", [](ry::Config& self, const std::string& file) {
    self.set()->addFile(file.c_str());
  } )

  .def("addFrame", [](ry::Config& self, const std::string& name, const std::string& parent, const std::string& args) {
    return self.set()->addFrame(name.c_str(), parent.c_str(), args.c_str())->ID;
  }, "",
    py::arg("name"),
    py::arg("parent") = std::string(),
    py::arg("args") = std::string() )

  .def("delFrame", [](ry::Config& self, const std::string& name) {
    auto Kset = self.set();
    rai::Frame *p = Kset->getFrameByName(name.c_str(), true);
    if(p) delete p;
  } )

  .def("addObject", [](ry::Config& self, const std::string& name, const std::string& parent,
       rai::ShapeType shape,
       const std::vector<double>& size,
       const std::vector<double>& color,
       const std::vector<double>& pos,
       const std::vector<double>& quat,
       const std::vector<double>& rot,
       double radius){
    auto Kset = self.set();
    rai::Frame *f = Kset->addObject(shape, conv_stdvec2arr(size), conv_stdvec2arr(color), radius);
    f->name = name;
    if(parent.size()){
      rai::Frame *p = Kset->getFrameByName(parent.c_str());
      if(p) f->linkFrom(p);
    }
    if(pos.size()) f->Q.pos.set(pos);
    if(quat.size()) f->Q.rot.set(quat);
    if(rot.size()) f->Q.addRelativeRotationDeg(rot[0], rot[1], rot[2], rot[3]);
    if(f->parent){
      f->X = f->parent->X * f->Q;
    }else{
      f->X = f->Q;
    }
  }, "",
    py::arg("name"),
    py::arg("parent") = std::string(),
    py::arg("shape"),
    py::arg("size") = std::vector<double>(),
    py::arg("color") = std::vector<double>(),
    py::arg("pos") = std::vector<double>(),
    py::arg("quat") = std::vector<double>(),
    py::arg("rot") = std::vector<double>(),
    py::arg("radius") = -1. )


  .def("getJointNames", [](ry::Config& self) {
    return I_conv(self.get()->getJointNames());
  } )

  .def("getJointDimension", [](ry::Config& self) {
    return self.get()->getJointStateDimension();
  } )

  .def("getJointState", [](ry::Config& self, const ry::I_StringA& joints) {
    arr q;
    if(joints.size()) q = self.get()->getJointState(I_conv(joints));
    else q = self.get()->getJointState();
    return pybind11::array(q.dim(), q.p);
  }, "",
    py::arg("joints") = ry::I_StringA() )

  .def("setJointState", [](ry::Config& self, const std::vector<double>& q, const ry::I_StringA& joints){
    arr _q = conv_stdvec2arr(q);
    if(joints.size()){
      self.set()->setJointState(_q, I_conv(joints));
    }else{
      self.set()->setJointState(_q);
    }
  }, "",
    py::arg("q"),
    py::arg("joints") = ry::I_StringA() )

  .def("getFrameNames", [](ry::Config& self){
    return I_conv(self.get()->getFrameNames());
  } )

  .def("getFrameState", [](ry::Config& self){
    arr X = self.get()->getFrameState();
    return pybind11::array(X.dim(), X.p);
  } )

  .def("getFrameState", [](ry::Config& self, const char* frame){
    arr X;
    auto Kget = self.get();
    rai::Frame *f = Kget->getFrameByName(frame, true);
    if(f) X = f->X.getArr7d();
    return pybind11::array(X.dim(), X.p);
  } )

  .def("setFrameState", [](ry::Config& self, const std::vector<double>& X, const ry::I_StringA& frames, bool calc_q_from_X){
    arr _X = conv_stdvec2arr(X);
    _X.reshape(_X.N/7, 7);
    self.set()->setFrameState(_X, I_conv(frames), calc_q_from_X);
  }, "",
    py::arg("X"),
    py::arg("frames") = ry::I_StringA(),
    py::arg("calc_q_from_X") = true )

  .def("setFrameState", [](ry::Config& self, const pybind11::array& X, const ry::I_StringA& frames, bool calc_q_from_X){
    arr _X = numpy2arr(X);
    _X.reshape(_X.N/7, 7);
    self.set()->setFrameState(_X, I_conv(frames), calc_q_from_X);
  }, "",
    py::arg("X"),
        py::arg("frames") = ry::I_StringA(),
        py::arg("calc_q_from_X") = true )

  .def("feature", [](ry::Config& self, FeatureSymbol fs, const ry::I_StringA& frames) {
    ry::RyFeature F;
//    F.feature = make_shared<::Feature>(symbols2feature(fs, I_conv(frames), Kget));
    F.feature = symbols2feature(fs, I_conv(frames), self.get());
    return F;
  } )

  .def("evalFeature", [](ry::Config& self, FeatureSymbol fs, const ry::I_StringA& frames) {
    arr y,J;
    self.get()->evalFeature(y, J, fs, I_conv(frames));
    return pybind11::make_tuple(pybind11::array(y.dim(), y.p), pybind11::array(J.dim(), J.p));
  } )

  .def("selectJointsByTag", [](ry::Config& self, const ry::I_StringA& jointGroups){
    auto Kset = self.set();
    Kset->selectJointsByGroup(I_conv(jointGroups), true, true);
    Kset->calc_q();
  } )

  .def("selectJoints", [](ry::Config& self, const ry::I_StringA& joints){
    // TODO: this is joint groups
    // TODO: maybe call joint groups just joints and joints DOFs
    self.set()->selectJointsByName(I_conv(joints));
  } )

  .def("makeObjectsFree", [](ry::Config& self, const ry::I_StringA& objs){
    self.set()->makeObjectsFree(I_conv(objs));
  } )

  .def("computeCollisions", [](ry::Config& self){
    self.set()->stepSwift();
  } )

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
  }, "",
    py::arg("belowMargin") = 1.)

  .def("view", [](ry::Config& self, const std::string& frame){
    ry::ConfigViewer view;
    view.view = make_shared<KinViewer>(self, -1, rai::String(frame));
    return view;
  }, "",
    py::arg("frame")="")

  .def("cameraView", [](ry::Config& self){
    ry::RyCameraView view;
    view.cam = make_shared<rai::CameraView>(self.get(), true, 0);
    return view;
   } )

  .def("komo_IK", [](ry::Config& self){
    ry::RyKOMO komo;
    komo.komo = make_shared<KOMO>(self.get());
    komo.config.set() = komo.komo->world;
    komo.komo->setIKOpt();
    return komo;
  } )

  .def("komo_CGO", [](ry::Config& self, uint numConfigs){
    CHECK_GE(numConfigs, 1, "");
    ry::RyKOMO komo;
    komo.komo = make_shared<KOMO>(self.get());
    komo.config.set() = komo.komo->world;
    komo.komo->setDiscreteOpt(numConfigs);
    return komo;
  } )

  .def("komo_path",  [](ry::Config& self, double phases, uint stepsPerPhase, double timePerPhase){
    ry::RyKOMO komo;
    komo.komo = make_shared<KOMO>(self.get());
    komo.config.set() = komo.komo->world;
    komo.komo->setPathOpt(phases, stepsPerPhase, timePerPhase);
    return komo;
  }, "",
    py::arg("phases"),
    py::arg("stepsPerPhase")=20,
    py::arg("timePerPhase")=5. )

  .def("lgp", [](ry::Config& self, const std::string& folFileName){
    ry::RyLGP_Tree lgp;
    lgp.lgp = make_shared<LGP_Tree>(self.get(), folFileName.c_str());
    return lgp;
  } )
    
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
    
  .def("getEnergy", [](ry::Config& self){
    return self.set()->getEnergy();
  } )
    
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

   .def("computeImageAndDepth", [](ry::RyCameraView& self){
      auto imageSet = self.image.set();
      auto depthSet = self.depth.set();
      self.cam->computeImageAndDepth(imageSet, depthSet);
      pybind11::tuple ret(2);
      ret[0] = pybind11::array(imageSet->dim(), imageSet->p);
      ret[1] = pybind11::array(depthSet->dim(), depthSet->p);
      return ret;
    } )

    .def("computePointCloud", [](ry::RyCameraView& self, const pybind11::array& depth, bool globalCoordinates){
      arr _depth = numpy2arr(depth);
      auto ptsSet = self.pts.set();
      self.cam->computePointCloud(ptsSet, _depth, globalCoordinates);
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
    self.feature->phi(y, J, K.get());
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
    self.feature->phi(y, J, Ktuple);
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
    arr _scale = arr(scale);
    if(scaleTrans.size()) _scale=vecvec2arr(scaleTrans);
    self.komo->addObjective(arr(time), type, feature, I_conv(frames), arr(scale), arr(target), order);
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

  .def("addSkeletonBound", [](ry::RyKOMO& self, const py::list& L, BoundType boundType){
    Skeleton S = list2skeleton(L);
    cout <<"SKELETON: " <<S <<endl;
//    self.komo->setSkeleton(S);
    skeleton2Bound(*self.komo, boundType, S, self.komo->world, self.komo->world, false);
  } )

  //-- run

  .def("optimize", [](ry::RyKOMO& self){
    self.komo->optimize();
    self.path.set() = self.komo->getPath_frames();
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

  .def("getDecisions", [](ry::RyLGP_Tree& self, const char* seq){
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

  .def("optBound", [](ry::RyLGP_Tree& self, BoundType bound, bool collisions){
    self.lgp->focusNode->optBound(bound, collisions);
    if(bound == BD_seqPath){
      self.lgp->focusNode->komoProblem(bound)->displayTrajectory(.02, false, false);
    }else{
      self.lgp->focusNode->komoProblem(bound)->displayTrajectory(.1, false, false);
    }
  } )

  .def("addTerminalRule", [](ry::RyLGP_Tree& self, const char* precondition){
    self.lgp->fol.addTerminalRule(precondition);
  } )

  .def("run", [](ry::RyLGP_Tree& self, int verbose){
    self.lgp->displayBound = BD_seqPath;
    self.lgp->verbose=verbose;
    self.lgp->run();
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
