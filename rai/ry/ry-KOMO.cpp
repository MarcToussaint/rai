#ifdef RAI_PYBIND

#include "ry-KOMO.h"
#include "types.h"

#include "../LGP/bounds.h"
#include "../Kin/kinViewer.h"

Skeleton list2skeleton(const pybind11::list& L) {
  Skeleton S;
  for(uint i=0; i<L.size(); i+=3) {
    std::vector<double> when = L[i].cast<std::vector<double>>();
    SkeletonSymbol symbol = L[i+1].cast<SkeletonSymbol>();
    ry::I_StringA frames = L[i+2].cast<ry::I_StringA>();
    S.append(SkeletonEntry(when[0], when[1], symbol, I_conv(frames)));
  }
  return S;
}

void init_KOMO(pybind11::module &m) {
pybind11::class_<ry::RyKOMO>(m, "KOMOpy")
.def("makeObjectsFree", [](ry::RyKOMO& self, const ry::I_StringA& objs) {
  self.komo->world.makeObjectsFree(I_conv(objs));
})

.def("activateCollisionPairs", [](ry::RyKOMO& self, const std::vector<std::pair<std::string, std::string>>& collision_pairs) {
  for(const auto&  pair : collision_pairs) {
    self.komo->activateCollisions(rai::String(pair.first), rai::String(pair.second));
  }
})

.def("deactivateCollisionPairs", [](ry::RyKOMO& self, const std::vector<std::pair<std::string, std::string>>& collision_pairs) {
  for(const auto&  pair : collision_pairs) {
    self.komo->deactivateCollisions(rai::String(pair.first), rai::String(pair.second));
  }
})

.def("addTimeOptimization", [](ry::RyKOMO& self) {
  self.komo->setTimeOptimization();
})

.def("clearObjectives", [](ry::RyKOMO& self) {
  self.komo->clearObjectives();
})

.def("addSwitch_magic", [](ry::RyKOMO& self, double time, const char* from, const char* to) {
  self.komo->addSwitch_magic(time, time, from, to, 0., 0.);
})

.def("addSwitch_dynamicTrans", [](ry::RyKOMO& self, double startTime, double endTime, const char* from, const char* to) {
  self.komo->addSwitch_dynamicTrans(startTime, endTime, from, to);
})

.def("addInteraction_elasticBounce", [](ry::RyKOMO& self, double time, const char* from, const char* to, double elasticity, double stickiness) {
  self.komo->addContact_elasticBounce(time, from, to, elasticity, stickiness);
}, "", pybind11::arg("time"),
pybind11::arg("from"),
pybind11::arg("to"),
pybind11::arg("elasticity") = .8,
pybind11::arg("stickiness") = 0.)

.def("addObjective", [](ry::RyKOMO& self, const std::vector<double>& time, const FeatureSymbol& feature, const ry::I_StringA& frames, const ObjectiveType& type, const std::vector<double> scale, const std::vector<std::vector<double>> scaleTrans, const std::vector<double>& target, int order) {
  arr _scale;
  if(scale.size()) _scale=conv_stdvec2arr(scale);
  if(scaleTrans.size()) _scale=vecvec2arr(scaleTrans);
  self.komo->addObjective(arr(time), feature, I_conv(frames), type, _scale, arr(target), order);
}, "", pybind11::arg("time")=std::vector<double>(),
pybind11::arg("feature"),
pybind11::arg("frames")=ry::I_StringA(),
pybind11::arg("type"),
pybind11::arg("scale")=std::vector<double>(),
pybind11::arg("scaleTrans")=std::vector<std::vector<double>>(),
pybind11::arg("target")=std::vector<double>(),
pybind11::arg("order")=-1)

.def("add_StableRelativePose", [](ry::RyKOMO& self, const std::vector<int>& confs, const char* gripper, const char* object) {
  for(uint i=1; i<confs.size(); i++)
    self.komo->addObjective(ARR(confs[0], confs[i]), FS_poseDiff, {gripper, object}, OT_eq);
  //  for(uint i=0;i<confs.size();i++) self.self->configurations(self.self->k_order+confs[i]) -> makeObjectsFree({object});
  self.komo->world.makeObjectsFree({object});
}, "", pybind11::arg("confs"),
pybind11::arg("gripper"),
pybind11::arg("object"))

.def("add_StablePose", [](ry::RyKOMO& self, const std::vector<int>& confs, const char* object) {
  for(uint i=1; i<confs.size(); i++)
    self.komo->addObjective(ARR(confs[0], confs[i]), FS_pose, {object}, OT_eq);
  //  for(uint i=0;i<confs.size();i++) self.self->configurations(self.self->k_order+confs[i]) -> makeObjectsFree({object});
  self.komo->world.makeObjectsFree({object});
}, "", pybind11::arg("confs"),
pybind11::arg("object"))

.def("add_grasp", [](ry::RyKOMO& self, int conf, const char* gripper, const char* object) {
  self.komo->addObjective(ARR(conf), FS_distance, {gripper, object}, OT_eq);
})

.def("add_place", [](ry::RyKOMO& self, int conf, const char* object, const char* table) {
  self.komo->addObjective(ARR(conf), FS_aboveBox, {table, object}, OT_ineq);
  self.komo->addObjective(ARR(conf), FS_standingAbove, {table, object}, OT_eq);
  self.komo->addObjective(ARR(conf), FS_vectorZ, {object}, OT_sos, {}, {0., 0., 1.});
})

.def("add_resting", [](ry::RyKOMO& self, int conf1, int conf2, const char* object) {
  self.komo->addObjective(ARR(conf1, conf2), FS_pose, {object}, OT_eq);
})

.def("add_restingRelative", [](ry::RyKOMO& self, int conf1, int conf2, const char* object, const char* tableOrGripper) {
  self.komo->addObjective(ARR(conf1, conf2), FS_poseDiff, {tableOrGripper, object}, OT_eq);
})

.def("addSkeleton", [](ry::RyKOMO& self, const pybind11::list& L) {
  Skeleton S = list2skeleton(L);
  cout <<"SKELETON: " <<S <<endl;
  self.komo->setSkeleton(S);
//    skeleton2Bound(*self.komo, BD_path, S, self.komo->world, self.komo->world, false);
})

.def("addSkeletonBound", [](ry::RyKOMO& self, const pybind11::list& L, BoundType boundType, bool collisions) {
  Skeleton S = list2skeleton(L);
  cout <<"SKELETON: " <<S <<endl;
//    self.komo->setSkeleton(S);
  skeleton2Bound(*self.komo, boundType, S, self.komo->world, self.komo->world, collisions);
})

//-- run

.def("optimize", [](ry::RyKOMO& self, bool reinitialize, double initNoise) {
  self.komo->optimize(reinitialize, initNoise);
  self.path.set() = self.komo->getPath_frames();
}, "",
pybind11::arg("reinitialize")=true,
pybind11::arg("initNoise")=0.01)

//-- reinitialize with configuration
.def("setConfigurations", [](ry::RyKOMO& self, ry::Config& C) {
  for(rai::Configuration* c:self.komo->configurations) {
    c->setFrameState(C.get()->getFrameState());
  }
  self.komo->reset(0.);
})

//-- read out

.def("getT", [](ry::RyKOMO& self) {
  return self.komo->T;
})

.def("getConfiguration", [](ry::RyKOMO& self, int t) {
  arr X = self.komo->configurations(t+self.komo->k_order)->getFrameState();
  return pybind11::array(X.dim(), X.p);
})

.def("getPathFrames", [](ry::RyKOMO& self, const ry::I_StringA& frames) {
  arr X = self.komo->getPath_frames(I_conv(frames));
  return pybind11::array(X.dim(), X.p);
})

.def("getPathTau", [](ry::RyKOMO& self) {
  arr X = self.komo->getPath_tau();
  return pybind11::array(X.dim(), X.p);
})

.def("getForceInteractions", [](ry::RyKOMO& self) {
  Graph G = self.komo->getContacts();
  return graph2list(G);
})

.def("getReport", [](ry::RyKOMO& self) {
  Graph G = self.komo->getProblemGraph(true, false);
  return graph2list(G);
})

.def("getConstraintViolations", [](ry::RyKOMO& self) {
  Graph R = self.komo->getReport(false);
  return R.get<double>("constraints");
})

.def("getCosts", [](ry::RyKOMO& self) {
  Graph R = self.komo->getReport(false);
  return R.get<double>("sqrCosts");
})

//-- display

.def("view", [](ry::RyKOMO& self) {
  ry::PathViewer view;
  view.view = make_shared<KinPoseViewer>(self.config, self.path, .1);
  return view;
})

.def("display", [](ry::RyKOMO& self) {
  self.komo->displayPath(false, true);
})

.def("displayTrajectory", [](ry::RyKOMO& self) {
  rai::system("mkdir -p z.vid");
  self.komo->displayTrajectory(1., false, false, "z.vid/");
})
;

#define ENUMVAL(pre, x) .value(#x, pre##_##x)

pybind11::enum_<ObjectiveType>(m, "OT")
ENUMVAL(OT, none)
ENUMVAL(OT, f)
ENUMVAL(OT, sos)
ENUMVAL(OT, ineq)
ENUMVAL(OT, eq)
.export_values();

pybind11::enum_<BoundType>(m, "BT")
ENUMVAL(BD, all)
ENUMVAL(BD, symbolic)
ENUMVAL(BD, pose)
ENUMVAL(BD, seq)
ENUMVAL(BD, path)
ENUMVAL(BD, seqPath)
ENUMVAL(BD, max)
.export_values();

pybind11::enum_<SkeletonSymbol>(m, "SY")
ENUMVAL(SY, touch)
ENUMVAL(SY, above)
ENUMVAL(SY, inside)
ENUMVAL(SY, impulse)
ENUMVAL(SY, stable)
ENUMVAL(SY, stableOn)
ENUMVAL(SY, dynamic)
ENUMVAL(SY, dynamicOn)
ENUMVAL(SY, dynamicTrans)
ENUMVAL(SY, liftDownUp)

ENUMVAL(SY, contact)
ENUMVAL(SY, bounce)

ENUMVAL(SY, magic)

ENUMVAL(SY, push)
ENUMVAL(SY, graspSlide)
.export_values();

}

#endif
