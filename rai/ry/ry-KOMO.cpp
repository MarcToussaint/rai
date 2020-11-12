/*  ------------------------------------------------------------------
    Copyright (c) 2011-2020 Marc Toussaint
    email: toussaint@tu-berlin.de

    This code is distributed under the MIT License.
    Please see <root-path>/LICENSE for details.
    --------------------------------------------------------------  */

#ifdef RAI_PYBIND

#include "types.h"
#include "ry-Config.h"
#include "../KOMO/komo.h"

//#include "../LGP/bounds.h"
#include "../Kin/viewer.h"

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

void checkView(shared_ptr<KOMO>& self){ if(self->pathConfig.hasView()) self->pathConfig.watch(); }

void init_KOMO(pybind11::module& m) {
  pybind11::class_<KOMO, std::shared_ptr<KOMO>>(m, "KOMO", "Constrained solver to optimize configurations or paths. (KOMO = k-order Markov Optimization)")

      .def(pybind11::init<>())

  .def("makeObjectsFree", [](std::shared_ptr<KOMO>& self, const ry::I_StringA& objs) {
    self->world.makeObjectsFree(I_conv(objs));
  })

  .def("activateCollisionPairs", [](std::shared_ptr<KOMO>& self, const std::vector<std::pair<std::string, std::string>>& collision_pairs) {
    for(const auto&  pair : collision_pairs) {
      self->activateCollisions(rai::String(pair.first), rai::String(pair.second));
    }
  })

  .def("deactivateCollisionPairs", [](std::shared_ptr<KOMO>& self, const std::vector<std::pair<std::string, std::string>>& collision_pairs) {
    for(const auto&  pair : collision_pairs) {
      self->deactivateCollisions(rai::String(pair.first), rai::String(pair.second));
    }
  })

  .def("addTimeOptimization", &KOMO::addTimeOptimization)

  .def("clearObjectives", &KOMO::clearObjectives)

  .def("addSwitch_magic", &KOMO::addSwitch_magic)

  .def("addSwitch_dynamicTrans", &KOMO::addSwitch_dynamicTrans)

  .def("addInteraction_elasticBounce", &KOMO::addContact_elasticBounce, "", pybind11::arg("time"),
       pybind11::arg("from"),
       pybind11::arg("to"),
       pybind11::arg("elasticity") = .8,
       pybind11::arg("stickiness") = 0.)

  .def("addObjective", [](std::shared_ptr<KOMO>& self, const std::vector<double>& time, const FeatureSymbol& feature, const ry::I_StringA& frames, const ObjectiveType& type, const std::vector<double> scale, const std::vector<std::vector<double>> scaleTrans, const std::vector<double>& target, int order) {
    arr _scale;
    if(scale.size()) _scale = scale;
    if(scaleTrans.size()) _scale = vecvec2arr(scaleTrans);
    self->addObjective(arr(time, true), feature, I_conv(frames), type, _scale, arr(target, true), order);
  }, "", pybind11::arg("time")=std::vector<double>(),
  pybind11::arg("feature"),
  pybind11::arg("frames")=ry::I_StringA(),
  pybind11::arg("type"),
  pybind11::arg("scale")=std::vector<double>(),
  pybind11::arg("scaleTrans")=std::vector<std::vector<double>>(),
  pybind11::arg("target")=std::vector<double>(),
  pybind11::arg("order")=-1)

  .def("add_qControlObjective", [](std::shared_ptr<KOMO>& self, const std::vector<double>& time, uint order, double scale, const std::vector<double>& target) {
    self->add_qControlObjective(arr(time, true), order, scale, arr(target, true));
  }, "", pybind11::arg("time")=std::vector<double>(),
  pybind11::arg("order"),
  pybind11::arg("scale")=double(1.),
  pybind11::arg("target")=std::vector<double>())

  .def("addSquaredQuaternionNorms", &KOMO::addSquaredQuaternionNorms)

  .def("add_StableRelativePose", [](std::shared_ptr<KOMO>& self, const std::vector<int>& confs, const char* gripper, const char* object) {
    for(uint i=1; i<confs.size(); i++)
      self->addObjective(ARR(confs[0], confs[i]), FS_poseDiff, {gripper, object}, OT_eq);
    //  for(uint i=0;i<confs.size();i++) self.self->configurations(self.self->k_order+confs[i]) -> makeObjectsFree({object});
    self->world.makeObjectsFree({object});
  }, "", pybind11::arg("confs"),
  pybind11::arg("gripper"),
  pybind11::arg("object"))

  .def("add_StablePose", [](std::shared_ptr<KOMO>& self, const std::vector<int>& confs, const char* object) {
    for(uint i=1; i<confs.size(); i++)
      self->addObjective(ARR(confs[0], confs[i]), FS_pose, {object}, OT_eq);
    //  for(uint i=0;i<confs.size();i++) self.self->configurations(self.self->k_order+confs[i]) -> makeObjectsFree({object});
    self->world.makeObjectsFree({object});
  }, "", pybind11::arg("confs"),
  pybind11::arg("object"))

  .def("add_grasp", [](std::shared_ptr<KOMO>& self, int conf, const char* gripper, const char* object) {
    self->addObjective(ARR(conf), FS_distance, {gripper, object}, OT_eq);
  })

  .def("add_place", [](std::shared_ptr<KOMO>& self, int conf, const char* object, const char* table) {
    self->addObjective(ARR(conf), FS_aboveBox, {table, object}, OT_ineq);
    self->addObjective(ARR(conf), FS_standingAbove, {table, object}, OT_eq);
    self->addObjective(ARR(conf), FS_vectorZ, {object}, OT_sos, {}, {0., 0., 1.});
  })

  .def("add_resting", [](std::shared_ptr<KOMO>& self, int conf1, int conf2, const char* object) {
    self->addObjective(ARR(conf1, conf2), FS_pose, {object}, OT_eq);
  })

  .def("add_restingRelative", [](std::shared_ptr<KOMO>& self, int conf1, int conf2, const char* object, const char* tableOrGripper) {
    self->addObjective(ARR(conf1, conf2), FS_poseDiff, {tableOrGripper, object}, OT_eq);
  })

  .def("addSkeleton", [](std::shared_ptr<KOMO>& self, const pybind11::list& L) {
    Skeleton S = list2skeleton(L);
    cout <<"SKELETON: " <<S <<endl;
    self->setSkeleton(S);
//    skeleton2Bound(*self.komo, BD_path, S, self->world, self->world, false);
  })

//.def("addSkeletonBound", [](std::shared_ptr<KOMO>& self, const pybind11::list& L, BoundType boundType, bool collisions) {
//  Skeleton S = list2skeleton(L);
//  cout <<"SKELETON: " <<S <<endl;
////    self->setSkeleton(S);
//  skeleton2Bound(self.komo, boundType, S, self->world, self->world, collisions);
//})

//-- run

  .def("optimize", [](std::shared_ptr<KOMO>& self, double addInitializationNoise) {
    self->optimize(addInitializationNoise);
    checkView(self);
  }, "",
  pybind11::arg("addInitializationNoise")=0.01)

//-- reinitialize with configuration
  .def("setConfigurations", [](std::shared_ptr<KOMO>& self, shared_ptr<rai::Configuration>& C) {
    arr X = C->getFrameState();
    for(uint t=0;t<self->T;t++){
      self->pathConfig.setFrameState( X, self->timeSlices[t] );
    }
    checkView(self);
  })

//-- read out

  .def("getT", [](std::shared_ptr<KOMO>& self) {
    return self->T;
  })

  .def("getConfiguration", [](std::shared_ptr<KOMO>& self, int t) {
      return self->getFrameState(t);
  })

  .def("getPathFrames", &KOMO::getPath_frames)
//  .def("getPathFrames", [](std::shared_ptr<KOMO>& self, const ry::I_StringA& frames) {
//    arr X = self->getPath_frames(I_conv(frames));
//    return pybind11::array(X.dim(), X.p);
//  })

  .def("getPathTau", [](std::shared_ptr<KOMO>& self) {
    arr X = self->getPath_tau();
    return pybind11::array(X.dim(), X.p);
  })

  .def("getForceInteractions", [](std::shared_ptr<KOMO>& self) {
    rai::Graph G = self->pathConfig.reportForces();
    return graph2list(G);
  })

  .def("getReport", [](std::shared_ptr<KOMO>& self) {
    rai::Graph G = self->getProblemGraph(true);
    return graph2list(G);
  })

  .def("getConstraintViolations", [](std::shared_ptr<KOMO>& self) {
    rai::Graph R = self->getReport(false);
    return R.get<double>("ineq") + R.get<double>("eq");
  })

  .def("getCosts", [](std::shared_ptr<KOMO>& self) {
    rai::Graph R = self->getReport(false);
    return R.get<double>("sos");
  })

//-- display

  .def("view", [](std::shared_ptr<KOMO>& self) {
    self->pathConfig.watch(false, "KOMO path configuration");
  })
  ;

#define ENUMVAL(pre, x) .value(#x, pre##_##x)

  // pybind11::enum_<ObjectiveType>(m, "OT")
  // ENUMVAL(OT, none)
  // ENUMVAL(OT, f)
  // ENUMVAL(OT, sos)
  // ENUMVAL(OT, ineq)
  // ENUMVAL(OT, eq)
  // .export_values();

//pybind11::enum_<BoundType>(m, "BT")
//ENUMVAL(BD, all)
//ENUMVAL(BD, symbolic)
//ENUMVAL(BD, pose)
//ENUMVAL(BD, seq)
//ENUMVAL(BD, path)
//ENUMVAL(BD, seqPath)
//ENUMVAL(BD, max)
//.export_values();

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
