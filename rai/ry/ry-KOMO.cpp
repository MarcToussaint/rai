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
    CHECK(when.size()<=2, "Skeleton error entry " <<i/3 <<" time interval: interval needs no, 1, or 2 elements");
    if(when.size()==0) when={0.,-1.};
    if(when.size()==1) when={when[0],when[0]};
    SkeletonSymbol symbol;
    try{
      symbol = L[i+1].cast<SkeletonSymbol>();
    } catch(std::runtime_error& err) {
      LOG(-1) <<"Skeleton error line " <<i/3 <<" symbol: " <<err.what() <<endl;
    }
    StringA frames;
    try{
      frames = L[i+2].cast<StringA>();
    } catch(std::runtime_error& err) {
      LOG(-1) <<"Skeleton error line " <<i/3 <<" frames: " <<err.what() <<endl;
    }
    S.append(SkeletonEntry(when[0], when[1], symbol, frames));
  }
  return S;
}

void checkView(shared_ptr<KOMO>& self){ if(self->pathConfig.hasView()) self->pathConfig.watch(); }

void init_KOMO(pybind11::module& m) {
  pybind11::class_<KOMO, std::shared_ptr<KOMO>>(m, "KOMO", "Constrained solver to optimize configurations or paths. (KOMO = k-order Markov Optimization)")

  .def(pybind11::init<>())

  .def("setModel", &KOMO::setModel)
  .def("setTiming", &KOMO::setTiming)

//  .def("makeObjectsFree", [](std::shared_ptr<KOMO>& self, const ry::I_StringA& objs) {
//    self->world.makeObjectsFree(I_conv(objs));
//  })

//  .def("activateCollisionPairs", [](std::shared_ptr<KOMO>& self, const std::vector<std::pair<std::string, std::string>>& collision_pairs) {
//    for(const auto&  pair : collision_pairs) {
//      self->activateCollisions(rai::String(pair.first), rai::String(pair.second));
//    }
//  })

//  .def("deactivateCollisionPairs", [](std::shared_ptr<KOMO>& self, const std::vector<std::pair<std::string, std::string>>& collision_pairs) {
//    for(const auto&  pair : collision_pairs) {
//      self->deactivateCollisions(rai::String(pair.first), rai::String(pair.second));
//    }
//  })

  .def("addTimeOptimization", &KOMO::addTimeOptimization)

  .def("clearObjectives", &KOMO::clearObjectives)

#if 0
      .def("addObjective",
       pybind11::overload_cast<const arr&, const FeatureSymbol&, const StringA&, ObjectiveType, const arr&, const arr&, int, int, int>
       (&KOMO::addObjective),
       "",
       pybind11::arg("times")=arr(),
       pybind11::arg("feature"),
       pybind11::arg("frames")=StringA(),
       pybind11::arg("type"),
       pybind11::arg("scale")=arr(),
       pybind11::arg("target")=arr(),
       pybind11::arg("order")=-1,
       pybind11::arg("deltaFromStep")=0,
       pybind11::arg("deltaToStep")=0
       )
#else
  .def("addObjective", [](std::shared_ptr<KOMO>& self, const arr& times, const FeatureSymbol& feature, const ry::I_StringA& frames, const ObjectiveType& type, const arr& scale, const arr& target, int order) {
        self->addObjective(times, feature, I_conv(frames), type, scale, target, order);
      }, "",
      pybind11::arg("times"),
      pybind11::arg("feature"),
      pybind11::arg("frames")=ry::I_StringA(),
      pybind11::arg("type"),
      pybind11::arg("scale")=arr(),
      pybind11::arg("target")=arr(),
      pybind11::arg("order")=-1)
#endif

//      .def("add_qControlObjective", [](std::shared_ptr<KOMO>& self, const std::vector<double>& time, uint order, double scale, const std::vector<double>& target) {
//        self->add_qControlObjective(arr(time, true), order, scale, arr(target, true));
//      }, "", pybind11::arg("time")=std::vector<double>(),
//      pybind11::arg("order"),
//      pybind11::arg("scale")=double(1.),
//      pybind11::arg("target")=std::vector<double>())

  .def("addSquaredQuaternionNorms",
       &KOMO::addSquaredQuaternionNorms,
       "",
       pybind11::arg("times")=arr(),
       pybind11::arg("scale")=3.
                              )

  .def("add_qControlObjective",
       &KOMO::add_qControlObjective,
       "",
       pybind11::arg("times"),
       pybind11::arg("order"),
       pybind11::arg("scale")=1.,
       pybind11::arg("target")=arr(),
       pybind11::arg("deltaFromStep")=0,
       pybind11::arg("deltaToStep")=0
                                        )

  .def("addSwitch_stable",
       &KOMO::addSwitch_stable,
       "",
       pybind11::arg("startTime"),
       pybind11::arg("endTime"),
       pybind11::arg("prevFromFrame"),
       pybind11::arg("fromFrame"),
       pybind11::arg("toFrame"),
       pybind11::arg("firstSwitch")=true
       )

  .def("addSwitch_magic", &KOMO::addSwitch_magic)

  .def("addSwitch_dynamicTrans", &KOMO::addSwitch_dynamicTrans)

  .def("addInteraction_elasticBounce",
       &KOMO::addContact_elasticBounce,
       "",
       pybind11::arg("time"),
       pybind11::arg("from"),
       pybind11::arg("to"),
       pybind11::arg("elasticity") = .8,
       pybind11::arg("stickiness") = 0.)

  .def("setSkeleton", [](std::shared_ptr<KOMO>& self, const pybind11::list& S, rai::ArgWord sequenceOrPath) {
        self->setSkeleton(list2skeleton(S), sequenceOrPath);
      })

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

  .def("getFrameState", &KOMO::getFrameState)

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
    rai::Graph R = self->getReport(true);
    return graph2dict(R);
  })

  .def("reportProblem", [](std::shared_ptr<KOMO>& self) {
    std::stringstream str;
    self->reportProblem(str);
    return str.str();
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

  .def("view", &KOMO::view)
    .def("view_play",
	 &KOMO::view_play,
	 "",
	 pybind11::arg("pause"),
       pybind11::arg("delay"),
	 pybind11::arg("saveVideoPath") = nullptr)

  .def("view_close", [](shared_ptr<KOMO>& self) {
    self->pathConfig.gl().reset();
  }, "close the view")

  ;


  //===========================================================================

  //  pybind11::class_<ry::ConfigViewer>(m, "ConfigViewer");
    pybind11::class_<Objective, shared_ptr<Objective>>(m, "KOMO_Objective");

  //===========================================================================

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
      //geometric:
      ENUMVAL(SY,touch)
      ENUMVAL(SY,above)
      ENUMVAL(SY,inside)
      ENUMVAL(SY,oppose)

      ENUMVAL(SY,impulse) //old
      ENUMVAL(SY,initial)
      ENUMVAL(SY,free) //old

      //pose constraints:
      ENUMVAL(SY,poseEq)
      ENUMVAL(SY,stableRelPose)
      ENUMVAL(SY,stablePose)

      //mode switches:
      ENUMVAL(SY,stable)
      ENUMVAL(SY,stableOn)
      ENUMVAL(SY,dynamic)
      ENUMVAL(SY,dynamicOn)
      ENUMVAL(SY,dynamicTrans)
      ENUMVAL(SY,quasiStatic)
      ENUMVAL(SY,quasiStaticOn)
      ENUMVAL(SY,downUp) //old
      ENUMVAL(SY,break)

      //interactions:
      ENUMVAL(SY,contact)
      ENUMVAL(SY,contactStick)
      ENUMVAL(SY,contactComplementary)
      ENUMVAL(SY,bounce)

      //mode switches:
      ENUMVAL(SY,magic)
      ENUMVAL(SY,magicTrans)

      //grasps/placements:
      ENUMVAL(SY,topBoxGrasp)
      ENUMVAL(SY,topBoxPlace)

      ENUMVAL(SY,push)  //old
      ENUMVAL(SY,graspSlide) //old

      ENUMVAL(SY,dampMotion)

      ENUMVAL(SY,noCollision) //old
      ENUMVAL(SY,identical)

      ENUMVAL(SY,alignByInt)

      ENUMVAL(SY,makeFree)
      ENUMVAL(SY,forceBalance)
  .export_values();

}

#endif
