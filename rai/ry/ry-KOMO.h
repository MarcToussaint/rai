#pragma once

#include <Core/thread.h>
#include <KOMO/komo.h>

#include "ry-Config.h"

namespace ry{
    struct RyKOMO {
      RyKOMO() {}
      RyKOMO(ry::Config& self, bool useSwift) {
        komo = make_shared<KOMO>(self.get(), useSwift);
        config.set() = komo->world;
        komo->setIKOpt();
      }
      RyKOMO(ry::Config& self, uint numConfigs, bool useSwift) {
        CHECK_GE(numConfigs, 1, "");
        komo = make_shared<KOMO>(self.get(), useSwift);
        config.set() = komo->world;
        komo->setDiscreteOpt(numConfigs);
      }
      RyKOMO(ry::Config& self, double phases, uint stepsPerPhase, double timePerPhase, bool useSwift) {
        komo = make_shared<KOMO>(self.get(), useSwift);
        config.set() = komo->world;
        komo->setPathOpt(phases, stepsPerPhase, timePerPhase);
      }
      RyKOMO(const ptr<KOMO>& _komo) {
        komo = _komo;
      }

      ptr<KOMO> komo;
      Var<rai::Configuration> config;
      Var<arr> path;
    };
}
