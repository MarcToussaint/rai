/*  ------------------------------------------------------------------
    Copyright (c) 2011-2020 Marc Toussaint
    email: toussaint@tu-berlin.de

    This code is distributed under the MIT License.
    Please see <root-path>/LICENSE for details.
    --------------------------------------------------------------  */

#pragma once

#include "../LGP/LGP_tree.h"
#include "../Core/thread.h"

#include "ry.h"

namespace ry {

typedef Var<rai::Configuration> Config;

struct LGPpy_self : LGP_Tree {
  Config& kin;
  rai::Configuration K;
  FOL_World L;

  LGPpy_self(Config& _kin, const std::string& folFileName);
  ~LGPpy_self();
};

struct LGPpy {
  ptr<LGPpy_self> self;

  LGPpy(Config& _kin, const std::string& folFileName);
  ~LGPpy();

  void optimizeFixedSequence(const std::string& seq);
};

}
