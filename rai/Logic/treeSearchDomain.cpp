/*  ------------------------------------------------------------------
    Copyright (c) 2011-2020 Marc Toussaint
    email: toussaint@tu-berlin.de

    This code is distributed under the MIT License.
    Please see <root-path>/LICENSE for details.
    --------------------------------------------------------------  */

#include "treeSearchDomain.h"

#include "../Core/util.h"

std::shared_ptr<const rai::TreeSearchDomain::SAO> rai::NoHandle;

std::shared_ptr<rai::TreeSearchNode> rai::TreeSearchNode::transitionRandomly() { return transition(rnd(getNumActions())); }
