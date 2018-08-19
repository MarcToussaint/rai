/*  ------------------------------------------------------------------
    Copyright (c) 2017 Marc Toussaint
    email: marc.toussaint@informatik.uni-stuttgart.de

    This code is distributed under the MIT License.
    Please see <root-path>/LICENSE for details.
    --------------------------------------------------------------  */

#pragma once
#include "feature.h"

//===========================================================================

//TODO: change naming: TMP_...

enum PTMtype {
  TMT_allP, //phi=sum over all proxies (as is standard)
  TMT_listedVsListedP, //phi=sum over all proxies between listed shapes
  TMT_allVsListedP, //phi=sum over all proxies against listed shapes
  TMT_allExceptListedP, //as above, but excluding listed shapes
  TMT_bipartiteP, //sum over proxies between the two sets of shapes (shapes, shapes2)
  TMT_pairsP, //sum over proxies of explicitly listed pairs (shapes is n-times-2)
  TMT_allExceptPairsP, //sum excluding these pairs
  TMT_vectorP //vector of all pair proxies (this is the only case where dim(phi)>1)
};

//===========================================================================

/// Proxy task variable -> TM_AccumulatedCollision
struct TM_Proxy : Feature {
  /// @name data fields
  PTMtype type;
  uintA shapes,shapes2;
  double margin;
  
  TM_Proxy(PTMtype _type,
           uintA _shapes,
           double _margin=.0);
  virtual ~TM_Proxy() {}
  
  virtual void phi(arr& y, arr& J, const rai::KinematicWorld& G);
  virtual uint dim_phi(const rai::KinematicWorld& G);
  virtual rai::String shortTag(const rai::KinematicWorld& G) { return STRING("ProxyCost"); }
  virtual Graph getSpec(const rai::KinematicWorld& K){ return Graph({{"feature", "ProxyCost"}}); }
};

//===========================================================================

struct TM_ProxyConstraint : Feature {
  TM_Proxy proxyCosts;
  TM_ProxyConstraint(PTMtype _type,
                     uintA _shapes,
                     double _margin=.02);
  virtual void phi(arr& y, arr& J, const rai::KinematicWorld& G);
  virtual uint dim_phi(const rai::KinematicWorld& G) { return 1; }
  virtual rai::String shortTag(const rai::KinematicWorld& G) { return "ProxyConstraint"; }
};
