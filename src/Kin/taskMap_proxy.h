#pragma once
#include "taskMap.h"

//===========================================================================

enum PTMtype {
  allPTMT, //phi=sum over all proxies (as is standard)
  listedVsListedPTMT, //phi=sum over all proxies between listed shapes
  allVsListedPTMT, //phi=sum over all proxies against listed shapes
  allExceptListedPTMT, //as above, but excluding listed shapes
  bipartitePTMT, //sum over proxies between the two sets of shapes (shapes, shapes2)
  pairsPTMT, //sum over proxies of explicitly listed pairs (shapes is n-times-2)
  allExceptPairsPTMT, //sum excluding these pairs
  vectorPTMT //vector of all pair proxies (this is the only case where dim(phi)>1)
};

//===========================================================================

/// Proxy task variable
struct TaskMap_Proxy:TaskMap {
  /// @name data fields
  PTMtype type;
  uintA shapes,shapes2;
  double margin;
  bool useCenterDist;
  bool useDistNotCost;

  TaskMap_Proxy(PTMtype _type,
               uintA _shapes,
               double _margin=.02,
               bool _useCenterDist=false,
               bool _useDistNotCost=false);
  virtual ~TaskMap_Proxy() {}

  virtual void phi(arr& y, arr& J, const mlr::KinematicWorld& G, int t=-1);
  virtual uint dim_phi(const mlr::KinematicWorld& G);
  virtual mlr::String shortTag(const mlr::KinematicWorld& G){ return STRING("Proxy:"<<shapes); }
};

//===========================================================================

struct TaskMap_ProxyConstraint:TaskMap {
  TaskMap_Proxy proxyCosts;
  TaskMap_ProxyConstraint(PTMtype _type,
                  uintA _shapes,
                  double _margin=.02,
                  bool _useCenterDist=false,
                  bool _useDistNotCost=false);
  virtual void phi(arr& y, arr& J, const mlr::KinematicWorld& G, int t=-1);
  virtual uint dim_phi(const mlr::KinematicWorld& G){ return 1; }
  virtual mlr::String shortTag(const mlr::KinematicWorld& G){ return "ProxyConstraint"; }
};
