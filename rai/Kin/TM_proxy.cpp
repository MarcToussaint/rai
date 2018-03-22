/*  ------------------------------------------------------------------
    Copyright (c) 2017 Marc Toussaint
    email: marc.toussaint@informatik.uni-stuttgart.de
    
    This code is distributed under the MIT License.
    Please see <root-path>/LICENSE for details.
    --------------------------------------------------------------  */

#include "TM_proxy.h"
#include <Kin/proxy.h>

TM_Proxy::TM_Proxy(PTMtype _type,
                           uintA _shapes,
                           double _margin,
                           bool _useCenterDist,
                           bool _useDistNotCost) {
  type=_type;
  shapes=_shapes;
  margin=_margin;
  useCenterDist=_useCenterDist;
  useDistNotCost=_useDistNotCost;
//  cout <<"creating TM_Proxy with shape list" <<shapes <<endl;
}

void TM_Proxy::phi(arr& y, arr& J, const rai::KinematicWorld& G, int t){
  uintA shapes_t;
  shapes_t.referTo(shapes);

  y.resize(1).setZero();
  if(&J) J.resize(1, G.getJointStateDimension()).setZero();

  switch(type) {
    case TMT_allP:
      for(const rai::Proxy& p: G.proxies){
        G.kinematicsProxyCost(y, J, p, margin, useCenterDist, true);
      }
      break;
    case TMT_listedVsListedP:
      for(const rai::Proxy& p: G.proxies){
        if(shapes.contains(p.a->ID) && shapes.contains(p.b->ID)) {
          G.kinematicsProxyCost(y, J, p, margin, useCenterDist, true);
//          p.colorCode = 2;
        }
      }
      break;
    case TMT_allVsListedP: {
      if(t && shapes.nd==2) shapes_t.referToDim(shapes,t);
      for(const rai::Proxy& p: G.proxies){
        if(shapes_t.contains(p.a->ID) || shapes_t.contains(p.b->ID)) {
          G.kinematicsProxyCost(y, J, p, margin, useCenterDist, true);
//          p.colorCode = 2;
        }
      }
    } break;
    case TMT_allExceptListedP:
      for(const rai::Proxy& p: G.proxies){
        if(!(shapes.contains(p.a->ID) && shapes.contains(p.b->ID))) {
          G.kinematicsProxyCost(y, J, p, margin, useCenterDist, true);
//          p.colorCode = 3;
        }
      }
      break;
    case TMT_bipartiteP:
      for(const rai::Proxy& p: G.proxies){
        if((shapes.contains(p.a->ID) && shapes2.contains(p.b->ID)) ||
            (shapes.contains(p.b->ID) && shapes2.contains(p.a->ID))) {
          G.kinematicsProxyCost(y, J, p, margin, useCenterDist, true);
//          p.colorCode = 4;
        }
      }
      break;
    case TMT_pairsP: {
      shapes.reshape(shapes.N/2,2);
      // only explicit paris in 2D array shapes
      uint j;
      for(const rai::Proxy& p: G.proxies){
        for(j=0; j<shapes.d0; j++) {
          if((shapes(j,0)==p.a->ID && shapes(j,1)==p.b->ID) || (shapes(j,0)==p.b->ID && shapes(j,1)==p.a->ID))
            break;
        }
        if(j<shapes.d0) { //if a pair was found
          if(useDistNotCost) G.kinematicsProxyDist(y, J, p, margin, useCenterDist, true);
          else G.kinematicsProxyCost(y, J, p, margin, useCenterDist, true);
//          p.colorCode = 5;
        }
      }
    } break;
    case TMT_allExceptPairsP: {
      shapes.reshape(shapes.N/2,2);
      // only explicit paris in 2D array shapes
      uint j;
      for(const rai::Proxy& p: G.proxies){
        for(j=0; j<shapes.d0; j++) {
          if((shapes(j,0)==p.a->ID && shapes(j,1)==p.b->ID) || (shapes(j,0)==p.b->ID && shapes(j,1)==p.a->ID))
            break;
        }
        if(j==shapes.d0) { //if a pair was not found
          G.kinematicsProxyCost(y, J, p, margin, useCenterDist, true);
//          p.colorCode = 5;
        }
      }
    } break;
    case TMT_vectorP: {
      //outputs a vector of collision meassures, with entry for each explicit pair
      shapes.reshape(shapes.N/2,2);
      y.resize(shapes.d0, 1);  y.setZero();
      if(&J){ J.resize(shapes.d0,J.d1);  J.setZero(); }
      uint j;
      for(const rai::Proxy& p: G.proxies){
        for(j=0; j<shapes.d0; j++) {
          if((shapes(j,0)==p.a->ID && shapes(j,1)==p.b->ID) || (shapes(j,0)==p.b->ID && shapes(j,1)==p.a->ID))
            break;
        }
        if(j<shapes.d0) {
          G.kinematicsProxyCost(y[j](), (&J?J[j]():NoArr), p, margin, useCenterDist, true);
//          p.colorCode = 5;
        }
      }
      y.reshape(shapes.d0);
    } break;
    default: NIY;
  }
}

uint TM_Proxy::dim_phi(const rai::KinematicWorld& G){
  switch(type) {
  case TMT_allP:
  case TMT_listedVsListedP:
  case TMT_allVsListedP:
  case TMT_allExceptListedP:
  case TMT_bipartiteP:
  case TMT_pairsP:
  case TMT_allExceptPairsP:
    return 1;
  case TMT_vectorP:
    return shapes.d0;
  default: NIY;
  }
}

//===========================================================================


TM_ProxyConstraint::TM_ProxyConstraint(PTMtype _type,
                                 uintA _shapes,
                                 double _margin,
                                 bool _useCenterDist,
                                 bool _useDistNotCost)
  : proxyCosts(_type, _shapes, _margin, _useCenterDist, _useDistNotCost){
}

void TM_ProxyConstraint::phi(arr& y, arr& J, const rai::KinematicWorld& G, int t){
  proxyCosts.phi(y, J, G, t);
  y -= .5;
}

