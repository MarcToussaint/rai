//===========================================================================

struct TM_ProxyConstraint : Feature {
  TM_Proxy proxyCosts;
  TM_ProxyConstraint(PTMtype _type,
                     uintA _shapes,
                     double _margin=.02);
  virtual void phi(arr& y, arr& J, const rai::Configuration& G);
  virtual uint dim_phi(const rai::Configuration& G) { return 1; }
  virtual rai::String shortTag(const rai::Configuration& G) { return "ProxyConstraint"; }
};

//===========================================================================

TM_ProxyConstraint::TM_ProxyConstraint(PTMtype _type,
                                       uintA _shapes,
                                       double _margin)
  : proxyCosts(_type, _shapes, _margin) {
}

void TM_ProxyConstraint::phi(arr& y, arr& J, const rai::Configuration& G) {
  proxyCosts.phi(y, J, G);
  y -= .5;
}

//===========================================================================

void TM_proxy::phi(arr& y, arr& J, const rai::Configuration& C) {
  C.kinematicsZero(y, J, 1);

  switch(type) {
    case TMT_allP:
      for(const rai::Proxy& p: C.proxies) {
        C.kinematicsProxyCost(y, J, p, margin, true);
      }
      break;
    case TMT_listedVsListedP:
      for(const rai::Proxy& p: C.proxies) {
        if(frameIDs.contains(p.a->ID) && frameIDs.contains(p.b->ID)) {
          C.kinematicsProxyCost(y, J, p, margin, true);
//          p.colorCode = 2;
        }
      }
      break;
    case TMT_allVsListedP: {
      for(const rai::Proxy& p: C.proxies) {
        if(frameIDs.contains(p.a->ID) || frameIDs.contains(p.b->ID)) {
          C.kinematicsProxyCost(y, J, p, margin, true);
//          p.colorCode = 2;
        }
      }
    } break;
    case TMT_allExceptListedP:
      for(const rai::Proxy& p: C.proxies) {
        if(!(frameIDs.contains(p.a->ID) && frameIDs.contains(p.b->ID))) {
          C.kinematicsProxyCost(y, J, p, margin, true);
//          p.colorCode = 3;
        }
      }
      break;
    case TMT_bipartiteP:
      for(const rai::Proxy& p: C.proxies) {
        if((frameIDs.contains(p.a->ID) && shapes2.contains(p.b->ID)) ||
            (frameIDs.contains(p.b->ID) && shapes2.contains(p.a->ID))) {
          C.kinematicsProxyCost(y, J, p, margin, true);
//          p.colorCode = 4;
        }
      }
      break;
    case TMT_pairsP: {
      frameIDs.reshape(frameIDs.N/2, 2);
      // only explicit paris in 2D array shapes
      uint j;
      for(const rai::Proxy& p: C.proxies) {
        for(j=0; j<frameIDs.d0; j++) {
          if((frameIDs(j, 0)==p.a->ID && frameIDs(j, 1)==p.b->ID) || (frameIDs(j, 0)==p.b->ID && frameIDs(j, 1)==p.a->ID))
            break;
        }
        if(j<frameIDs.d0) { //if a pair was found
          C.kinematicsProxyCost(y, J, p, margin, true);
//          p.colorCode = 5;
        }
      }
    } break;
    case TMT_allExceptPairsP: {
      frameIDs.reshape(frameIDs.N/2, 2);
      // only explicit paris in 2D array shapes
      uint j;
      for(const rai::Proxy& p: C.proxies) {
        for(j=0; j<frameIDs.d0; j++) {
          if((frameIDs(j, 0)==p.a->ID && frameIDs(j, 1)==p.b->ID) || (frameIDs(j, 0)==p.b->ID && frameIDs(j, 1)==p.a->ID))
            break;
        }
        if(j==frameIDs.d0) { //if a pair was not found
          C.kinematicsProxyCost(y, J, p, margin, true);
//          p.colorCode = 5;
        }
      }
    } break;
    case TMT_vectorP: {
      //outputs a vector of collision meassures, with entry for each explicit pair
      frameIDs.reshape(frameIDs.N/2, 2);
      y.resize(frameIDs.d0, 1);  y.setZero();
      if(!!J) { J.resize(frameIDs.d0, J.d1);  J.setZero(); }
      uint j;
      for(const rai::Proxy& p: C.proxies) {
        for(j=0; j<frameIDs.d0; j++) {
          if((frameIDs(j, 0)==p.a->ID && frameIDs(j, 1)==p.b->ID) || (frameIDs(j, 0)==p.b->ID && frameIDs(j, 1)==p.a->ID))
            break;
        }
        if(j<frameIDs.d0) {
          C.kinematicsProxyCost(y[j](), J[j](), p, margin, true);
//          p.colorCode = 5;
        }
      }
      y.reshape(frameIDs.d0);
    } break;
    default: NIY;
  }
}

uint TM_proxy::dim_phi(const rai::Configuration& G) {
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
      return frameIDs.d0;
    default: NIY;
  }
}

//===========================================================================
