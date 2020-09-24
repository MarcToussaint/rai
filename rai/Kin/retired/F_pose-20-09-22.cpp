//===========================================================================

struct TM_Align : Feature {
  int i, j;               ///< which shapes does it refer to?

  TM_Align(const rai::Configuration& G, const char* iName=nullptr, const char* jName=nullptr);

  virtual void phi(arr& y, arr& J, const rai::Configuration& G);
  virtual uint dim_phi(const rai::Configuration& G) { return 3; }
};

//===========================================================================

TM_Align::TM_Align(const rai::Configuration& K, const char* iName, const char* jName)
  : i(-1), j(-1) {
  rai::Frame* a = iName ? K.getFrameByName(iName):nullptr;
  rai::Frame* b = jName ? K.getFrameByName(jName):nullptr;
  if(a) i=a->ID;
  if(b) j=b->ID;
}

void TM_Align::phi(arr& y, arr& J, const rai::Configuration& K) {
  y.resize(3);
  if(!!J) J.resize(3, K.q.N);

  rai::Frame* body_i = K.frames(i);
  rai::Frame* body_j = K.frames(j);

  arr zi, Ji, zj, Jj;

  K.kinematicsVec(zi, Ji, body_i, Vector_z);
  K.kinematicsVec(zj, Jj, body_j, Vector_x);
  y(0) = scalarProduct(zi, zj);
  if(!!J) J[0] = ~zj * Ji + ~zi * Jj;

  K.kinematicsVec(zi, Ji, body_i, Vector_z);
  K.kinematicsVec(zj, Jj, body_j, Vector_y);
  y(1) = scalarProduct(zi, zj);
  if(!!J) J[1] = ~zj * Ji + ~zi * Jj;

  K.kinematicsVec(zi, Ji, body_i, Vector_y);
  K.kinematicsVec(zj, Jj, body_j, Vector_x);
  y(2) = scalarProduct(zi, zj);
  if(!!J) J[2] = ~zj * Ji + ~zi * Jj;
}

//===========================================================================




