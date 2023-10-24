rai::Frame* rai::Configuration::addFile(const char* filename, const char* parentOfRoot, const rai::Transformation& relOfRoot) {
  rai::Frame* f = addFile(filename);
  if(parentOfRoot) {
    CHECK(f, "nothing added?");
    f->linkFrom(getFrameByName(parentOfRoot));
    new rai::Joint(*f, rai::JT_rigid);
    f->set_Q() = relOfRoot;
  }
//  calc_activeSets();
//  calc_fwdPropagateFrames();
  return f;
}

/// The vector vec1, attached to b1, relative to the frame of b2
void rai::Configuration::kinematicsRelVec(arr& y, arr& J, Frame* a, const rai::Vector& vec1, Frame* b) const {
  arr y1, J1;
  a->ensure_X();
  b->ensure_X();
  kinematicsVec(y1, J1, a, vec1);
  //  kinematicsVec(y2, J2, b2, vec2);
  arr Rinv = ~(b->ensure_X().rot.getArr());
  y = Rinv * y1;
  if(!!J) {
    arr A;
    jacobian_angular(A, b);
    J = Rinv * (J1 - crossProduct(A, y1));
  }
}

#if 0
/// The position vec1, attached to b1, relative to the frame of b2 (plus vec2)
void rai::Configuration::kinematicsRelRot(arr& y, arr& J, Frame* a, Frame* b) const {
  rai::Quaternion rot_b = a->X.rot;
  if(!!y) y = conv_vec2arr(rot_b.getVec());
  if(!!J) {
    double phi=acos(rot_b.w);
    double s=2.*phi/sin(phi);
    double ss=-2./(1.-rai::sqr(rot_b.w)) * (1.-phi/tan(phi));
    arr A;
    axesMatrix(A, a, useSparseJacobians);
    J = 0.5 * (rot_b.w*A*s + crossProduct(A, y));
    J -= 0.5 * ss/s/s*(y*~y*A);
  }
}
#endif
void rai::Configuration::kinematicsContactPOA(arr& y, arr& J, const rai::ForceExchange* c) const {
  kinematicsZero(y, J, 3);

  y = c->poa;
  if(!!J) {
    for(uint i=0; i<3; i++) J.elem(i, c->qIndex+i) = 1.;
  }
}

void rai::Configuration::kinematicsContactForce(arr& y, arr& J, const ForceExchange* c) const {
  y = c->force;

  if(!!J) {
    if(jacMode==JM_dense) {
      J.resize(3, q.N).setZero();
    } else if(jacMode==JM_sparse){
      J.sparse().resize(3, q.N, 0);
    } else if(jacMode==JM_noArr){
      J.setNoArr();
      return;
    }

    for(uint i=0; i<3; i++) J.elem(i, c->qIndex+3+i) = 1.;
  }
}

void rai::Configuration::kinematicsLimits(arr& y, arr& J, const arr& limits) const {
  kinematicsZero(y, J, 1);
//  y.resize(1).setZero();
//  if(!!J) J.resize(1, getJointStateDimension()).setZero();
  double d;
  for(uint i=0; i<limits.d0; i++) if(limits(i, 1)>limits(i, 0)) { //only consider proper limits (non-zero interval)
      d = limits(i, 0) - q(i); //lo
      if(d>0.) {  y.elem(0) += d;  if(!!J) J.elem(0, i)-=1.;  }
      d = q(i) - limits(i, 1); //up
      if(d>0.) {  y.elem(0) += d;  if(!!J) J.elem(0, i)+=1.;  }
  }
}

FrameL Configuration::getParts() const {
  FrameL F;
  for(Frame* f:frames) if(f->isPart()) F.append(f);
  return F;
}

uint Configuration::kinematicsJoints(arr& y, arr& J, const FrameL& F, bool relative_q0) const {
  CHECK_EQ(F.nd, 1, "");
  uint m=0;
  for(Frame *f: F){
    Joint *j = f->joint;
    CHECK(j, "selected frame " <<*f <<" ('" <<f->name <<"') is not a joint");
    m += j->qDim();
  }
  if(!y) return m;

  kinematicsZero(y, J, m);

  m=0;
  for(Frame *f: F){
    Joint *j = f->joint;
    for(uint k=0; k<j->dim; k++) {
      if(j->active){
        y.elem(m) = q.elem(j->qIndex+k);
      }else{
        y.elem(m) = qInactive.elem(j->qIndex+k);
      }
//      if(flipSign) q.elem(m) *= -1.;
      if(relative_q0 && j->q0.N) y.elem(m) -= j->q0(k);
      if(j->active) {
//        if(flipSign) J.elem(m, j->qIndex+k) = -1.; else
        J.elem(m, j->qIndex+k) = 1.;
      }
      m++;
    }
  }

  CHECK_EQ(y.N, m, "");
  return m;
}

