/*  ------------------------------------------------------------------
    Copyright (c) 2011-2020 Marc Toussaint
    email: toussaint@tu-berlin.de

    This code is distributed under the MIT License.
    Please see <root-path>/LICENSE for details.
    --------------------------------------------------------------  */

#include "TM_default.h"
#include "frame.h"

template<> const char* rai::Enum<TM_DefaultType>::names []= {
  "no",      ///< non-initialization
  "pos",     ///< 3D position of reference
  "vec",     ///< 3D vec (orientation)
  "quat",    ///< 4D quaterion
  "posDiff", ///< the difference of two positions (NOT the relative position)
  "vecDiff", ///< the difference of two vectors (NOT the relative position)
  "quatDiff",///< the difference of 2 quaternions (NOT the relative quaternion)
  "vecAlign",///< 1D vector alignment, can have 2nd reference, param (optional) determins alternative reference world vector
  "gazeAt",  ///< 2D orthogonality measure of object relative to camera plane
  "pose",
  "poseDiff",
  "pos1D",
  nullptr,
};

TM_Default::TM_Default(TM_DefaultType _type,
                       int iShape, const rai::Vector& _ivec,
                       int jShape, const rai::Vector& _jvec)
  :type(_type), i(iShape), j(jShape) {
  if(!!_ivec) ivec=_ivec; else ivec.setZero();
  if(!!_jvec) jvec=_jvec; else jvec.setZero();
  if(type==TMT_quat) flipTargetSignOnNegScalarProduct=true;
}

TM_Default::TM_Default(TM_DefaultType _type, const rai::Configuration& K,
                       const char* iShapeName, const rai::Vector& _ivec,
                       const char* jShapeName, const rai::Vector& _jvec)
  : TM_Default(_type, initIdArg(K, iShapeName), _ivec, initIdArg(K, jShapeName), _jvec) {
}

TM_Default::TM_Default(const rai::Graph& specs, const rai::Configuration& G)
  :type(TMT_no), i(-1), j(-1) {
  rai::Node* it=specs["type"];
  if(!it) it=specs["map"];
  if(!it) HALT("no type given");
  rai::String Type=it->get<rai::String>();
  if(Type=="pos") type=TMT_pos;
  else if(Type=="vec") type=TMT_vec;
  else if(Type=="quat") type=TMT_quat;
  else if(Type=="posDiff") type=TMT_posDiff;
  else if(Type=="vecDiff") type=TMT_vecDiff;
  else if(Type=="quatDiff") type=TMT_quatDiff;
  else if(Type=="vecAlign") type=TMT_vecAlign;
  else if(Type=="gazeAt") type=TMT_gazeAt;
  else HALT("unknown type " <<Type);
  if((it=specs["sym2"]) || (it=specs["ref1"])) { auto name=it->get<rai::String>(); auto* s=G.getFrame(name); CHECK(s, "shape name '" <<name <<"' does not exist"); i=s->ID; }
  if((it=specs["sym3"]) || (it=specs["ref2"])) { auto name=it->get<rai::String>(); auto* s=G.getFrame(name); CHECK(s, "shape name '" <<name <<"' does not exist"); j=s->ID; }
  if((it=specs["vec1"])) ivec = rai::Vector(it->get<arr>());  else ivec.setZero();
  if((it=specs["vec2"])) jvec = rai::Vector(it->get<arr>());  else jvec.setZero();
  if(type==TMT_quat) flipTargetSignOnNegScalarProduct=true;
}

TM_Default::TM_Default(const rai::Node* specs, const rai::Configuration& G)
  :type(TMT_no), i(-1), j(-1) {
  CHECK(specs->parents.N>1, "");
  //  rai::String& tt=specs->parents(0)->key;
  rai::String& Type=specs->parents(1)->key;
  const char* ref1=nullptr, *ref2=nullptr;
  if(specs->parents.N>2) ref1=specs->parents(2)->key.p;
  if(specs->parents.N>3) ref2=specs->parents(3)->key.p;
  if(Type=="pos") type=TMT_pos;
  else if(Type=="vec") type=TMT_vec;
  else if(Type=="quat") type=TMT_quat;
  else if(Type=="posDiff") type=TMT_posDiff;
  else if(Type=="vecDiff") type=TMT_vecDiff;
  else if(Type=="quatDiff") type=TMT_quatDiff;
  else if(Type=="vecAlign") type=TMT_vecAlign;
  else if(Type=="gazeAt") type=TMT_gazeAt;
  else HALT("unknown type " <<Type);
  if(ref1) { rai::Frame* s=G.getFrame(ref1); CHECK(s, "shape name '" <<ref1 <<"' does not exist"); i=s->ID; }
  if(ref2) { rai::Frame* s=G.getFrame(ref2); CHECK(s, "shape name '" <<ref2 <<"' does not exist"); j=s->ID; }
  if(specs->isGraph()) {
    const rai::Graph& params = specs->graph();
    rai::Node* it;
    if((it=params.getNode("vec1"))) ivec = rai::Vector(it->get<arr>());  else ivec.setZero();
    if((it=params.getNode("vec2"))) jvec = rai::Vector(it->get<arr>());  else jvec.setZero();
  }
  if(type==TMT_quat) flipTargetSignOnNegScalarProduct=true;
}

void TM_Default::phi(arr& y, arr& J, const rai::Configuration& C) {
  rai::Frame* a = i<0?nullptr: C.frames(i);
  rai::Frame* b = j<0?nullptr: C.frames(j);

  if(a) a->ensure_X();
  if(b) b->ensure_X();

  if(type==TMT_pos) {
    rai::Vector vec_i = ivec;
    rai::Vector vec_j = jvec;
    CHECK(a, "");
    if(b==nullptr) { //simple, no j reference
      C.kinematicsPos(y, J, a, vec_i);
      y -= conv_vec2arr(vec_j);
    } else {
      HALT("use F_PositionRel!")
//      C.kinematicsRelPos(y, J, a, vec_i, b, vec_j);
    }
    return;
  }

  if(type==TMT_posDiff) {
    rai::Vector vec_i = ivec;
    rai::Vector vec_j = jvec;
    C.kinematicsPos(y, J, a, vec_i);
    if(!b) { //relative to world
      y -= conv_vec2arr(vec_j);
    } else {
      arr y2, J2;
      C.kinematicsPos(y2, J2, b, vec_j);
      y -= y2;
      if(!!J) J -= J2;
    }
    return;
  }

  if(type==TMT_vec) {
    rai::Vector vec_i = ivec;
    //    rai::Vector vec_j = j<0?jvec: G.shapes(j)->rel.rot*jvec;
    if(vec_i.isZero) RAI_MSG("attached vector is zero -- can't control that");
    if(b==nullptr) { //simple, no j reference
      C.kinematicsVec(y, J, a, vec_i);
    } else {
      //relative
      RAI_MSG("warning - don't have a correct Jacobian for this TMT_ype yet");
      //      fi = G.bodies(body_i)->X; fi.appendTransformation(irel);
      //      fj = G.bodies(body_j)->X; fj.appendTransformation(jrel);
      //      f.setDifference(fi, fj);
      //      f.rot.getZ(c);
      //      y = conv_vec2arr(c);
      NIY; //TODO: Jacobian?
    }
    return;
  }

  if(type==TMT_vecDiff) {
    rai::Vector vec_i = ivec;
    rai::Vector vec_j = jvec;
    C.kinematicsVec(y, J, a, vec_i);
    if(!b) { //relative to world
      if(vec_i.isZero) RAI_MSG("attached vector is zero -- can't control that");
      y -= conv_vec2arr(vec_j);
    } else {
      if(vec_i.isZero) RAI_MSG("attached vector1 is zero -- can't control that");
      if(vec_j.isZero) RAI_MSG("attached vector2 is zero -- can't control that");
      arr y2, J2;
      C.kinematicsVec(y2, J2, b, vec_j);
      y -= y2;
      if(!!J) J -= J2;
    }
    return;
  }

  if(type==TMT_vecAlign) {
    CHECK(fabs(ivec.length()-1.)<1e-4, "vector references must be normalized");
    CHECK(fabs(jvec.length()-1.)<1e-4, "vector references must be normalized");
    rai::Vector vec_i = ivec;
    rai::Vector vec_j = jvec;
    arr zi, Ji, zj, Jj;
    C.kinematicsVec(zi, Ji, a, vec_i);
    if(b==nullptr) {
      zj = conv_vec2arr(vec_j);
      if(!!J) { Jj.resizeAs(Ji); Jj.setZero(); }
    } else {
      C.kinematicsVec(zj, Jj, b, vec_j);
    }
    y.resize(1);
    y(0) = scalarProduct(zi, zj);
    if(!!J) {
      J = ~zj * Ji + ~zi * Jj;
//      J.reshape(1, C.getJointStateDimension());
    }
    return;
  }

  if(type==pos1TMT_D) {
    CHECK(fabs(ivec.length()-1.)<1e-10, "vector references must be normalized");
    arr orientation = conv_vec2arr(ivec);
    C.kinematicsPos(y, NoArr, a);
    y = ~orientation*y;
    if(!!J) {
      C.kinematicsPos(NoArr, J, a);
      J = ~orientation*J;
      J.reshape(1, J.N);
    }
    return;
  }

  if(type==TMT_gazeAt) {
    CHECK_GE(i, 0, "sym2 is not set!");

    // i    := index of shape to look with (i.e. the shape with the camera)
    // ivec := relative position of the camera center
    // j    := index of shape to look at
    // jvec := relative position on the target shape; where in the target shape should we look.
    //         If j is not set, the target shape is WORLD and jvec is a vector in world coordinates

    rai::Vector vec_i = ivec;
    rai::Vector vec_j = jvec;
    arr pi, Jpi, xi, Jxi, yi, Jyi, pj, Jpj;
    C.kinematicsPos(pi, Jpi, a, vec_i);
    C.kinematicsVec(xi, Jxi, a, Vector_x);
    C.kinematicsVec(yi, Jyi, a, Vector_y);
    if(b==nullptr) { //we look at WORLD
      pj = conv_vec2arr(vec_j);
      if(!!J) { Jpj.resizeAs(Jpi); Jpj.setZero(); }
    } else {
      C.kinematicsPos(pj, Jpj, b, vec_j);
    }
    y.resize(2);
    y(0) = scalarProduct(xi, (pj-pi));
    y(1) = scalarProduct(yi, (pj-pi));
    if(!!J) {
      J = cat(~xi * (Jpj-Jpi) + ~(pj-pi) * Jxi,
              ~yi * (Jpj-Jpi) + ~(pj-pi) * Jyi);
      J.reshape(2, C.getJointStateDimension());
    }
    return;
  }

  if(type==TMT_quat) {
    if(b==nullptr) { //simple, no j reference
      C.kinematicsQuat(y, J, a);
    } else {
      arr qa, qb, Ja, Jb;
      C.kinematicsQuat(qb, Jb, a);
      C.kinematicsQuat(qa, Ja, b);

      arr Jya, Jyb;
      arr ainv = qa;
      if(qa(0)!=1.) ainv(0) *= -1.;
      quat_concat(y, Jya, Jyb, ainv, qb);
      if(qa(0)!=1.) for(uint i=0; i<Jya.d0; i++) Jya(i, 0) *= -1.;

      J = Jya * Ja + Jyb * Jb;
      checkNan(J);
    }
    return;
  }

  if(type==TMT_quatDiff) {
    C.kinematicsQuat(y, J, a);
    if(!b) { //relative to world
      //diff to world, which is Id
      if(y(0)>=0.) y(0) -= 1.; else y(0) += 1.;
    } else {
      arr y2, J2;
      C.kinematicsQuat(y2, J2, b);
      if(scalarProduct(y, y2)>=0.) {
        y -= y2;
        J -= J2;
      } else {
        y += y2;
        J += J2;
      }
    }
    return;
  }

  if(type==TMT_pose) {
    arr yq, Jq;
    TM_Default tmp(*this);
    tmp.type = TMT_pos;
    tmp.phi(y, J, C);
    tmp.type = TMT_quat;
    tmp.phi(yq, Jq, C);
    y.append(yq);
    if(!!J) J.append(Jq);
    return;
  }

  if(type==TMT_poseDiff) {
    arr yq, Jq;
    TM_Default tmp(*this);
    tmp.type = TMT_posDiff;
    tmp.phi(y, J, C);
    tmp.type = TMT_quatDiff;
    tmp.phi(yq, Jq, C);
    y.append(yq);
    if(!!J) J.append(Jq);
    return;
  }

  HALT("no such TVT");
}

uint TM_Default::dim_phi(const rai::Configuration& C) {
  switch(type) {
    case TMT_pos: return 3;
    case TMT_vec: return 3;
    case TMT_quat: return 4;
    case TMT_posDiff: return 3;
    case TMT_vecDiff: return 3;
    case TMT_quatDiff: return 4;
    case TMT_vecAlign: return 1;
    case TMT_gazeAt: return 2;
    case TMT_pose: return 7;
    case TMT_poseDiff: return 7;
    case pos1TMT_D: return 1;
    default:  HALT("no such TMT_");
  }
}

void TM_Default::signature(intA& S, const rai::Configuration& C) {
  rai::Frame* a = i<0?nullptr: C.frames(i);
  rai::Frame* b = j<0?nullptr: C.frames(j);

  S.clear();
  FrameL F;
  if(a) F.append(a->getPathToRoot());
  if(b) F.append(b->getPathToRoot());

  for(rai::Frame* f:F) if(f->joint) {
      rai::Joint* j = f->joint;
      for(uint i=0; i<j->qDim(); i++) S.setAppendInSorted(j->qIndex+i);
    }
}

rai::String TM_Default::shortTag(const rai::Configuration& C) {
  rai::String s="Default-";
  s <<order;
  s <<'-' <<type;
  s <<'-' <<(i<0?"WORLD":C.frames(i)->name);
  s <<'-' <<(j<0?"WORLD":C.frames(j)->name);
  return s;
}

rai::Graph TM_Default::getSpec(const rai::Configuration& C) {
  rai::Graph G;
  G.newNode<rai::String>({"feature"}, {}, STRING(type));
  if(i>=0) G.newNode<rai::String>({"o1"}, {}, C.frames(i)->name);
  if(j>=0) G.newNode<rai::String>({"o2"}, {}, C.frames(j)->name);
  if(!ivec.isZero) G.newNode<arr>({"v1"}, {}, ivec.getArr());
  if(!jvec.isZero) G.newNode<arr>({"v2"}, {}, jvec.getArr());
  return G;
}

//===========================================================================

