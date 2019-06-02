/*  ------------------------------------------------------------------
    Copyright (c) 2017 Marc Toussaint
    email: marc.toussaint@informatik.uni-stuttgart.de

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
  NULL,
};

TM_Default::TM_Default(TM_DefaultType _type,
                       int iShape, const rai::Vector& _ivec,
                       int jShape, const rai::Vector& _jvec)
  :type(_type), i(iShape), j(jShape) {
  if(!!_ivec) ivec=_ivec; else ivec.setZero();
  if(!!_jvec) jvec=_jvec; else jvec.setZero();
  if(type==TMT_quat) flipTargetSignOnNegScalarProduct=true;
}

TM_Default::TM_Default(TM_DefaultType _type, const rai::KinematicWorld &K,
                       const char* iShapeName, const rai::Vector& _ivec,
                       const char* jShapeName, const rai::Vector& _jvec)
  : TM_Default(_type, initIdArg(K, iShapeName), _ivec, initIdArg(K, jShapeName), _jvec) {
}

TM_Default::TM_Default(const Graph& specs, const rai::KinematicWorld& G)
  :type(TMT_no), i(-1), j(-1) {
  Node *it=specs["type"];
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
  if((it=specs["sym2"]) || (it=specs["ref1"])) { auto name=it->get<rai::String>(); auto *s=G.getFrameByName(name); CHECK(s,"shape name '" <<name <<"' does not exist"); i=s->ID; }
  if((it=specs["sym3"]) || (it=specs["ref2"])) { auto name=it->get<rai::String>(); auto *s=G.getFrameByName(name); CHECK(s,"shape name '" <<name <<"' does not exist"); j=s->ID; }
  if((it=specs["vec1"])) ivec = rai::Vector(it->get<arr>());  else ivec.setZero();
  if((it=specs["vec2"])) jvec = rai::Vector(it->get<arr>());  else jvec.setZero();
  if(type==TMT_quat) flipTargetSignOnNegScalarProduct=true;
}

TM_Default::TM_Default(const Node *specs, const rai::KinematicWorld& G)
  :type(TMT_no), i(-1), j(-1) {
  CHECK(specs->parents.N>1,"");
  //  rai::String& tt=specs->parents(0)->keys.last();
  rai::String& Type=specs->parents(1)->keys.last();
  const char *ref1=NULL, *ref2=NULL;
  if(specs->parents.N>2) ref1=specs->parents(2)->keys.last().p;
  if(specs->parents.N>3) ref2=specs->parents(3)->keys.last().p;
  if(Type=="pos") type=TMT_pos;
  else if(Type=="vec") type=TMT_vec;
  else if(Type=="quat") type=TMT_quat;
  else if(Type=="posDiff") type=TMT_posDiff;
  else if(Type=="vecDiff") type=TMT_vecDiff;
  else if(Type=="quatDiff") type=TMT_quatDiff;
  else if(Type=="vecAlign") type=TMT_vecAlign;
  else if(Type=="gazeAt") type=TMT_gazeAt;
  else HALT("unknown type " <<Type);
  if(ref1) { rai::Frame *s=G.getFrameByName(ref1); CHECK(s,"shape name '" <<ref1 <<"' does not exist"); i=s->ID; }
  if(ref2) { rai::Frame *s=G.getFrameByName(ref2); CHECK(s,"shape name '" <<ref2 <<"' does not exist"); j=s->ID; }
  if(specs->isGraph()) {
    const Graph& params = specs->graph();
    Node *it;
    if((it=params.getNode("vec1"))) ivec = rai::Vector(it->get<arr>());  else ivec.setZero();
    if((it=params.getNode("vec2"))) jvec = rai::Vector(it->get<arr>());  else jvec.setZero();
  }
  if(type==TMT_quat) flipTargetSignOnNegScalarProduct=true;
}

void TM_Default::phi(arr& y, arr& J, const rai::KinematicWorld& G) {
  rai::Frame *body_i = i<0?NULL: G.frames(i);
  rai::Frame *body_j = j<0?NULL: G.frames(j);
  
  if(type==TMT_pos) {
    rai::Vector vec_i = ivec;
    rai::Vector vec_j = jvec;
    CHECK(body_i,"");
    if(body_j==NULL) { //simple, no j reference
      G.kinematicsPos(y, J, body_i, vec_i);
      y -= conv_vec2arr(vec_j);
    }else{
      rai::Vector pi = body_i->X * vec_i;
      rai::Vector pj = body_j->X * vec_j;
      y = conv_vec2arr(body_j->X.rot / (pi-pj));
      if(!!J) {
        arr Ji, Jj, JRj;
        G.kinematicsPos(NoArr, Ji, body_i, vec_i);
        G.kinematicsPos(NoArr, Jj, body_j, vec_j);
        G.axesMatrix(JRj, body_j);
        J.resize(3, Jj.d1);
        for(uint k=0; k<Jj.d1; k++) {
          rai::Vector vi(Ji(0, k), Ji(1, k), Ji(2, k));
          rai::Vector vj(Jj(0, k), Jj(1, k), Jj(2, k));
          rai::Vector r(JRj(0, k), JRj(1, k), JRj(2, k));
          rai::Vector jk =  body_j->X.rot / (vi-vj);
          jk -= body_j->X.rot / (r ^ (pi-pj));
          J(0, k)=jk.x;
          J(1, k)=jk.y;
          J(2, k)=jk.z;
        }
      }
    }
    return;
  }
  
  if(type==TMT_posDiff) {
    rai::Vector vec_i = ivec;
    rai::Vector vec_j = jvec;
    G.kinematicsPos(y, J, body_i, vec_i);
    if(!body_j) { //relative to world
      y -= conv_vec2arr(vec_j);
    }else{
      arr y2, J2;
      G.kinematicsPos(y2, (!!J?J2:NoArr), body_j, vec_j);
      y -= y2;
      if(!!J) J -= J2;
    }
    return;
  }
  
  if(type==TMT_vec) {
    rai::Vector vec_i = ivec;
    //    rai::Vector vec_j = j<0?jvec: G.shapes(j)->rel.rot*jvec;
    if(vec_i.isZero) RAI_MSG("attached vector is zero -- can't control that");
    if(body_j==NULL) { //simple, no j reference
      G.kinematicsVec(y, J, body_i, vec_i);
    }else{
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
    G.kinematicsVec(y, J, body_i, vec_i);
    if(!body_j) { //relative to world
      if(vec_i.isZero) RAI_MSG("attached vector is zero -- can't control that");
      y -= conv_vec2arr(vec_j);
    } else {
      if(vec_i.isZero) RAI_MSG("attached vector1 is zero -- can't control that");
      if(vec_j.isZero) RAI_MSG("attached vector2 is zero -- can't control that");
      arr y2, J2;
      G.kinematicsVec(y2, J2, body_j, vec_j);
      y -= y2;
      if(!!J) J -= J2;
    }
    return;
  }
  
  if(type==TMT_vecAlign) {
    CHECK(fabs(ivec.length()-1.)<1e-4,"vector references must be normalized");
    CHECK(fabs(jvec.length()-1.)<1e-4,"vector references must be normalized");
    rai::Vector vec_i = ivec;
    rai::Vector vec_j = jvec;
    arr zi,Ji,zj,Jj;
    G.kinematicsVec(zi, Ji, body_i, vec_i);
    if(body_j==NULL) {
      zj = conv_vec2arr(vec_j);
      if(!!J) { Jj.resizeAs(Ji); Jj.setZero(); }
    } else {
      G.kinematicsVec(zj, Jj, body_j, vec_j);
    }
    y.resize(1);
    y(0) = scalarProduct(zi, zj);
    if(!!J) {
      J = ~zj * Ji + ~zi * Jj;
      J.reshape(1, G.getJointStateDimension());
    }
    return;
  }
  
  if(type==pos1TMT_D) {
    CHECK(fabs(ivec.length()-1.)<1e-10,"vector references must be normalized");
    arr orientation = conv_vec2arr(ivec);
    G.kinematicsPos(y, NoArr, body_i);
    y = ~orientation*y;
    if(!!J) {
      G.kinematicsPos(NoArr, J, body_i);
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
    arr pi,Jpi, xi,Jxi, yi,Jyi, pj,Jpj;
    G.kinematicsPos(pi, Jpi, body_i, vec_i);
    G.kinematicsVec(xi, Jxi, body_i, Vector_x);
    G.kinematicsVec(yi, Jyi, body_i, Vector_y);
    if(body_j==NULL) { //we look at WORLD
      pj = conv_vec2arr(vec_j);
      if(!!J) { Jpj.resizeAs(Jpi); Jpj.setZero(); }
    } else {
      G.kinematicsPos(pj, Jpj, body_j, vec_j);
    }
    y.resize(2);
    y(0) = scalarProduct(xi, (pj-pi));
    y(1) = scalarProduct(yi, (pj-pi));
    if(!!J) {
      J = cat(~xi * (Jpj-Jpi) + ~(pj-pi) * Jxi,
              ~yi * (Jpj-Jpi) + ~(pj-pi) * Jyi);
      J.reshape(2, G.getJointStateDimension());
    }
    return;
  }
  
  if(type==TMT_quat) {
    if(body_j==NULL) { //simple, no j reference
      G.kinematicsQuat(y, J, body_i);
    }else{
      arr a,b,Ja,Jb;
      G.kinematicsQuat(b, Jb, body_i);
      G.kinematicsQuat(a, Ja, body_j);

      arr Jya, Jyb;
      arr ainv = a;
      if(a(0)!=1.) ainv(0) *= -1.;
      quat_concat(y, Jya, Jyb, ainv, b);
      if(a(0)!=1.) for(uint i=0;i<Jya.d0;i++) Jya(i,0) *= -1.;

      if(!!J){
        J = Jya * Ja + Jyb * Jb;
        checkNan(J);
      }
    }
    return;
  }
  
  if(type==TMT_quatDiff) {
    G.kinematicsQuat(y, J, body_i);
    if(!body_j) { //relative to world
      //diff to world, which is Id
      if(y(0)>=0.) y(0) -= 1.; else y(0) += 1.;
    } else {
      arr y2, J2;
      G.kinematicsQuat(y2, J2, body_j);
      if(scalarProduct(y,y2)>=0.) {
        y -= y2;
        if(!!J) J -= J2;
      } else {
        y += y2;
        if(!!J) J += J2;
      }
    }
    return;
  }

  if(type==TMT_pose) {
    arr yq, Jq;
    TM_Default tmp(*this);
    tmp.type = TMT_pos;
    tmp.phi(y, J, G);
    tmp.type = TMT_quat;
    tmp.phi(yq, (!!J?Jq:NoArr), G);
    y.append(yq);
    if(!!J) J.append(Jq);
    return;
  }

  if(type==TMT_poseDiff) {
    arr yq, Jq;
    TM_Default tmp(*this);
    tmp.type = TMT_posDiff;
    tmp.phi(y, J, G);
    tmp.type = TMT_quatDiff;
    tmp.phi(yq, (!!J?Jq:NoArr), G);
    y.append(yq);
    if(!!J) J.append(Jq);
    return;
  }
  
  HALT("no such TVT");
}

uint TM_Default::dim_phi(const rai::KinematicWorld& G) {
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

rai::String TM_Default::shortTag(const rai::KinematicWorld& K) {
  rai::String s="Default-";
  s <<order;
  s <<'-' <<type;
  s <<'-' <<(i<0?"WORLD":K.frames(i)->name);
  s <<'-' <<(j<0?"WORLD":K.frames(j)->name);
  return s;
}

Graph TM_Default::getSpec(const rai::KinematicWorld& K){
  Graph G;
  G.newNode<rai::String>({"feature"}, {}, STRING(type));
  if(i>=0) G.newNode<rai::String>({"o1"}, {}, K.frames(i)->name);
  if(j>=0) G.newNode<rai::String>({"o2"}, {}, K.frames(j)->name);
  if(!ivec.isZero) G.newNode<arr>({"v1"}, {}, ivec.getArr());
  if(!jvec.isZero) G.newNode<arr>({"v2"}, {}, jvec.getArr());
  return G;
}

//===========================================================================

