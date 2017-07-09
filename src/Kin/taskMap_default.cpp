/*  ------------------------------------------------------------------
    Copyright 2016 Marc Toussaint
    email: marc.toussaint@informatik.uni-stuttgart.de
    
    This program is free software: you can redistribute it and/or modify
    it under the terms of the GNU General Public License as published by
    the Free Software Foundation, either version 3 of the License, or (at
    your option) any later version. This program is distributed without
    any warranty. See the GNU General Public License for more details.
    You should have received a COPYING file of the full GNU General Public
    License along with this program. If not, see
    <http://www.gnu.org/licenses/>
    --------------------------------------------------------------  */

#include "taskMap_default.h"

const char* TaskMap_DefaultType2String[] = {
"no",      ///< non-initialization
"pos",     ///< 3D position of reference
"vec",     ///< 3D vec (orientation)
"quat",    ///< 4D quaterion
"posDiff", ///< the difference of two positions (NOT the relative position)
"vecDiff", ///< the difference of two vectors (NOT the relative position)
"quatDiff",///< the difference of 2 quaternions (NOT the relative quaternion)
"vecAlign",///< 1D vector alignment, can have 2nd reference, param (optional) determins alternative reference world vector
"gazeAt",  ///< 2D orthogonality measure of object relative to camera plane
"pos1D"
};

TaskMap_Default::TaskMap_Default(TaskMap_DefaultType _type,
                               int iShape, const mlr::Vector& _ivec,
                               int jShape, const mlr::Vector& _jvec)
  :type(_type), i(iShape), j(jShape){
  if(&_ivec) ivec=_ivec; else ivec.setZero();
  if(&_jvec) jvec=_jvec; else jvec.setZero();
}

TaskMap_Default::TaskMap_Default(TaskMap_DefaultType _type, const mlr::KinematicWorld &G,
                               const char* iShapeName, const mlr::Vector& _ivec,
                               const char* jShapeName, const mlr::Vector& _jvec)
  :type(_type), i(-1), j(-1){
  mlr::Shape *a = iShapeName ? G.getShapeByName(iShapeName):NULL;
  mlr::Shape *b = jShapeName ? G.getShapeByName(jShapeName):NULL;
  if(a) i=a->index;
  if(b) j=b->index;
  if(&_ivec) ivec=_ivec; else ivec.setZero();
  if(&_jvec) jvec=_jvec; else jvec.setZero();
}

TaskMap_Default::TaskMap_Default(const Graph& specs, const mlr::KinematicWorld& G)
  :type(noTMT), i(-1), j(-1){
  Node *it=specs["type"];
  if(!it) it=specs["map"];
  if(!it) HALT("no type given");
  mlr::String Type=it->get<mlr::String>();
  if(Type=="pos") type=posTMT;
  else if(Type=="vec") type=vecTMT;
  else if(Type=="quat") type=quatTMT;
  else if(Type=="posDiff") type=posDiffTMT;
  else if(Type=="vecDiff") type=vecDiffTMT;
  else if(Type=="quatDiff") type=quatDiffTMT;
  else if(Type=="vecAlign") type=vecAlignTMT;
  else if(Type=="gazeAt") type=gazeAtTMT;
  else HALT("unknown type " <<Type);
  if((it=specs["sym2"]) || (it=specs["ref1"])){ auto name=it->get<mlr::String>(); auto *s=G.getShapeByName(name); CHECK(s,"shape name '" <<name <<"' does not exist"); i=s->index; }
  if((it=specs["sym3"]) || (it=specs["ref2"])){ auto name=it->get<mlr::String>(); auto *s=G.getShapeByName(name); CHECK(s,"shape name '" <<name <<"' does not exist"); j=s->index; }
  if((it=specs["vec1"])) ivec = mlr::Vector(it->get<arr>());  else ivec.setZero();
  if((it=specs["vec2"])) jvec = mlr::Vector(it->get<arr>());  else jvec.setZero();
}

TaskMap_Default::TaskMap_Default(const Node *specs, const mlr::KinematicWorld& G)
  :type(noTMT), i(-1), j(-1){
  CHECK(specs->parents.N>1,"");
//  mlr::String& tt=specs->parents(0)->keys.last();
  mlr::String& Type=specs->parents(1)->keys.last();
  const char *ref1=NULL, *ref2=NULL;
  if(specs->parents.N>2) ref1=specs->parents(2)->keys.last().p;
  if(specs->parents.N>3) ref2=specs->parents(3)->keys.last().p;
       if(Type=="pos") type=posTMT;
  else if(Type=="vec") type=vecTMT;
  else if(Type=="quat") type=quatTMT;
  else if(Type=="posDiff") type=posDiffTMT;
  else if(Type=="vecDiff") type=vecDiffTMT;
  else if(Type=="quatDiff") type=quatDiffTMT;
  else if(Type=="vecAlign") type=vecAlignTMT;
  else if(Type=="gazeAt") type=gazeAtTMT;
  else HALT("unknown type " <<Type);
  if(ref1){ mlr::Shape *s=G.getShapeByName(ref1); CHECK(s,"shape name '" <<ref1 <<"' does not exist"); i=s->index; }
  if(ref2){ mlr::Shape *s=G.getShapeByName(ref2); CHECK(s,"shape name '" <<ref2 <<"' does not exist"); j=s->index; }
  if(specs->isGraph()){
    const Graph& params = specs->graph();
    Node *it;
    if((it=params.getNode("vec1"))) ivec = mlr::Vector(it->get<arr>());  else ivec.setZero();
    if((it=params.getNode("vec2"))) jvec = mlr::Vector(it->get<arr>());  else jvec.setZero();
  }
}


void TaskMap_Default::phi(arr& y, arr& J, const mlr::KinematicWorld& G, int t) {
  if(t>=0 && referenceIds.N){
    if(referenceIds.nd==1){  i=referenceIds(t); j=-1; }
    if(referenceIds.nd==2){  i=referenceIds(t,0); j=referenceIds(t,1); }
  }

  mlr::Body *body_i = i<0?NULL: G.shapes(i)->body;
  mlr::Body *body_j = j<0?NULL: G.shapes(j)->body;

  if(type==posTMT){
    mlr::Vector vec_i = i<0?ivec: G.shapes(i)->rel*ivec;
    mlr::Vector vec_j = j<0?jvec: G.shapes(j)->rel*jvec;
    CHECK(body_i,"");
    if(body_j==NULL) { //simple, no j reference
      G.kinematicsPos(y, J, body_i, vec_i);
      y -= conv_vec2arr(vec_j);
      return;
    }//else...
    mlr::Vector pi = body_i->X * vec_i;
    mlr::Vector pj = body_j->X * vec_j;
    y = conv_vec2arr(body_j->X.rot / (pi-pj));
    if(&J) {
      arr Ji, Jj, JRj;
      G.kinematicsPos(NoArr, Ji, body_i, vec_i);
      G.kinematicsPos(NoArr, Jj, body_j, vec_j);
      G.axesMatrix(JRj, body_j);
      J.resize(3, Jj.d1);
      for(uint k=0; k<Jj.d1; k++) {
        mlr::Vector vi(Ji(0, k), Ji(1, k), Ji(2, k));
        mlr::Vector vj(Jj(0, k), Jj(1, k), Jj(2, k));
        mlr::Vector r (JRj(0, k), JRj(1, k), JRj(2, k));
        mlr::Vector jk =  body_j->X.rot / (vi-vj);
        jk -= body_j->X.rot / (r ^ (pi-pj));
        J(0, k)=jk.x;
        J(1, k)=jk.y;
        J(2, k)=jk.z;
      }
    }
    return;
  }

  if(type==posDiffTMT){
    mlr::Vector vec_i = i<0?ivec: G.shapes(i)->rel*ivec;
    mlr::Vector vec_j = j<0?jvec: G.shapes(j)->rel*jvec;
    G.kinematicsPos(y, J, body_i, vec_i);
    if(!body_j){ //relative to world
      y -= conv_vec2arr(vec_j);
    }else{
      arr y2, J2;
      G.kinematicsPos(y2, (&J?J2:NoArr), body_j, vec_j);
      y -= y2;
      if(&J) J -= J2;
    }
    return;
  }

  if(type==vecTMT){
    mlr::Vector vec_i = i<0?ivec: G.shapes(i)->rel.rot*ivec;
//    mlr::Vector vec_j = j<0?jvec: G.shapes(j)->rel.rot*jvec;
    if(vec_i.isZero) MLR_MSG("attached vector is zero -- can't control that");
    if(body_j==NULL) { //simple, no j reference
      G.kinematicsVec(y, J, body_i, vec_i);
      return;
    }//else...
    //relative
    MLR_MSG("warning - don't have a correct Jacobian for this TMType yet");
    //      fi = G.bodies(body_i)->X; fi.appendTransformation(irel);
    //      fj = G.bodies(body_j)->X; fj.appendTransformation(jrel);
    //      f.setDifference(fi, fj);
    //      f.rot.getZ(c);
    //      y = conv_vec2arr(c);
    NIY; //TODO: Jacobian?
    return;
  }

  if(type==vecDiffTMT){
    mlr::Vector vec_i = i<0?ivec: G.shapes(i)->rel.rot*ivec;
    mlr::Vector vec_j = j<0?jvec: G.shapes(j)->rel.rot*jvec;
    G.kinematicsVec(y, J, body_i, vec_i);
    if(!body_j){ //relative to world
      if(vec_i.isZero) MLR_MSG("attached vector is zero -- can't control that");
      y -= conv_vec2arr(vec_j);
    }else{
      if(vec_i.isZero) MLR_MSG("attached vector1 is zero -- can't control that");
      if(vec_j.isZero) MLR_MSG("attached vector2 is zero -- can't control that");
      arr y2, J2;
      G.kinematicsVec(y2, J2, body_j, vec_j);
      y -= y2;
      J -= J2;
    }
    return;
  }

  if(type==vecAlignTMT) {
    CHECK(fabs(ivec.length()-1.)<1e-4,"vector references must be normalized");
    CHECK(fabs(jvec.length()-1.)<1e-4,"vector references must be normalized");
    mlr::Vector vec_i = i<0?ivec: G.shapes(i)->rel.rot*ivec;
    mlr::Vector vec_j = j<0?jvec: G.shapes(j)->rel.rot*jvec;
    arr zi,Ji,zj,Jj;
    G.kinematicsVec(zi, Ji, body_i, vec_i);
    if(body_j==NULL) {
      zj = conv_vec2arr(vec_j);
      if(&J) { Jj.resizeAs(Ji); Jj.setZero(); }
    } else {
      G.kinematicsVec(zj, Jj, body_j, vec_j);
    }
    y.resize(1);
    y(0) = scalarProduct(zi, zj);
    if(&J) {
      J = ~zj * Ji + ~zi * Jj;
      J.reshape(1, G.getJointStateDimension());
    }
    return;
  }

  if(type==pos1DTMT) {
    CHECK(fabs(ivec.length()-1.)<1e-10,"vector references must be normalized");
    arr orientation = conv_vec2arr(ivec);
    G.kinematicsPos(y, NoArr, body_i);
    y = ~orientation*y;
    if(&J) {
      G.kinematicsPos(NoArr, J, body_i);
      J = ~orientation*J;
      J.reshape(1, J.N);
    }
    return;
  }

  if(type==gazeAtTMT){
    CHECK(i>=0, "sym2 is not set!");

    // i    := index of shape to look with (i.e. the shape with the camera)
    // ivec := relative position of the camera center
    // j    := index of shape to look at
    // jvec := relative position on the target shape; where in the target shape should we look.
    //         If j is not set, the target shape is WORLD and jvec is a vector in world coordinates

    mlr::Vector vec_i = G.shapes(i)->rel*ivec;
    mlr::Vector vec_xi = G.shapes(i)->rel.rot*Vector_x;
    mlr::Vector vec_yi = G.shapes(i)->rel.rot*Vector_y;
    mlr::Vector vec_j = j<0?jvec: G.shapes(j)->rel*jvec;
    arr pi,Jpi, xi,Jxi, yi,Jyi, pj,Jpj;
    G.kinematicsPos(pi, Jpi, body_i, vec_i);
    G.kinematicsVec(xi, Jxi, body_i, vec_xi);
    G.kinematicsVec(yi, Jyi, body_i, vec_yi);
    if(body_j==NULL) { //we look at WORLD
      pj = conv_vec2arr(vec_j);
      if(&J) { Jpj.resizeAs(Jpi); Jpj.setZero(); }
    } else {
      G.kinematicsPos(pj, Jpj, body_j, vec_j);
    }
    y.resize(2);
    y(0) = scalarProduct(xi, (pj-pi));
    y(1) = scalarProduct(yi, (pj-pi));
    if(&J) {
      J = cat( ~xi * (Jpj-Jpi) + ~(pj-pi) * Jxi,
               ~yi * (Jpj-Jpi) + ~(pj-pi) * Jyi );
      J.reshape(2, G.getJointStateDimension());
    }
    return;
  }

  if(type==quatTMT){
    if(body_j==NULL) { //simple, no j reference
      G.kinematicsQuat(y, J, body_i);
      return;
    }//else...
    NIY;
  }

  if(type==quatDiffTMT){
    G.kinematicsQuat(y, J, body_i);
    if(!body_j){ //relative to world
       //diff to world, which is Id
      if(y(0)>=0.) y(0) -= 1.; else y(0) += 1.;
    }else{
      arr y2, J2;
      G.kinematicsQuat(y2, J2, body_j);
      if(scalarProduct(y,y2)>=0.){
        y -= y2;
        if(&J) J -= J2;
      }else{
        y += y2;
        if(&J) J += J2;
      }
    }
    return;
  }

  if(type==poseDiffTMT){
    arr yq, Jq;
    TaskMap_Default tmp(*this);
    tmp.type = posDiffTMT;
    tmp.phi(y, J, G);
    tmp.type = quatDiffTMT;
    tmp.phi(yq, (&J?Jq:NoArr), G);
    y.append(yq);
    if(&J) J.append(Jq);
    return;
  }

  HALT("no such TVT");
}

uint TaskMap_Default::dim_phi(const mlr::KinematicWorld& G) {
  switch(type) {
    case posTMT: return 3;
    case vecTMT: return 3;
    case quatTMT: return 4;
    case posDiffTMT: return 3;
    case vecDiffTMT: return 3;
    case quatDiffTMT: return 4;
    case vecAlignTMT: return 1;
    case gazeAtTMT: return 2;
    case pos1DTMT: return 1;
    default:  HALT("no such TMT");
  }
}

mlr::String TaskMap_Default::shortTag(const mlr::KinematicWorld& G){
  mlr::String s="Default";
  s <<':' <<TaskMap_DefaultType2String[type];
  s <<':' <<(i<0?"WORLD":G.shapes(i)->name);
  s <<'/' <<(j<0?"WORLD":G.shapes(j)->name);
  return s;
}


//===========================================================================

