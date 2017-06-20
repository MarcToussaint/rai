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


#include "taskMap.h"
#include "taskMap_qItself.h"
#include "taskMap_GJK.h"
#include "taskMap_FixSwitchedObjects.h"

//===========================================================================

void TaskMap::phi(arr& y, arr& J, const WorldL& G, double tau, int t){
  CHECK(G.N>=order+1,"I need at least " <<order+1 <<" configurations to evaluate");
  uint k=order;
  if(k==0){// basic case: order=0
    arr J_bar;
    phi(y, (&J?J_bar:NoArr), *G.last(), t);
    if(&J){
      uint qidx=0;
      for(uint i=0;i<G.N;i++) qidx+=G(i)->q.N;
      J.resize(y.N, qidx).setZero();
      J.setMatrixBlock(J_bar, 0, qidx-J_bar.d1);
    }
    return;
  }
  arrA y_bar, J_bar;
  double tau2=tau*tau, tau3=tau2*tau;
  y_bar.resize(k+1);
  J_bar.resize(k+1);
  //-- read out the task variable from the k+1 configurations
  uint offset = G.N-1-k; //G.N might contain more configurations than the order of THIS particular task -> the front ones are not used
  for(uint i=0;i<=k;i++)
    phi(y_bar(i), (&J?J_bar(i):NoArr), *G(offset+i), t-k+i);
 
  // check for quaternion flipping
  if(k==1 && flipTargetSignOnNegScalarProduct && scalarProduct(y_bar(1), y_bar(0))<0.){
      if(&J) J_bar(0) = -J_bar(0);
      y_bar(0)*=-1.;
  }
  // NIY
  if(k==2 && flipTargetSignOnNegScalarProduct) HALT("Quaternion flipping NIY for acceleration");
  if(k==3 && flipTargetSignOnNegScalarProduct) HALT("Quaternion flipping NIY for jerk");

  if(k==1)  y = (y_bar(1)-y_bar(0))/tau; //penalize velocity
  if(k==2)  y = (y_bar(2)-2.*y_bar(1)+y_bar(0))/tau2; //penalize acceleration
  if(k==3)  y = (y_bar(3)-3.*y_bar(2)+3.*y_bar(1)-y_bar(0))/tau3; //penalize jerk
  if(&J) {
#if 1
    uintA qidx(G.N);
    qidx(0)=0;
    for(uint i=1;i<G.N;i++) qidx(i) = qidx(i-1)+G(i-1)->q.N;
    J = zeros(y.N, qidx.last()+G.last()->q.N);
    if(k==1){ J.setMatrixBlock(J_bar(1), 0, qidx(offset+1));  J.setMatrixBlock(   -J_bar(0), 0, qidx(offset+0));  J/=tau; }
    if(k==2){ J.setMatrixBlock(J_bar(2), 0, qidx(offset+2));  J.setMatrixBlock(-2.*J_bar(1), 0, qidx(offset+1));  J.setMatrixBlock(J_bar(0)   , 0, qidx(offset+0));  J/=tau2; }
    if(k==3){ J.setMatrixBlock(J_bar(3), 0, qidx(offset+3));  J.setMatrixBlock(-3.*J_bar(2), 0, qidx(offset+2));  J.setMatrixBlock(3.*J_bar(1), 0, qidx(offset+1));  J.setMatrixBlock(-J_bar(0), 0, qidx(offset+0));  J/=tau3; }
#else
    J = zeros(G.N, y.N, J_bar(0).d1);
    if(k==1){ J[offset+1]() =  J_bar(1);  J[offset+0]() =    -J_bar(0);  J/=tau; }
    if(k==2){ J[offset+2]() =  J_bar(2);  J[offset+1]() = -2.*J_bar(1);  J[offset+0]() = J_bar(0);  J/=tau2; }
    if(k==3){ J[offset+3]() =  J_bar(3);  J[offset+2]() = -3.*J_bar(2);  J[offset+1]() = 3.*J_bar(1);  J[offset+0]() = -J_bar(0);  J/=tau3; }
    arr tmp(J);
    tensorPermutation(J, tmp, TUP(1u,0u,2u));
    J.reshape(y.N, G.N*J_bar(0).d1);
#endif
  }
}

//===========================================================================

TaskMap *TaskMap::newTaskMap(const Graph& params, const mlr::KinematicWorld& world){
  //-- get tags
  mlr::String type = params.get<mlr::String>("map", "default");

  //-- create a task map
  TaskMap *map;
  if(type=="wheels"){
    map = new TaskMap_qItself(QIP_byJointNames, {"worldTranslationRotation"}, world);
  }else if(type=="collisionIneq"){
    map = new CollisionConstraint( params.get<double>("margin", 0.1) );
  }else if(type=="limitIneq"){
    map = new LimitsConstraint();
  }else if(type=="proxy"){
    map = new TaskMap_Proxy(allPTMT, {0u}, params.get<double>("margin", 0.1) );
  }else if(type=="collisionPairs"){
    uintA shapes;
    NIY;
//    for(uint i=2;i<params->parents.N;i++){
//      mlr::Shape *s = world.getShapeByName(params->parents(i)->keys.last());
//      CHECK(s,"No Shape '" <<params->parents(i)->keys.last() <<"'");
//      shapes.append(s->index);
//    }
//    map = new ProxyConstraint(pairsPTMT, shapes, (params?params->get<double>("margin", 0.1):0.1));
  }else if(type=="collisionExceptPairs"){
    uintA shapes;
    NIY;
//    for(uint i=2;i<params->parents.N;i++){
//      mlr::Shape *s = world.getShapeByName(params->parents(i)->keys.last());
//      CHECK(s,"No Shape '" <<params->parents(i)->keys.last() <<"'");
//      shapes.append(s->index);
//    }
//    map = new ProxyConstraint(allExceptPairsPTMT, shapes, (params?params->get<double>("margin", 0.1):0.1));
  }else if(type=="collisionExcept"){
    uintA shapes;
    NIY;
//    for(uint i=2;i<params->parents.N;i++){
//      mlr::Shape *s = world.getShapeByName(params->parents(i)->keys.last());
//      if(!s){
//        mlr::Body *b = world.getBodyByName(params->parents(i)->keys.last());
//        CHECK(b,"No shape or body '" <<params->parents(i)->keys.last() <<"'");
//        for(mlr::Shape *s:b->shapes) shapes.append(s->index);
//      }else{
//        shapes.append(s->index);
//      }
//    }
//    map = new ProxyConstraint(allExceptListedPTMT, shapes, (params?params->get<double>("margin", 0.1):0.1));
  }else if(type=="qItself"){
    if(params["ref1"] && params["ref2"]){
      mlr::Joint *j=world.getJointByBodyNames(params.get<mlr::String>("ref1"), params.get<mlr::String>("ref2"));
      if(!j) return NULL;
      map = new TaskMap_qItself({j->to->index}, false);
    }else if(params["ref1"]) map = new TaskMap_qItself(QIP_byJointNames, {params.get<mlr::String>("ref1")}, world);
    else if(params["Hmetric"]){ NIY /* map = new TaskMap_qItself(params.get<double>("Hmetric")*world.getHmetric());*/ } //world.naturalQmetric()); //
    else map = new TaskMap_qItself();
  }else if(type=="qZeroVels"){
    map = new TaskMap_qZeroVels();
  }else if(type=="GJK"){
    map = new TaskMap_GJK(world, params.get<mlr::String>("ref1"), params.get<mlr::String>("ref2"), true);
  }else{
    map = new TaskMap_Default(params, world);
  }

  map->order = params.get<double>("order", 0);
  return map;
}

//===========================================================================

TaskMap *TaskMap::newTaskMap(const Node* specs, const mlr::KinematicWorld& world){
  if(specs->parents.N<2) return NULL; //these are not task specs

  //-- get tags
  mlr::String& type=specs->parents(1)->keys.last();
  const char *ref1=NULL, *ref2=NULL;
  if(specs->parents.N>2) ref1=specs->parents(2)->keys.last().p;
  if(specs->parents.N>3) ref2=specs->parents(3)->keys.last().p;

  //-- create a task map
  TaskMap *map;
  const Graph* params=NULL;
  if(specs->isGraph()) params = &specs->graph();
//  mlr::String type = specs.get<mlr::String>("type", "pos");
  if(type=="wheels"){
    map = new TaskMap_qItself(QIP_byJointNames, {"worldTranslationRotation"}, world);
  }else if(type=="collisionIneq"){
    map = new CollisionConstraint( (params?params->get<double>("margin", 0.1):0.1) );
  }else if(type=="limitIneq"){
    map = new LimitsConstraint();
  }else if(type=="collisionPairs"){
    uintA shapes;
    for(uint i=2;i<specs->parents.N;i++){
      mlr::Shape *s = world.getShapeByName(specs->parents(i)->keys.last());
      CHECK(s,"No Shape '" <<specs->parents(i)->keys.last() <<"'");
      shapes.append(s->index);
    }
    map = new TaskMap_ProxyConstraint(pairsPTMT, shapes, (params?params->get<double>("margin", 0.1):0.1));
  }else if(type=="collisionExceptPairs"){
    uintA shapes;
    for(uint i=2;i<specs->parents.N;i++){
      mlr::Shape *s = world.getShapeByName(specs->parents(i)->keys.last());
      CHECK(s,"No Shape '" <<specs->parents(i)->keys.last() <<"'");
      shapes.append(s->index);
    }
    map = new TaskMap_ProxyConstraint(allExceptPairsPTMT, shapes, (params?params->get<double>("margin", 0.1):0.1));
  }else if(type=="collisionExcept"){
    uintA shapes;
    for(uint i=2;i<specs->parents.N;i++){
      mlr::Shape *s = world.getShapeByName(specs->parents(i)->keys.last());
      if(!s){
        mlr::Body *b = world.getBodyByName(specs->parents(i)->keys.last());
        CHECK(b,"No shape or body '" <<specs->parents(i)->keys.last() <<"'");
        for(mlr::Shape *s:b->shapes) shapes.append(s->index);
      }else{
        shapes.append(s->index);
      }
    }
    map = new TaskMap_ProxyConstraint(allExceptListedPTMT, shapes, (params?params->get<double>("margin", 0.1):0.1));
  }else if(type=="proxy"){
    map = new TaskMap_Proxy(allPTMT, {0u}, (params?params->get<double>("margin", 0.1):0.1) );
  }else if(type=="qItself"){
    if(ref1 && ref2){
      mlr::Joint *j=world.getJointByBodyNames(ref1, ref2);
      if(!j) return NULL;
      map = new TaskMap_qItself({j->to->index}, false);
    }else if(ref1) map = new TaskMap_qItself(QIP_byJointNames, {ref1}, world);
    else if(params && params->getNode("Hmetric")){ NIY /*map = new TaskMap_qItself(params->getNode("Hmetric")->get<double>()*world.getHmetric()); */}//world.naturalQmetric()); //
    else if(params && params->getNode("relative")) map = new TaskMap_qItself(true); //world.naturalQmetric()); //
    else map = new TaskMap_qItself();
  }else if(type=="qZeroVels"){
    map = new TaskMap_qZeroVels();
  }else if(type=="GJK"){
    map = new TaskMap_GJK(world, ref1, ref2, true);
  }else if(type=="Transition"){
    map = new TaskMap_Transition(world);
  }else if(type=="FixJointVelocities"){
    map = new TaskMap_Transition(world, true);
    dynamic_cast<TaskMap_Transition*>(map)->velCoeff = 1.;
  }else if(type=="FixSwichedObjects"){
    map = new TaskMap_FixSwichedObjects();
  }else{
    map = new TaskMap_Default(specs, world);
  }

  //-- check additional real-valued parameters: order
  if(specs->isGraph()){
    const Graph& params = specs->graph();
    map->order = params.get<double>("order", 0);
  }
  return map;
}
