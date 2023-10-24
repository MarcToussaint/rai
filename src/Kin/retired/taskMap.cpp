/*  ------------------------------------------------------------------
    Copyright (c) 2011-2020 Marc Toussaint
    email: toussaint@tu-berlin.de

    This code is distributed under the MIT License.
    Please see <root-path>/LICENSE for details.
    --------------------------------------------------------------  */

//===========================================================================

Feature* Feature::newTaskMap(const Graph& params, const rai::Configuration& world) {
  //-- get tags
  rai::String type = params.get<rai::String>("map", "default");

  //-- create a task map
  Feature* map;
  if(type=="wheels") {
    map = new TM_qItself(QIP_byJointNames, {"worldTranslationRotation"}, world);
  } else if(type=="collisionIneq") {
    map = new CollisionConstraint(params.get<double>("margin", 0.1));
  } else if(type=="limitIneq") {
    map = new LimitsConstraint();
  } else if(type=="proxy") {
    map = new TM_Proxy(TMT_allP, {0u}, params.get<double>("margin", 0.1));
  } else if(type=="collisionPairs") {
    uintA shapes;
    NIY;
//    for(uint i=2;i<params->parents.N;i++){
//      rai::Shape *s = world.getShapeByName(params->parents(i)->keys.last());
//      CHECK(s,"No Shape '" <<params->parents(i)->keys.last() <<"'");
//      shapes.append(s->index);
//    }
//    map = new ProxyConstraint(TMT_pairsP, shapes, (params?params->get<double>("margin", 0.1):0.1));
  } else if(type=="collisionExceptPairs") {
    uintA shapes;
    NIY;
//    for(uint i=2;i<params->parents.N;i++){
//      rai::Shape *s = world.getShapeByName(params->parents(i)->keys.last());
//      CHECK(s,"No Shape '" <<params->parents(i)->keys.last() <<"'");
//      shapes.append(s->index);
//    }
//    map = new ProxyConstraint(TMT_allExceptPairsP, shapes, (params?params->get<double>("margin", 0.1):0.1));
  } else if(type=="collisionExcept") {
    uintA shapes;
    NIY;
//    for(uint i=2;i<params->parents.N;i++){
//      rai::Shape *s = world.getShapeByName(params->parents(i)->keys.last());
//      if(!s){
//        rai::Body *b = world.getBodyByName(params->parents(i)->keys.last());
//        CHECK(b,"No shape or body '" <<params->parents(i)->keys.last() <<"'");
//        for(rai::Shape *s:b->shapes) shapes.append(s->index);
//      }else{
//        shapes.append(s->index);
//      }
//    }
//    map = new ProxyConstraint(TMT_allExceptListedP, shapes, (params?params->get<double>("margin", 0.1):0.1));
  } else if(type=="qItself") {
    if(params["ref1"] && params["ref2"]) {
      rai::Joint* j=world.getJointByBodyNames(params.get<rai::String>("ref1"), params.get<rai::String>("ref2"));
      if(!j) return nullptr;
      map = new TM_qItself({j->frame.ID}, false);
    } else if(params["ref1"]) map = new TM_qItself(QIP_byJointNames, {params.get<rai::String>("ref1")}, world);
    else if(params["Hmetric"]) { NIY /* map = new TM_qItself(params.get<double>("Hmetric")*world.getHmetric());*/ } //world.naturalQmetric()); //
    else map = new TM_qItself();
  } else if(type=="qZeroVels") {
    map = new TM_qZeroVels();
  } else if(type=="GJK") {
    map = new TM_GJK(world, params.get<rai::String>("ref1"), params.get<rai::String>("ref2"), true);
  } else {
    map = new TM_Default(params, world);
  }

  map->order = params.get<double>("order", 0);
  return map;
}

//===========================================================================

Feature* Feature::newTaskMap(const Node* specs, const rai::Configuration& world) {
  if(specs->parents.N<2) return nullptr; //these are not task specs

  //-- get tags
  rai::String& type=specs->parents(1)->keys.last();
  const char* ref1=nullptr, *ref2=nullptr;
  if(specs->parents.N>2) ref1=specs->parents(2)->keys.last().p;
  if(specs->parents.N>3) ref2=specs->parents(3)->keys.last().p;

  //-- create a task map
  Feature* map;
  const Graph* params=nullptr;
  if(specs->isGraph()) params = &specs->graph();
//  rai::String type = specs.get<rai::String>("type", "pos");
  if(type=="wheels") {
    map = new TM_qItself(QIP_byJointNames, {"worldTranslationRotation"}, world);
  } else if(type=="collisionIneq") {
    map = new CollisionConstraint((params?params->get<double>("margin", 0.1):0.1));
  } else if(type=="limitIneq") {
    map = new LimitsConstraint();
  } else if(type=="collisionPairs") {
    uintA shapes;
    for(uint i=2; i<specs->parents.N; i++) {
      rai::Frame* s = world.getFrameByName(specs->parents(i)->keys.last());
      CHECK(s, "No Shape '" <<specs->parents(i)->keys.last() <<"'");
      shapes.append(s->ID);
    }
    map = new TM_ProxyConstraint(TMT_pairsP, shapes, (params?params->get<double>("margin", 0.1):0.1));
  } else if(type=="collisionExceptPairs") {
    uintA shapes;
    for(uint i=2; i<specs->parents.N; i++) {
      rai::Frame* s = world.getFrameByName(specs->parents(i)->keys.last());
      CHECK(s, "No Shape '" <<specs->parents(i)->keys.last() <<"'");
      shapes.append(s->ID);
    }
    map = new TM_ProxyConstraint(TMT_allExceptPairsP, shapes, (params?params->get<double>("margin", 0.1):0.1));
  } else if(type=="collisionExcept") {
    uintA shapes;
    for(uint i=2; i<specs->parents.N; i++) {
      rai::Frame* s = world.getFrameByName(specs->parents(i)->keys.last());
      CHECK(s, "No shape or body '" <<specs->parents(i)->keys.last() <<"'");
      shapes.append(s->ID);
    }
    map = new TM_ProxyConstraint(TMT_allExceptListedP, shapes, (params?params->get<double>("margin", 0.1):0.1));
  } else if(type=="proxy") {
    map = new TM_Proxy(TMT_allP, {0u}, (params?params->get<double>("margin", 0.1):0.1));
  } else if(type=="qItself") {
    if(ref1 && ref2) {
      rai::Joint* j=world.getJointByBodyNames(ref1, ref2);
      if(!j) return nullptr;
      map = new TM_qItself({j->frame.ID}, false);
    } else if(ref1) map = new TM_qItself(QIP_byJointNames, {ref1}, world);
    else if(params && params->getNode("Hmetric")) { NIY /*map = new TM_qItself(params->getNode("Hmetric")->get<double>()*world.getHmetric()); */} //world.naturalQmetric()); //
    else if(params && params->getNode("relative")) map = new TM_qItself(true); //world.naturalQmetric()); //
    else map = new TM_qItself();
  } else if(type=="qZeroVels") {
    map = new TM_qZeroVels();
  } else if(type=="GJK") {
    map = new TM_GJK(world, ref1, ref2, true);
  } else if(type=="Transition") {
    map = new TM_Transition(world);
  } else if(type=="FixJointVelocities") {
    map = new TM_Transition(world, true);
    dynamic_cast<TM_Transition*>(map)->velCoeff = 1.;
  } else if(type=="FixSwichedObjects") {
    map = new TM_FixSwichedObjects();
  } else {
    map = new TM_Default(specs, world);
  }

  //-- check additional real-valued parameters: order
  if(specs->isGraph()) {
    const Graph& params = specs->graph();
    map->order = params.get<double>("order", 0);
  }
  return map;
}
