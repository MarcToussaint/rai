/*  ------------------------------------------------------------------
    Copyright (c) 2011-2020 Marc Toussaint
    email: toussaint@tu-berlin.de

    This code is distributed under the MIT License.
    Please see <root-path>/LICENSE for details.
    --------------------------------------------------------------  */

#include "switch.h"
#include "kin.h"
#include "flag.h"
#include "forceExchange.h"

#include <climits>

//===========================================================================

int conv_time2step(double time, uint stepsPerPhase) {
  return (floor(time*double(stepsPerPhase) + .500001))-1;
}
//#define STEP(t) (floor(t*double(stepsPerPhase) + .500001))-1

//===========================================================================

template<> const char* rai::Enum<rai::SwitchType>::names []= {
  "deleteJoint",
  "SW_effJoint",
  "addJointAtFrom",
  "addJointAtTo",
  "SW_actJoint",
  "addSliderMechanism",
  "SW_insertEffJoint",
  "insertActuated",
  "makeDynamic",
  "makeKinematic",
  "SW_fixCurrent",
  "SW_delContact",
  "SW_addContact",
  "SW_addSoftContact",
  nullptr
};

//===========================================================================
//
// Kinematic Switch
//

rai::KinematicSwitch::KinematicSwitch()
  : symbol(none), jointType(JT_none), timeOfApplication(-1), fromId(UINT_MAX), toId(UINT_MAX), jA(0), jB(0) {
}

rai::KinematicSwitch::KinematicSwitch(SwitchType op, JointType type, const char* ref1, const char* ref2, const rai::Configuration& K, int _timeOfApplication, const rai::Transformation& jFrom, const rai::Transformation& jTo)
  : symbol(op), jointType(type), timeOfApplication(_timeOfApplication), fromId(UINT_MAX), toId(UINT_MAX), jA(0), jB(0) {
  if(ref1) fromId = K.getFrameByName(ref1)->ID;
  if(ref2) toId = K.getFrameByName(ref2)->ID;
  if(!!jFrom) jA = jFrom;
  if(!!jTo)   jB = jTo;
}

void rai::KinematicSwitch::setTimeOfApplication(double time, bool before, int stepsPerPhase, uint T) {
  if(stepsPerPhase<0) stepsPerPhase=T;
  timeOfApplication = (time<0.?0:conv_time2step(time, stepsPerPhase))+(before?0:1);
}

void rai::KinematicSwitch::apply(Configuration& K) {
  Frame* from=nullptr, *to=nullptr;
  if(fromId!=UINT_MAX) from=K.frames(fromId);
  if(toId!=UINT_MAX) to=K.frames(toId);

  if(symbol==deleteJoint) {
    CHECK_EQ(jointType, JT_none, "");

#if 1
    //first search for the joint below frame
    Frame* f = to;
    for(;;) {
      if(!f->parent) break;
      if(f->joint) break;
      f = f->parent;
    }
    if(!f->joint) {
      LOG(-1) <<"there were no deletable links below '" <<to->name <<"'! Deleted before?";
    } else {
      f->unLink();
    }
#else
    //this deletes ALL links downward from to until a named or shape-attached one!
    Frame* f = to;
    uint i=0;
    for(;;) {
      if(!f->link) break;
      Frame* from = f->link->from;
      delete f->link;
      i++;
      if(from->name.N || from->shape || from->inertia) break;
      f = from;
    }
    if(!i) {
      LOG(-1) <<"there were no deletable links below '" <<to->name <<"'! Deleted before?";
    }
#endif
    K.ensure_q();
    K.checkConsistency();
    return;
  }

  bool newVersion = true;
  if(symbol==SW_effJoint || symbol==SW_actJoint || symbol==SW_insertEffJoint || symbol==insertActuated) {
    //first find lowest frame below to
    if(!newVersion) {
      rai::Transformation Q = 0;
#if 1
      rai::Frame* link = to->getUpwardLink(Q);
      if(link && link!=to) to = link;
#else
      while(to->parent) {
        if(to->joint) break; //don't jump over joints
        Q = to->Q * Q;
        to = to->parent;
      }
#endif
      if(!Q.isZero())
        jA.appendTransformation(-Q);
    }

    Joint* j = nullptr;
    if(symbol!=SW_insertEffJoint && symbol!=insertActuated) {
      if(newVersion) {
        rai::Frame* link = to->getUpwardLink();
        if(link->parent) link->unLink();
        K.reconfigureRootOfSubtree(to);
      } else {
        if(to->parent) to->unLink();
      }
      to->linkFrom(from);
      j = new Joint(*to);
    } else {
      CHECK(!from, "from should not be specified");
      CHECK(to->parent, "to needs to have a link already");
      Frame* mid = to->insertPreLink(rai::Transformation(0));
      j = new Joint(*mid);
    }
    if(!jA.isZero()) {
      if(newVersion) {
        j->frame.insertPreLink(jA);
      } else { //in the old version: when not reconfiguring the grasped frame to root, you need to insert -Q AFTER the joint to transform back to the old link's root
        Frame* mid = to->insertPreLink();
        delete j;
        j = new rai::Joint(*mid);
        to->Q = jA;
      }
    }
    if(!jB.isZero()) {
      j->frame.insertPostLink(jB);
    }
    if(symbol==SW_actJoint || symbol==insertActuated) {
      j->constrainToZeroVel=false;
      j->frame.flags &= ~(1<<FL_zeroQVel);
    } else {
      j->constrainToZeroVel=true;
      j->frame.flags |= (1<<FL_zeroQVel);
      j->H = 0.;
    }
    j->setType(jointType);
    if(false) { //this is so annoying!
      j->frame.Q = j->frame.X / j->frame.parent->X; //that's important for the initialization of x during the very first komo.setupConfigurations !!
      arr q = j->calc_q_from_Q(j->frame.Q);
      j->frame.Q.setZero();
      j->calc_Q_from_q(q, 0);
//      j->frame.Q.pos.setZero();
    }
    K.ensure_q();
    K.calc_fwdPropagateFrames();
    K.checkConsistency();
    return;
  }

  if(symbol==addSliderMechanism) {
//    HALT("I think it is better if there is fixed  slider mechanisms in the world, that may jump; no dynamic creation of bodies");
    Frame* slider1 = new Frame(K); //{ type=ST_box size=[.2 .1 .05 0] color=[0 0 0] }
    Frame* slider2 = new Frame(K); //{ type=ST_box size=[.2 .1 .05 0] color=[1 0 0] }
    Shape* s1 = new Shape(*slider1); s1->type()=ST_box; s1->size()= {.2, .1, .05}; s1->mesh().C= {0., 0, 0};
    Shape* s2 = new Shape(*slider2); s2->type()=ST_box; s2->size()= {.2, .1, .05}; s2->mesh().C= {1., 0, 0};

    //unlink to
    if(to->parent) to->unLink();

    //placement of the slider1 on the table -> fixed
    slider1->linkFrom(from);
    Joint* j1 = new Joint(*slider1);
    j1->type = JT_transXYPhi;
    j1->constrainToZeroVel=true;
    //the actual sliding translation -> articulated
    slider2->linkFrom(slider1);
    Joint* j2 = new Joint(*slider2);
    j2->type = JT_transX;
    j2->constrainToZeroVel=false;
    //orientation of the object on the slider2 -> fixed
    to->linkFrom(slider2);
    Joint* j3 = new Joint(*to);
    j3->type = JT_hingeZ;
    j3->constrainToZeroVel=true;

    //    NIY;//j3->B = jB;
    if(!jA.isZero()) {
      slider1->insertPreLink(jA);
    }

    K.ensure_q();
    K.calc_fwdPropagateFrames();
    K.checkConsistency();
    return;
  }

  if(symbol==addJointAtTo) {
    if(to->parent) to->unLink();
    to->linkFrom(from, true);
//    Joint *j = new Joint(*to);
//    j->constrainToZeroVel=true;
//    j->type = jointType;

//    jA.setDifference(from->X, to->X);
//    j->frame.insertPreLink(jA);

    K.ensure_q();
    K.calc_fwdPropagateFrames();
    K.checkConsistency();
    return;
  }

  if(symbol==SW_fixCurrent) {
    CHECK_EQ(jointType, JT_none, "");

    if(to->parent) to->unLink();
    to->linkFrom(from, true);

    K.ensure_q();
    K.calc_fwdPropagateFrames();
    K.checkConsistency();
    return;
  }

  if(symbol==makeDynamic) {
    CHECK_EQ(jointType, JT_none, "");
    CHECK_EQ(to, 0, "");
    CHECK(from->inertia, "can only make frames with intertia dynamic");

    from->inertia->type=rai::BT_dynamic;
    if(from->joint) {
      from->joint->constrainToZeroVel=false;
      from->joint->H = 1e-1;
    }
    return;
  }

  if(symbol==makeKinematic) {
    CHECK_EQ(jointType, JT_none, "");
    CHECK_EQ(to, 0, "");
    CHECK(from->inertia, "can only make frames with intertia kinematic");

    from->inertia->type=rai::BT_kinematic;
//    if(from->joint){
//      from->joint->constrainToZeroVel=false;
//      from->joint->H = 1e-1;
//    }
    return;
  }

  if(symbol==SW_addContact || symbol==SW_addComplementaryContact) {
    CHECK_EQ(jointType, JT_none, "");
    auto c = new rai::ForceExchange(*from, *to);
    if(symbol==SW_addComplementaryContact) c->soft=true;
    c->setZero();
    return;
  }

  if(symbol==SW_delContact) {
    CHECK_EQ(jointType, JT_none, "");
    rai::ForceExchange* c = nullptr;
    for(rai::ForceExchange* cc:to->forces) if(&cc->a==from || &cc->b==from) { c=cc; break; }
    if(!c) HALT("not found");
    delete c;
    return;
  }

  HALT("shouldn't be here!");
}

rai::String rai::KinematicSwitch::shortTag(const rai::Configuration* G) const {
  rai::String str;
  str <<"  timeOfApplication=" <<timeOfApplication;
  str <<"  symbol=" <<symbol;
  str <<"  jointType=" <<jointType;
  str <<"  fromId=" <<(fromId==UINT_MAX?"nullptr":(G?G->frames(fromId)->name:STRING(fromId)));
  str <<"  toId=" <<(G?G->frames(toId)->name:STRING(toId)) <<endl;
  return str;
}

void rai::KinematicSwitch::write(std::ostream& os, rai::Configuration* K) const {
  os <<"SWITCH  timeOfApplication=" <<timeOfApplication;
  os <<"  symbol=" <<symbol;
  os <<"  jointType=" <<jointType;
  os <<"  fromId=" <<(int)fromId;
  if(K && fromId<UINT_MAX) os <<"'" <<K->frames(fromId)->name <<"'";
  os <<"  toId=" <<toId;
  if(K && toId<UINT_MAX) os <<"'" <<K->frames(toId)->name <<"'";
}

//===========================================================================

rai::KinematicSwitch* rai::KinematicSwitch::newSwitch(const Node* specs, const rai::Configuration& world, int stepsPerPhase, uint T) {
  if(specs->parents.N<2) return nullptr;

  //-- get tags
  rai::String& tt=specs->parents(0)->keys.last();
  rai::String& type=specs->parents(1)->keys.last();
  const char* ref1=nullptr, *ref2=nullptr;
  if(specs->parents.N>2) ref1=specs->parents(2)->keys.last().p;
  if(specs->parents.N>3) ref2=specs->parents(3)->keys.last().p;

  if(tt!="MakeJoint") return nullptr;
  rai::KinematicSwitch* sw = newSwitch(type, ref1, ref2, world, stepsPerPhase + 1);

  if(specs->isGraph()) {
    const Graph& params = specs->graph();
    sw->setTimeOfApplication(params.get<double>("time", 1.), params.get<bool>("time", false), stepsPerPhase, T);
//    sw->timeOfApplication = *stepsPerPhase + 1;
    params.get(sw->jA, "from");
    params.get(sw->jB, "to");
  }
  return sw;
}

rai::KinematicSwitch* rai::KinematicSwitch::newSwitch(const rai::String& type, const char* ref1, const char* ref2, const rai::Configuration& world, int _timeOfApplication, const rai::Transformation& jFrom, const rai::Transformation& jTo) {
  //-- create switch
  rai::KinematicSwitch* sw= new rai::KinematicSwitch();
  if(type=="addRigid") { sw->symbol=rai::SW_effJoint; sw->jointType=rai::JT_rigid; }
//  else if(type=="addRigidRel"){ sw->symbol = rai::KinematicSwitch::addJointAtTo; sw->jointType=rai::JT_rigid; }
  else if(type=="rigidAtTo") { sw->symbol = rai::addJointAtTo; sw->jointType=rai::JT_rigid; }
  else if(type=="rigidAtFrom") { sw->symbol = rai::addJointAtFrom; sw->jointType=rai::JT_rigid; }
  else if(type=="rigidZero") { sw->symbol = rai::SW_effJoint; sw->jointType=rai::JT_rigid; }
  else if(type=="transXActuated") { sw->symbol = rai::SW_actJoint; sw->jointType=rai::JT_transX; }
  else if(type=="transXYPhiAtFrom") { sw->symbol = rai::addJointAtFrom; sw->jointType=rai::JT_transXYPhi; }
  else if(type=="transXYPhiZero") { sw->symbol = rai::SW_effJoint; sw->jointType=rai::JT_transXYPhi; }
  else if(type=="transXYPhiActuated") { sw->symbol = rai::SW_actJoint; sw->jointType=rai::JT_transXYPhi; }
  else if(type=="freeAtTo") { sw->symbol = rai::addJointAtTo; sw->jointType=rai::JT_free; }
  else if(type=="freeZero") { sw->symbol = rai::SW_effJoint; sw->jointType=rai::JT_free; }
  else if(type=="freeActuated") { sw->symbol = rai::SW_actJoint; sw->jointType=rai::JT_free; }
  else if(type=="ballZero") { sw->symbol = rai::SW_effJoint; sw->jointType=rai::JT_quatBall; }
  else if(type=="hingeZZero") { sw->symbol = rai::SW_effJoint; sw->jointType=rai::JT_hingeZ; }
  else if(type=="sliderMechanism") { sw->symbol = rai::addSliderMechanism; }
  else if(type=="delete") { sw->symbol = rai::deleteJoint; }
  else if(type=="JT_XBall") { sw->symbol = rai::SW_effJoint; sw->jointType=rai::JT_XBall; }
  else if(type=="JT_transZ") { sw->symbol = rai::SW_effJoint; sw->jointType=rai::JT_transZ; }
  else if(type=="JT_transX") { sw->symbol = rai::SW_effJoint; sw->jointType=rai::JT_transX; }
  else if(type=="JT_trans3") { sw->symbol = rai::SW_effJoint; sw->jointType=rai::JT_trans3; }
  else if(type=="insert_transX") { sw->symbol = rai::SW_insertEffJoint; sw->jointType=rai::JT_transX; }
  else if(type=="insert_trans3") { sw->symbol = rai::SW_insertEffJoint; sw->jointType=rai::JT_trans3; }
  else if(type=="createSlider") { sw->symbol = rai::addSliderMechanism; }
  else if(type=="makeDynamic") { sw->symbol = rai::makeDynamic; }
  else if(type=="makeKinematic") { sw->symbol = rai::makeKinematic; }
  else HALT("unknown type: "<< type);
  if(ref1) sw->fromId = world.getFrameByName(ref1)->ID;
  if(ref2) sw->toId = world.getFrameByName(ref2)->ID;
//  if(!ref2){
//    CHECK_EQ(sw->symbol, rai::deleteJoint, "");
//    rai::Body *b = fromShape->body;
//    if(b->hasJoint()==1){
////      CHECK_EQ(b->children.N, 0, "");
//      sw->toId = sw->fromId;
//      sw->fromId = b->joint()->from->shapes.first()->index;
//    }else if(b->children.N==1){
//      CHECK_EQ(b->hasJoint(), 0, "");
//      sw->toId = b->children(0)->from->shapes.first()->index;
//    }else if(b->hasJoint()==0 && b->children.N==0){
//      RAI_MSG("No link to delete for shape '" <<ref1 <<"'");
//      delete sw;
//      return nullptr;
//    }else HALT("that's ambiguous");
//  }else{

  sw->timeOfApplication = _timeOfApplication;
  if(!!jFrom) sw->jA = jFrom;
  if(!!jTo) sw->jB = jTo;
  return sw;
}
