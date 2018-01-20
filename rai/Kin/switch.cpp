#include "switch.h"
#include "kin.h"
#include <climits>
#include "flag.h"

//===========================================================================

template<> const char* mlr::Enum<mlr::SwitchType>::names []={
  "deleteJoint",
  "SW_effJoint",
  "addJointAtFrom",
  "addJointAtTo",
  "SW_actJoint",
  "addSliderMechanism",
  "SW_insertEffJoint",
  "insertActuated",
  "makeDynamic",
  NULL
};

//===========================================================================
//
// Kinematic Switch
//

mlr::KinematicSwitch::KinematicSwitch()
  : symbol(none), jointType(JT_none), timeOfApplication(UINT_MAX), fromId(UINT_MAX), toId(UINT_MAX), jA(0), jB(0){
}

mlr::KinematicSwitch::KinematicSwitch(SwitchType op, JointType type, const char* ref1, const char* ref2, const mlr::KinematicWorld& K, uint _timeOfApplication, const mlr::Transformation& jFrom, const mlr::Transformation& jTo)
  : symbol(op), jointType(type), timeOfApplication(_timeOfApplication), fromId(UINT_MAX), toId(UINT_MAX), jA(0), jB(0){
  if(ref1) fromId = K.getFrameByName(ref1)->ID;
  if(ref2) toId = K.getFrameByName(ref2)->ID;
  if(&jFrom) jA = jFrom;
  if(&jTo)   jB = jTo;
}

#define STEP(t) (floor(t*double(stepsPerPhase) + .500001))-1

void mlr::KinematicSwitch::setTimeOfApplication(double time, bool before, int stepsPerPhase, uint T){
  if(stepsPerPhase<0) stepsPerPhase=T;
  timeOfApplication = STEP(time)+(before?0:1);
}

void mlr::KinematicSwitch::apply(KinematicWorld& K){
  Frame *from=NULL, *to=NULL;
  if(fromId!=UINT_MAX) from=K.frames(fromId);
  if(toId!=UINT_MAX) to=K.frames(toId);

  if(symbol==deleteJoint){
#if 1
    //first search for the joint below frame
    Frame *f = to;
    for(;;){
      if(!f->parent) break;
      if(f->joint) break;
      f = f->parent;
    }
    if(!f->joint){
      LOG(-1) <<"there were no deletable links below '" <<to->name <<"'! Deleted before?";
    }else{
      f->unLink();
    }
#else
    //this deletes ALL links downward from to until a named or shape-attached one!
    Frame *f = to;
    uint i=0;
    for(;;){
      if(!f->link) break;
      Frame *from = f->link->from;
      delete f->link;
      i++;
      if(from->name.N || from->shape || from->inertia) break;
      f = from;
    }
    if(!i){
      LOG(-1) <<"there were no deletable links below '" <<to->name <<"'! Deleted before?";
    }
#endif
    K.calc_q();
    K.checkConsistency();
    return;
  }

  if(symbol==SW_effJoint || symbol==SW_actJoint || symbol==SW_insertEffJoint || symbol==insertActuated){
    //first find lowest frame below to
    {
      mlr::Transformation Q = 0;
#if 1
      mlr::Frame *link = to->getUpwardLink(Q);
      if(link && link!=to) to = link;
#else
      while(to->parent){
        if(to->joint) break; //don't jump over joints
        Q = to->Q * Q;
        to = to->parent;
      }
#endif
      if(!Q.isZero())
        jA.appendTransformation(-Q);
    }

    Joint *j = NULL;
    if(symbol!=SW_insertEffJoint && symbol!=insertActuated){
      if(to->parent) to->unLink();
      to->linkFrom(from);
      j = new Joint(*to);
    }else{
      CHECK(!from, "from should not be specified");
      CHECK(to->parent, "to needs to have a link already");
      Frame *l = to->insertPreLink(mlr::Transformation(0));
      j = new Joint(*l);
    }
    if(symbol==SW_actJoint || symbol==insertActuated){
      j->constrainToZeroVel=false;
      j->frame.flags &= ~(1<<FL_zeroQVel);
    }else{
      j->constrainToZeroVel=true;
      j->frame.flags |= (1<<FL_zeroQVel);
    }
    j->type = jointType;
    if(!jA.isZero()){
      j->frame.insertPreLink(jA);
    }
    if(!jB.isZero()){
      j->frame.insertPostLink(jB);
    }
    K.calc_q();
    K.calc_fwdPropagateFrames();
    K.checkConsistency();
    return;
  }

  if(symbol==addSliderMechanism){
//    HALT("I think it is better if there is fixed slider mechanisms in the world, that may jump; no dynamic creation of bodies");
    Frame *slider1 = new Frame(K); //{ type=ST_box size=[.2 .1 .05 0] color=[0 0 0] }
    Frame *slider2 = new Frame(K); //{ type=ST_box size=[.2 .1 .05 0] color=[1 0 0] }
    Shape *s1 = new Shape(*slider1); s1->type()=ST_box; s1->size()={.2,.1,.05}; s1->mesh().C={0.,0,0};
    Shape *s2 = new Shape(*slider2); s2->type()=ST_box; s2->size()={.2,.1,.05}; s2->mesh().C={1.,0,0};

    //unlink to
    if(to->parent) to->unLink();

    //placement of the slider1 on the table -> fixed
    slider1->linkFrom(from);
    Joint *j1 = new Joint(*slider1);
    j1->type = JT_transXYPhi;
    j1->constrainToZeroVel=true;
    //the actual sliding translation -> articulated
    slider2->linkFrom(slider1);
    Joint *j2 = new Joint(*slider2);
    j2->type = JT_transX;
    j2->constrainToZeroVel=false;
    //orientation of the object on the slider2 -> fixed
    to->linkFrom(slider2);
    Joint *j3 = new Joint(*to);
    j3->type = JT_hingeZ;
    j3->constrainToZeroVel=true;

    //    NIY;//j3->B = jB;
    if(!jA.isZero()){
      slider1->insertPreLink(jA);
    }

    K.calc_q();
    K.calc_fwdPropagateFrames();
    K.checkConsistency();
    return;
  }

  if(symbol==addJointAtTo){
    if(to->parent) to->unLink();
    to->linkFrom(from, true);
//    Joint *j = new Joint(*to);
//    j->constrainToZeroVel=true;
//    j->type = jointType;

//    jA.setDifference(from->X, to->X);
//    j->frame.insertPreLink(jA);

    K.calc_q();
    K.calc_fwdPropagateFrames();
    K.checkConsistency();
    return;
  }

  if(symbol==makeDynamic){
    CHECK(from->inertia, "can only make frames with intertia dynamic");
    from->inertia->type=mlr::BT_dynamic;
    if(from->joint){
      from->joint->constrainToZeroVel=false;
      from->joint->H = 1e-1;
    }
    return;
  }

  HALT("shouldn't be here!");
}

mlr::String mlr::KinematicSwitch::shortTag(const mlr::KinematicWorld* G) const{
  mlr::String str;
  str <<"  timeOfApplication=" <<timeOfApplication;
  str <<"  symbol=" <<symbol;
  str <<"  jointType=" <<jointType;
  str <<"  fromId=" <<(fromId==UINT_MAX?"NULL":(G?G->frames(fromId)->name:STRING(fromId)));
  str <<"  toId=" <<(G?G->frames(toId)->name:STRING(toId)) <<endl;
  return str;
}

void mlr::KinematicSwitch::write(std::ostream& os, mlr::KinematicWorld* K) const{
  os <<"SWITCH  timeOfApplication=" <<timeOfApplication;
  os <<"  symbol=" <<symbol;
  os <<"  jointType=" <<jointType;
  os <<"  fromId=" <<(int)fromId;
  if(K && fromId<UINT_MAX) os <<"'" <<K->frames(fromId)->name <<"'";
  os <<"  toId=" <<toId;
  if(K && toId<UINT_MAX) os <<"'" <<K->frames(toId)->name <<"'";
}

//===========================================================================

mlr::KinematicSwitch* mlr::KinematicSwitch::newSwitch(const Node *specs, const mlr::KinematicWorld& world, int stepsPerPhase, uint T){
  if(specs->parents.N<2) return NULL;

  //-- get tags
  mlr::String& tt=specs->parents(0)->keys.last();
  mlr::String& type=specs->parents(1)->keys.last();
  const char *ref1=NULL, *ref2=NULL;
  if(specs->parents.N>2) ref1=specs->parents(2)->keys.last().p;
  if(specs->parents.N>3) ref2=specs->parents(3)->keys.last().p;

  if(tt!="MakeJoint") return NULL;
  mlr::KinematicSwitch* sw = newSwitch(type, ref1, ref2, world, stepsPerPhase + 1);

  if(specs->isGraph()){
    const Graph& params = specs->graph();
    sw->setTimeOfApplication(params.get<double>("time",1.), params.get<bool>("time", false), stepsPerPhase, T);
//    sw->timeOfApplication = *stepsPerPhase + 1;
    params.get(sw->jA, "from");
    params.get(sw->jB, "to");
  }
  return sw;
}

mlr::KinematicSwitch* mlr::KinematicSwitch::newSwitch(const mlr::String& type, const char* ref1, const char* ref2, const mlr::KinematicWorld& world, uint _timeOfApplication, const mlr::Transformation& jFrom, const mlr::Transformation& jTo){
  //-- create switch
  mlr::KinematicSwitch *sw= new mlr::KinematicSwitch();
  if(type=="addRigid"){ sw->symbol=mlr::SW_effJoint; sw->jointType=mlr::JT_rigid; }
//  else if(type=="addRigidRel"){ sw->symbol = mlr::KinematicSwitch::addJointAtTo; sw->jointType=mlr::JT_rigid; }
  else if(type=="rigidAtTo"){ sw->symbol = mlr::addJointAtTo; sw->jointType=mlr::JT_rigid; }
  else if(type=="rigidAtFrom"){ sw->symbol = mlr::addJointAtFrom; sw->jointType=mlr::JT_rigid; }
  else if(type=="rigidZero"){ sw->symbol = mlr::SW_effJoint; sw->jointType=mlr::JT_rigid; }
  else if(type=="transXActuated"){ sw->symbol = mlr::SW_actJoint; sw->jointType=mlr::JT_transX; }
  else if(type=="transXYPhiAtFrom"){ sw->symbol = mlr::addJointAtFrom; sw->jointType=mlr::JT_transXYPhi; }
  else if(type=="transXYPhiZero"){ sw->symbol = mlr::SW_effJoint; sw->jointType=mlr::JT_transXYPhi; }
  else if(type=="transXYPhiActuated"){ sw->symbol = mlr::SW_actJoint; sw->jointType=mlr::JT_transXYPhi; }
  else if(type=="freeAtTo"){ sw->symbol = mlr::addJointAtTo; sw->jointType=mlr::JT_free; }
  else if(type=="freeZero"){ sw->symbol = mlr::SW_effJoint; sw->jointType=mlr::JT_free; }
  else if(type=="freeActuated"){ sw->symbol = mlr::SW_actJoint; sw->jointType=mlr::JT_free; }
  else if(type=="ballZero"){ sw->symbol = mlr::SW_effJoint; sw->jointType=mlr::JT_quatBall; }
  else if(type=="hingeZZero"){ sw->symbol = mlr::SW_effJoint; sw->jointType=mlr::JT_hingeZ; }
  else if(type=="sliderMechanism"){ sw->symbol = mlr::addSliderMechanism; }
  else if(type=="delete"){ sw->symbol = mlr::deleteJoint; }
  else if(type=="JT_XBall"){ sw->symbol = mlr::SW_effJoint; sw->jointType=mlr::JT_XBall; }
  else if(type=="JT_transZ"){ sw->symbol = mlr::SW_effJoint; sw->jointType=mlr::JT_transZ; }
  else if(type=="JT_transX"){ sw->symbol = mlr::SW_effJoint; sw->jointType=mlr::JT_transX; }
  else if(type=="JT_trans3"){ sw->symbol = mlr::SW_effJoint; sw->jointType=mlr::JT_trans3; }
  else if(type=="insert_transX"){ sw->symbol = mlr::SW_insertEffJoint; sw->jointType=mlr::JT_transX; }
  else if(type=="insert_trans3"){ sw->symbol = mlr::SW_insertEffJoint; sw->jointType=mlr::JT_trans3; }
  else if(type=="createSlider"){ sw->symbol = mlr::addSliderMechanism; }
  else if(type=="makeDynamic"){ sw->symbol = mlr::makeDynamic; }
  else HALT("unknown type: "<< type);
  if(ref1) sw->fromId = world.getFrameByName(ref1)->ID;
  if(ref2) sw->toId = world.getFrameByName(ref2)->ID;
//  if(!ref2){
//    CHECK_EQ(sw->symbol, mlr::deleteJoint, "");
//    mlr::Body *b = fromShape->body;
//    if(b->hasJoint()==1){
////      CHECK_EQ(b->outLinks.N, 0, "");
//      sw->toId = sw->fromId;
//      sw->fromId = b->joint()->from->shapes.first()->index;
//    }else if(b->outLinks.N==1){
//      CHECK_EQ(b->hasJoint(), 0, "");
//      sw->toId = b->outLinks(0)->from->shapes.first()->index;
//    }else if(b->hasJoint()==0 && b->outLinks.N==0){
//      MLR_MSG("No link to delete for shape '" <<ref1 <<"'");
//      delete sw;
//      return NULL;
//    }else HALT("that's ambiguous");
//  }else{

  sw->timeOfApplication = _timeOfApplication;
  if(&jFrom) sw->jA = jFrom;
  if(&jTo) sw->jB = jTo;
  return sw;
}

