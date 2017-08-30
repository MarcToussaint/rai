#include "switch.h"
#include "kin.h"
#include <climits>

//===========================================================================

template<> const char* mlr::Enum<mlr::KinematicSwitch::OperatorSymbol>::names []={
  "deleteJoint", "addJointZero", "addJointAtFrom", "addJointAtTo", "addArticulated", "addSliderMechanism", "insertJoint", NULL
};

//===========================================================================
//
// Kinematic Switch
//

mlr::KinematicSwitch::KinematicSwitch()
  : symbol(none), jointType(JT_none), timeOfApplication(UINT_MAX), fromId(UINT_MAX), toId(UINT_MAX), jA(0), jB(0){
}

mlr::KinematicSwitch::KinematicSwitch(OperatorSymbol op, JointType type, const char* ref1, const char* ref2, const mlr::KinematicWorld& K, uint _timeOfApplication, const mlr::Transformation& jFrom, const mlr::Transformation& jTo)
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

void mlr::KinematicSwitch::apply(KinematicWorld& G){
  Frame *from=NULL, *to=NULL;
  if(fromId!=UINT_MAX) from=G.frames(fromId);
  if(toId!=UINT_MAX) to=G.frames(toId);

  if(symbol==deleteJoint){
#if 1
    //first search for the joint below frame
    Frame *f = to;
    for(;;){
      if(!f->link) break;
      if(f->link->joint) break;
      f = f->link->from;
    }
    if(!f->link){
      LOG(-1) <<"there were no deletable links below '" <<to->name <<"'! Deleted before?";
    }else{
      delete f->link;
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
    G.calc_q();
//    G.checkConsistency();
    return;
  }

  if(symbol==addJointZero || symbol==addActuated || symbol==insertJoint){
    //first find lowest frame below to
    mlr::Transformation Q = 0;
    while(to->link){
      if(to->link->joint) break; //don't jump over joints
      Q = to->link->Q * Q;
      to = to->link->from;
    }

    Joint *j = NULL;
    if(symbol!=insertJoint){
      j = new Joint(from, to);
    }else{
      CHECK(!from, "from should not be specified");
      CHECK(to->link, "to needs to have a link already");
      Link *l = to->link->insertPostLink(mlr::Transformation(0));
      j = new Joint(l);
    }
    if(symbol==addActuated) j->constrainToZeroVel=false;
    else                    j->constrainToZeroVel=true;
    j->type = jointType;
    if(!jA.isZero()){
      j->link->insertPreLink(jA);
    }
    if(!Q.isZero()){
      j->link->insertPostLink(-Q);
      G.checkConsistency();
    }
    G.calc_q();
    G.calc_fwdPropagateFrames();
    G.checkConsistency();
    return;
  }
  if(symbol==addSliderMechanism){
    HALT("I think it is better if there is fixed slider mechanisms in the world, that may jump; no dynamic creation of bodies");
    Frame *slider1 = new Frame(G); //{ type=ST_box size=[.2 .1 .05 0] color=[0 0 0] }
    Frame *slider2 = new Frame(G); //{ type=ST_box size=[.2 .1 .05 0] color=[1 0 0] }
    Shape *s1 = new Shape(slider1); s1->type=ST_box; s1->size={.2,.1,.05}; s1->mesh.C={0.,0,0};
    Shape *s2 = new Shape(slider2); s2->type=ST_box; s2->size={.2,.1,.05}; s2->mesh.C={1.,0,0};

    //placement of the slider1 on the table -> fixed
    Joint *j1 = new Joint(from, slider1);
    j1->type = JT_transXYPhi;
    j1->constrainToZeroVel=true;
    //the actual sliding translation -> articulated
    Joint *j2 = new Joint(slider1, slider2);
    j2->type = JT_transX;
    j2->constrainToZeroVel=false;
    //orientation of the object on the slider2 -> fixed
    Joint *j3 = new Joint(slider2, to);
    j3->type = JT_hingeZ;
    j3->constrainToZeroVel=true;
    NIY;//j3->B = jB;

    G.calc_q();
    G.calc_fwdPropagateFrames();
    G.checkConsistency();
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
  if(K) os <<"'" <<K->frames(toId)->name <<"'";
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
  if(type=="addRigid"){ sw->symbol=mlr::KinematicSwitch::addJointZero; sw->jointType=mlr::JT_rigid; }
//  else if(type=="addRigidRel"){ sw->symbol = mlr::KinematicSwitch::addJointAtTo; sw->jointType=mlr::JT_rigid; }
  else if(type=="rigidAtTo"){ sw->symbol = mlr::KinematicSwitch::addJointAtTo; sw->jointType=mlr::JT_rigid; }
  else if(type=="rigidAtFrom"){ sw->symbol = mlr::KinematicSwitch::addJointAtFrom; sw->jointType=mlr::JT_rigid; }
  else if(type=="rigidZero"){ sw->symbol = mlr::KinematicSwitch::addJointZero; sw->jointType=mlr::JT_rigid; }
  else if(type=="transXActuated"){ sw->symbol = mlr::KinematicSwitch::addActuated; sw->jointType=mlr::JT_transX; }
  else if(type=="transXYPhiAtFrom"){ sw->symbol = mlr::KinematicSwitch::addJointAtFrom; sw->jointType=mlr::JT_transXYPhi; }
  else if(type=="transXYPhiZero"){ sw->symbol = mlr::KinematicSwitch::addJointZero; sw->jointType=mlr::JT_transXYPhi; }
  else if(type=="transXYPhiActuated"){ sw->symbol = mlr::KinematicSwitch::addActuated; sw->jointType=mlr::JT_transXYPhi; }
  else if(type=="freeAtTo"){ sw->symbol = mlr::KinematicSwitch::addJointAtTo; sw->jointType=mlr::JT_free; }
  else if(type=="freeZero"){ sw->symbol = mlr::KinematicSwitch::addJointZero; sw->jointType=mlr::JT_free; }
  else if(type=="freeActuated"){ sw->symbol = mlr::KinematicSwitch::addActuated; sw->jointType=mlr::JT_free; }
  else if(type=="ballZero"){ sw->symbol = mlr::KinematicSwitch::addJointZero; sw->jointType=mlr::JT_quatBall; }
  else if(type=="hingeZZero"){ sw->symbol = mlr::KinematicSwitch::addJointZero; sw->jointType=mlr::JT_hingeZ; }
  else if(type=="sliderMechanism"){ sw->symbol = mlr::KinematicSwitch::addSliderMechanism; }
  else if(type=="delete"){ sw->symbol = mlr::KinematicSwitch::deleteJoint; }
  else if(type=="JT_XBall"){ sw->symbol = mlr::KinematicSwitch::addJointZero; sw->jointType=mlr::JT_XBall; }
  else if(type=="JT_transZ"){ sw->symbol = mlr::KinematicSwitch::addJointZero; sw->jointType=mlr::JT_transZ; }
  else if(type=="JT_trans3"){ sw->symbol = mlr::KinematicSwitch::addJointZero; sw->jointType=mlr::JT_trans3; }
  else if(type=="insert_transX"){ sw->symbol = mlr::KinematicSwitch::insertJoint; sw->jointType=mlr::JT_transX; }
  else if(type=="insert_trans3"){ sw->symbol = mlr::KinematicSwitch::insertJoint; sw->jointType=mlr::JT_trans3; }
  else HALT("unknown type: "<< type);
  if(ref1) sw->fromId = world.getFrameByName(ref1)->ID;
  if(ref2) sw->toId = world.getFrameByName(ref2)->ID;
//  if(!ref2){
//    CHECK_EQ(sw->symbol, mlr::KinematicSwitch::deleteJoint, "");
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

const char* mlr::KinematicSwitch::name(mlr::KinematicSwitch::OperatorSymbol s){
  HALT("deprecated");
  static const char* names[] = { "deleteJoint", "addJointZero", "addJointAtFrom", "addJointAtTo", "addArticulated" };
  if(s==none) return "none";
  return names[(int)s];
}
