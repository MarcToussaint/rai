/*  ------------------------------------------------------------------
    Copyright (c) 2011-2024 Marc Toussaint
    email: toussaint@tu-berlin.de

    This code is distributed under the MIT License.
    Please see <root-path>/LICENSE for details.
    --------------------------------------------------------------  */

#include "kin.h"
#include "kin_oldTaskVariables.h"
#include "kin_swift.h"
#include "kin_ode.h"
#include "kin_actionInterface.h"
#include "../Gui/opengl.h"
#include "../Gui/plot.h"

#include <sstream>
#include <limits.h>
#include <unistd.h>

// huepfen der bloecke, falls sie zb runterfallen
#define ODE_COLL_BOUNCE 0.0
//usually .2!! stiffness (time-scale of contact reaction) umso groesser, desto mehr Fehlerkorrektur; muss zwischen 0.1 und 0.5 sein (ungefaehr)
#define ODE_COLL_ERP 0.4
// 0.3
//softness // umso groesser, desto breiter, desto weicher, desto weniger Fehlerkorrektur; zwischen 10e-10 und 10e5
#define ODE_COLL_CFM 10e-4
//1e-2
//alternative: dInfinity;
#define ODE_FRICTION 0.02

#define DROP_TARGET_NOISE 0.11
#define SEC_ACTION_ABORT 500

inline const char* getObjectString(uint ID) {
  std::stringstream ss;
  ss <<"o" <<ID;
  return ss.str().c_str();
}

arr q0, W;

void drawOrsActionInterfaceEnv(void*) {
  glStandardLight(nullptr);
  glDrawFloor(4., 1, 1, 1);
}

void oneStep(const arr& q, rai::Configuration* C, OdeInterface* ode, SwiftInterface* swift) {
  C->setJointState(q);
#ifdef RAI_ODE
  if(ode) {
    C->ode().exportStateToOde();
    C->ode().step(.01);
    C->ode().importStateFromOde();
    //C->ode().importProxiesFromOde(*C);
    //C->getJointState(q);
  }
#endif
  if(swift) {
    swift->step(*C);
  } else {
#ifdef RAI_ODE
    C->ode().importProxiesFromOde();
#endif
  }

}

void controlledStep(arr& q, arr& W, rai::Configuration* C, OdeInterface* ode, SwiftInterface* swift, TaskVariableList& TVs) {
  static arr dq;
  updateState(TVs, *C);
  updateChanges(TVs); //computeXchangeWithAttractor(globalSpace);
  bayesianControl(TVs, dq, W);
  q += dq;
  oneStep(q, C, ode, swift);
}

ActionInterface::ActionInterface() {
  C=0;
  gl=0;
  ode=0;
  swift=0;

  Tabort = SEC_ACTION_ABORT;
}

ActionInterface::~ActionInterface() {
  shutdownAll();
}

void ActionInterface::shutdownAll() {
  if(C) delete C;          C=0;
  if(gl) delete gl;        gl=0;
#ifdef RAI_ODE
  if(ode) delete ode;      ode=0;
#endif
  if(swift) delete swift;  swift=0;
}

void ActionInterface::loadConfiguration(const char* ors_filename) {

//  char *path, *name, cwd[200];
//  rai::decomposeFilename(path, name, ors_filename);
//  getcwd(cwd, 200);
//  chdir(path);

  if(C) delete C;
  C = new rai::Configuration();
  FILE(ors_filename) >> *C;
  //C->reconfigureRoot(C->getName("rfoot"));

  C->getJointState(q0);

  //compute generic q-metric depending on tree depth
  uint i;
  arr BM(C->bodies.N);
  BM=1.;
  for(i=BM.N; i--;) {
    if(C->bodies(i)->children.N) {
      BM(i) += BM(C->bodies(i)->children(0)->to->index);
    }
  }
  arr Wdiag(q0.N);
  for(i=0; i<q0.N; i++) Wdiag(i)=BM(C->joints(i)->to->index);
  //cout <<Wdiag;
  //Wdiag <<"[20 20 20 10 10 10 10 1 1 1 1 10 10 1 1 1 1 1 1 1 1 1 1 1 1 1 1 1 1
// 1 1 20 20 10 10 10 10 10 10 ]";
  W.setDiag(Wdiag);

  // determine number of objects
  noObjects = 0;
  // assuming that all objects start with "o"
  std::stringstream ss;
  for(i=1;; i++) {
    ss.str("");
    ss <<"o" <<i;
    rai::Body* n = C->getBodyByName(ss.str().c_str());
    if(n==0)
      break;
    noObjects++;
  }

  if(gl) return;
  gl=new OpenGL;
  gl->add(drawOrsActionInterfaceEnv, 0);
  gl->add(rai::glDrawGraph, C);
  //gl->setClearColors(1., 1., 1., 1.);
  gl->camera.setPosition(7., -0., 2.);
  gl->camera.focus(0, 0, .8);
  gl->camera.upright();
  //gl->camera.setPosition(5., -7., 4.);
  //gl->camera.focus(0, -.3, .5);
  //gl->resize(800, 800);
  gl->update();
}

void ActionInterface::watch() {
  gl->text.clear() <<"watch" <<endl;
  gl->watch();
}

void ActionInterface::startOde(double ode_coll_bounce, double ode_coll_erp,
                               double ode_coll_cfm, double ode_friction) {
  CHECK(C, "load a configuration first");
#ifdef RAI_ODE
  C->ode();

  // SIMULATOR PARAMETER
  C->ode().coll_bounce = ode_coll_bounce; // huepfen der bloecke, falls sie zb runterfallen
  C->ode().coll_ERP = ode_coll_erp;   //usually .2!! stiffness (time-scale of contact reaction) umso groesser, desto mehr Fehlerkorrektur; muss zwischen 0.1 und 0.5 sein (ungefaehr)
  C->ode().coll_CFM = ode_coll_cfm;  //softness // umso groesser, desto breiter, desto weicher, desto weniger Fehlerkorrektur; zwischen 10e-10 und 10e5
  C->ode().friction = ode_friction;  //alternative: dInfinity;
//  C->ode().createOde(*C);
#endif
}

void ActionInterface::startSwift() {
  if(swift) delete swift;
  swift = new SwiftInterface(*C);
}

/*void ActionInterface::startSchunk(){
  if(schunk) delete schunk;
  schunk = new SchunkModule;

  createSwift(*C, *swift);
}*/

void ActionInterface::startIBDS() {
  NIY;
}

void ActionInterface::simulate(uint t) {
  arr q;
  C->getJointState(q);
  for(; t--;) {
    oneStep(q, C, ode, swift);
    gl->text.clear() <<"simulation -- time " <<t <<endl;
    gl->update();
  }
}

void ActionInterface::relaxPosition() {
  arr q, dq;
  C->getJointState(q);

  arr I(q.N, q.N); I.setId();

  DefaultTaskVariable x("full state", *C, qLinearTVT, 0, 0, 0, 0, I);
  x.setGainsAsAttractor(20, .1);
  x.y_prec=1000.;
  x.y_target=q0;

//   /*DefaultTaskVariable c("collision", *C, collTVT, 0, 0, 0, 0, arr());*/
//   c.setGainsAsAttractor(20, .1);
//   c.y_prec=10000.;
//   c.state_tol=.005;
//   if(!swift) c.active=false;

  uint t;
  for(t=0; t<Tabort; t++) {
    controlledStep(q, W, C, ode, swift, TVs);
    gl->text.clear() <<"relaxPosition --  time " <<t <<endl;
    gl->update();
    if(x.err<.2) break;
  }
  if(t==Tabort) { indicateFailure(); return; }
}

// void ActionInterface::catchObject(const char *man_id, const char *obj_id){
//   DefaultTaskVariable x("endeffector", *C, posTVT, man_id, 0, 0, 0, arr());
//   x.setGainsAsAttractor(20, .2);
//   x.y_prec=1000.;
//   rai::Configuration::node obj=C->getName(obj_id);
//
//   uint t;
//   arr q, dq;
//   C->getJointState(q);
//   for(t=0;;t++){
//     x.y_target.setCarray(obj->X.p.v, 3);
//     controlledStep(q, W, C, ode, swift, TVs);
//     gl->text.clear() <<"catchObject --  time " <<t <<endl;
//     gl->update();
//     if(x.state==1 || C->getContact(x.i, obj->index)) break;
//   }
//
//   C->glueBodies(C->bodies(x.i), obj);
// }
//
// void ActionInterface::catchObject(uint ID){
//   catchObject("fing1c", convertObjectID2name(ID));
// }
//
// void ActionInterface::catchObject(const char* obj){
//   catchObject("fing1c", obj);
// }

void ActionInterface::moveTo(const char* man_id, const arr& target) {
  DefaultTaskVariable x("endeffector", *C, posTVT, man_id, 0, 0, 0, arr());
  x.setGainsAsAttractor(20, .2);
  x.y_prec=1000.;

  uint t;
  arr q, dq;
  C->getJointState(q);
  for(t=0; t<Tabort; t++) {
    x.y_target=target;
    controlledStep(q, W, C, ode, swift, TVs);
    gl->text.clear() <<"catchObject --  time " <<t <<endl;
    gl->update();
    if(x.err<.05) break;
  }
  if(t==Tabort) { indicateFailure(); return; }
}

void ActionInterface::grab(const char* man_id, const char* obj_id) {
  rai::Body* obj=C->getBodyByName(obj_id);

  DefaultTaskVariable x("endeffector", *C, posTVT, man_id, 0, 0, 0, arr());
  x.setGainsAsAttractor(20, .2);
  x.y_prec=1000.;

//   DefaultTaskVariable c("collision", *C, collTVT, 0, 0, 0, 0, arr());
//   c.setGainsAsAttractor(20, .1);
//   c.y_prec=10000.;
//   c.state_tol=.005;
//   if(!swift) c.active=false;

  // (1) drop object if one is in hand
  for_list(rai::Joint,  e,  C->bodies(x.i)->children) {
    NIY;
    //C->del_edge(e);
  }

  // (2) move towards new object
  uint t;
  arr q, dq;
  C->getJointState(q);
  for(t=0; t<Tabort; t++) {
    x.y_target = conv_vec2arr(obj->X.pos);
    controlledStep(q, W, C, ode, swift, TVs);
    gl->text.clear() <<"catchObject --  time " <<t <<endl;
    gl->update();
    if(x.err<.05 || C->getContact(x.i, obj->index)) break;
  }
  if(t==Tabort) { indicateFailure(); return; }

  // (3) grasp if not table or world
  if(obj->index!=getTableID()) {
    C->glueBodies(C->bodies(x.i), obj);
  } else {
    //indicateFailure()?
  }

  // (4) move upwards (to avoid collisions)
  for(t=0; t<Tabort; t++) {
    x.y_target = conv_vec2arr(obj->X.pos);
    x.y_target(2) = 1.2;
    controlledStep(q, W, C, ode, swift, TVs);
    gl->text.clear() <<"catchObject --  time " <<t <<endl;
    gl->update();
    if(x.err<.05) break;
  }
  if(t==Tabort) { indicateFailure(); return; }
}

void ActionInterface::grab(uint ID) {
  grab("fing1c", convertObjectID2name(ID));
}

void ActionInterface::grab(const char* obj) {
  grab("fing1c", obj);
}

void ActionInterface::dropObjectAbove(const char* obj_id55, const char* rel_id) {
  arr I(q0.N, q0.N); I.setId();
  bool obj_is_inhand = strlen(obj_id55) > 0;
  char* obj_id1;
  if(obj_is_inhand) {
    obj_id1 = new char[strlen(obj_id55)];
    strcpy(obj_id1, obj_id55);
  } else
    obj_id1 = (char*) "fing1c";

  DefaultTaskVariable x("obj", *C, posTVT, obj_id1, 0, 0, 0, arr());
  DefaultTaskVariable z;
  //
  int obj_index=C->getBodyByName(obj_id1)->index;
  delete[] obj_id1;
  rai::Quaternion rot;
  rot = C->bodies(obj_index)->X.rot;
  rai::Vector upvec; double maxz=-2;
  if((rot*Vector_x).z>maxz) { upvec=Vector_x; maxz=(rot*upvec).z; }
  if((rot*Vector_y).z>maxz) { upvec=Vector_y; maxz=(rot*upvec).z; }
  if((rot*Vector_z).z>maxz) { upvec=Vector_z; maxz=(rot*upvec).z; }
  if((rot*(-Vector_x)).z>maxz) { upvec=-Vector_x; maxz=(rot*upvec).z; }
  if((rot*(-Vector_y)).z>maxz) { upvec=-Vector_y; maxz=(rot*upvec).z; }
  if((rot*(-Vector_z)).z>maxz) { upvec=-Vector_z; maxz=(rot*upvec).z; }
  rai::Transformation f;
  f.rot.setDiff(Vector_z, upvec);
  z.set("obj-z-align", *C, zalignTVT, obj_index, f, -1, Transformation_Id, arr());
  //
  DefaultTaskVariable r("full state", *C, qLinearTVT, 0, 0, 0, 0, I);
  DefaultTaskVariable c("collision", *C, collTVT, 0, 0, 0, 0, arr());

  r.setGainsAsAttractor(50, .1);
  r.y_prec=1.;
  r.y_target=q0;
  r.active=false;
  x.setGainsAsAttractor(20, .2);
  x.y_prec=1000.;
  z.setGainsAsAttractor(20, .2);
  z.y_prec=1000.;
  z.y_target.resize(1);  z.y_target = 1.;

  c.setGainsAsAttractor(20, .1);
  c.y_prec=10000.;
  if(!swift) c.active=false;

  uint t;
  arr q, dq;
  C->getJointState(q);

  // tl, 02 july 08
  // Noise for puton position
  double x_noise, y_noise;
  // object dependent noise [START]
//   double* relObj_shape = getShape(convertObjectName2ID(rel_id));
//   double std_dev_noise = DROP_TARGET_NOISE * relObj_shape[0];
  // object dependent noise [END]
  // hard noise [START]
  double std_dev_noise;
  if(convertObjectName2ID(rel_id) == getTableID()) {
    std_dev_noise = DROP_TARGET_NOISE * 0.7;
    uint tries = 0;
    while(true) {
      tries++;
      if(tries>1000)
        HALT("Can't find empty position on table")
        x_noise = std_dev_noise * rnd.gauss();
      y_noise = std_dev_noise * rnd.gauss();
      if(x_noise>0.5 || y_noise>0.5) // stay on table
        continue;
      if(freePosition(C->getBodyByName(rel_id)->X.pos.x+x_noise, C->getBodyByName(rel_id)->X.pos.y+y_noise, 0.05))
        break;
    }
  } else {
    std_dev_noise = DROP_TARGET_NOISE * 0.06;
    x_noise = std_dev_noise * rnd.gauss();
    y_noise = std_dev_noise * rnd.gauss();
  }
  // hard noise [END]

  //phase 1: up
  updateState(TVs, *C);
  x.y_target(2) += .3;
  for(t=0; t<Tabort; t++) {
    //x.y_target.setCarray(C->getBodyByName(rel_id)->X.p.v, 3);
    //x.y_target(2) += .3;
    controlledStep(q, W, C, ode, swift, TVs);
    gl->text.clear() <<"dropObject --  time " <<t <<endl;
    gl->update();
    if(x.err<.05) break;
  }
  if(t==Tabort) { indicateFailure(); return; }

  //phase 2: above object
  double HARD_LIMIT_DIST_Y = -0.8;

  double z_target;
  for(t=0; t<Tabort; t++) {
    x.y_target = conv_vec2arr(C->getBodyByName(rel_id)->X.pos);
    // BRING IN NOISE HERE
    x.y_target(0) += x_noise; // tl
    x.y_target(1) += y_noise; // tl
    // WHERE TO GO ABOVE
    z_target = highestPosition(x.y_target(0), x.y_target(1), 0.06, obj_index); // tl
    // hard limit on y-distance to robot
    if(x.y_target(1) < HARD_LIMIT_DIST_Y)
      x.y_target(1) = HARD_LIMIT_DIST_Y;
    x.y_target(2) = z_target + .2; // distance in m
    controlledStep(q, W, C, ode, swift, TVs);
    gl->text.clear() <<"catchObject --  time " <<t <<endl;
    gl->update();
    if(x.err<.05) break;
  }
  if(t==Tabort) { indicateFailure(); return; }

  //turn off collision avoidance
  c.active=false;

  //phase 3: down
  double* obj_shape = getShape(obj_index);
  for(t=0; t<Tabort; t++) {
    x.y_target = conv_vec2arr(C->getBodyByName(rel_id)->X.pos);
    // BRING IN NOISE HERE
    x.y_target(0) += x_noise; // tl
    x.y_target(1) += y_noise; // tl
    // WHERE TO GO ABOVE
    z_target = highestPosition(x.y_target(0), x.y_target(1), 0.06, obj_index); // tl
    // hard limit on y-distance to robot
    if(x.y_target(1) < HARD_LIMIT_DIST_Y)
      x.y_target(1) = HARD_LIMIT_DIST_Y;
    // IMPORTANT PARAM: set distance to target (relative height-distance in which "hand is opened" / object let loose)
    double Z_ADD_DIST = obj_shape[0]/2 + .05;
    x.y_target(2) = z_target + Z_ADD_DIST; // distance in m where obj is let loose
    controlledStep(q, W, C, ode, swift, TVs);
    gl->text.clear() <<"catchObject --  time " <<t <<endl;
    gl->update();
    if(x.err<.002 && z.err<.002) break;
  }
  if(t==Tabort) { indicateFailure(); return; }

  if(obj_is_inhand) {
    NIY;
    //rai::Joint *e=C->bodies(x.i)->inLinks(0);
    //C->del_edge(e); //otherwise: no object in hand
  }
}

void ActionInterface::dropObjectAbove(uint obj_id, uint rel_id) {
  dropObjectAbove(convertObjectID2name(obj_id), convertObjectID2name(rel_id));
}

void ActionInterface::dropObjectAbove(uint rel_id) {
  dropObjectAbove(getCatched(), rel_id);
}

bool ActionInterface::partOfBody(uint id) {
  NIY;
  return false;
}

uint ActionInterface::getCatched(uint man_id) {
#if 0
  //   rai::Configuration::node n = C->bodies(man_id);
  rai::Proxy* p;
  //  cout <<"davor";
  uint obj=C->getBodyByName(convertObjectID2name(man_id))->index;
  //   cout <<"danach";
  uint i;
  //   cout <<obj <<std::flush;
  //
  for(i=0; i<C->proxies.N; i++)
    if(!C->proxies(i).age && C->proxies(i).d<0.) {
      p=&C->proxies(i);
      //      cout <<"DOES THIS EVER HAPPEN?" <<endl;
      if(p->a==(int)obj && p->b!=(int)obj) {
        // TODO look only for objects "o"
        return p->b;
        //        cout <<"!!!!!!!!!" <<C->bodies(p->b)->name <<" and " <<C->bodies(p->a)->name <<std::flush <<endl;
      }
      if(p->b==(int)obj && p->a!=(int)obj) {
        // look only for objects "o"
        return p->a;
        //        cout <<"!!!!!!!!!" <<C->bodies(p->a)->name <<" and " <<C->bodies(p->b)->name <<std::flush <<endl;
      }
    }
  return UINT_MAX;
#else
  rai::Joint* e;
  e=C->bodies(man_id)->children(0);
  if(!e) return UINT_MAX;
  return e->to->index;
#endif
}

uint ActionInterface::getCatched() {
  return getCatched(convertObjectName2ID("fing1c"));
}

void ActionInterface::writeAllContacts(uint id) {
  rai::Proxy* p;
  //  cout <<"davor";
  uint obj=C->getBodyByName(convertObjectID2name(id))->index;
  //   cout <<"danach";
  uint i;
  //   cout <<obj <<std::flush;
  cout <<convertObjectID2name(id) <<" is in contact with ";
  for(i=0; i<C->proxies.N; i++)
    if(C->proxies(i)->d<0.) {
      p=C->proxies(i);
      //      cout <<"DOES THIS EVER HAPPEN?" <<endl;
      if(p->a==(int)obj && p->b!=(int)obj) {
        // TODO look only for objects "o"
        cout <<C->bodies(p->b)->name <<" ";
        //        cout <<"!!!!!!!!!" <<C->bodies(p->b)->name <<" and " <<C->bodies(p->a)->name <<std::flush <<endl;
      }
      if(p->b==(int)obj && p->a!=(int)obj) {
        // look only for objects "o"
        cout <<C->bodies(p->a)->name <<" ";
      }
    }
  cout <<endl;
}

void ActionInterface::getObjectsAbove(uintA& list, const char* obj_id) {
  list.clear();
  rai::Proxy* p;
  uint obj=C->getBodyByName(obj_id)->index;
//   writeAllContacts(convertObjectName2ID(obj_id));
  uint i;
  double TOL_COEFF = 0.9;
  double obj_rad = 0.5 * getShape(convertObjectName2ID(obj_id))[2];
  double other_rad;
  double dist;
  for(i=0; i<C->proxies.N; i++) if(C->proxies(i)->d<0.) {
      p=C->proxies(i);
      if(p->b == -1 || p->a == -1) // on bottom
        continue;
      if(p->a==(int)obj) {
        other_rad = 0.5 * getShape(p->b)[2];
        dist = TOL_COEFF * (obj_rad + other_rad);
        if(C->bodies(p->b)->X.pos.z - C->bodies(obj)->X.pos.z > dist)
          list.setAppend(p->b);
      } else if(p->b==(int)obj) {
        other_rad = 0.5 * getShape(p->a)[2];
        dist = TOL_COEFF * (obj_rad + other_rad);
        if(C->bodies(p->a)->X.pos.z - C->bodies(obj)->X.pos.z > dist)
          list.setAppend(p->a);
      }
    }
}

void ActionInterface::getObjectsAbove(uintA& list, const uint obj_id) {
  getObjectsAbove(list, convertObjectID2name(obj_id));
}

// void ActionInterface::getObjectsBelow(uintA& list, const char *obj_id){
//   list.clear();
//   rai::Proxy *p;
//   uint obj=C->getBodyByName(obj_id)->index;
//   uint i;
//   for(i=0;i<C->proxies.N;i++) if(!C->proxies(i).age && C->proxies(i).d<0.){
//     p=&C->proxies(i);
//     if(p->a==(int)obj && C->bodies(obj)->X.p(2)>C->bodies(p->b)->X.p(2)){
//       if(list.contains(p->b) < 0)
//         list.append(p->b);
//     }
//     if(p->b==(int)obj && C->bodies(obj)->X.p(2)>C->bodies(p->a)->X.p(2)){
//       if(list.contains(p->a) < 0)
//         list.append(p->a);
//     }
//   }
// }
//
// void ActionInterface::getObjectsBelow(uintA& list, const uint obj_id){
//   getObjectsBelow(list, convertObjectID2name(obj_id));
// }

void ActionInterface::getObjects(uintA& objects) { ///< return list all objects
  objects.clear();
  getManipulableObjects(objects);
  objects.append(getTableID());
}

void ActionInterface::getManipulableObjects(uintA& objects) {
  objects.clear();
  // assuming that all objects start with "o"
  std::stringstream ss;
  uint i, obj;
  for(i=1; i<=noObjects; i++) {
    ss.str("");
    ss <<"o" <<i;
    rai::Body* n = C->getBodyByName(ss.str().c_str());
    obj=n->index;
    objects.append(obj);
  }
}

uint ActionInterface::getTableID() {
  rai::Body* n = C->getBodyByName("table");
  return n->index;
}

bool ActionInterface::inContact(uint a, uint b) {
  if(C->getContact(a, b)) return true;
  return false;
}

bool ActionInterface::isUpright(uint id) {
  double TOLERANCE = 0.05; // in radians

  rai::Quaternion rot;
  rot = C->bodies(id)->X.rot;
  rai::Vector upvec; double maxz=-2;
  if((rot*Vector_x).z>maxz) { upvec=Vector_x; maxz=(rot*upvec).z; }
  if((rot*Vector_y).z>maxz) { upvec=Vector_y; maxz=(rot*upvec).z; }
  if((rot*Vector_z).z>maxz) { upvec=Vector_z; maxz=(rot*upvec).z; }
  if((rot*(-Vector_x)).z>maxz) { upvec=-Vector_x; maxz=(rot*upvec).z; }
  if((rot*(-Vector_y)).z>maxz) { upvec=-Vector_y; maxz=(rot*upvec).z; }
  if((rot*(-Vector_z)).z>maxz) { upvec=-Vector_z; maxz=(rot*upvec).z; }
  double angle;
  angle = acos(maxz);

//   cout <<id <<" angle = " <<angle <<endl;
  if(fabs(angle) < TOLERANCE)
    return true;
  else
    return false;
}

uint ActionInterface::convertObjectName2ID(const char* name) {
  return C->getBodyByName(name)->index;
}

const char* ActionInterface::convertObjectID2name(uint ID) {
  if(C->bodies.N > ID)
    return C->bodies(ID)->name;
  else
    return "";
}

// object = Body in kin.h
int ActionInterface::getType(uint id) {
  //  Body* corp = C->bodies(id);
  //  int q = corp->type;
  return C->bodies(id)->shapes(0)->type;
}

double* ActionInterface::getShape(uint id) {
  return C->bodies(id)->shapes(0)->size;
}

double* ActionInterface::getColor(uint id) {
  return C->bodies(id)->shapes(0)->color;
}

double* ActionInterface::getPosition(uint id) {
  return C->bodies(id)->X.pos.p();
}

// void ActionInterface::printAboveBelowInfos(){
//   uintA objects;
//   getManipulableObjects(objects);
//   uintA objects2;
//   uint i;
//   FOR1D(objects, i){
//     cout <<convertObjectID2name(objects(i)) <<" (" <<objects(i) <<"): " <<std::flush;
//     getObjectsAbove(objects2, objects(i));
//     cout <<"above=" <<objects2 <<" " <<std::flush;
//     getObjectsBelow(objects2, objects(i));
//     cout <<"below=" <<objects2 <<" ";
//     cout <<endl;
//   }
// }

void ActionInterface::printObjectInfo() {
  uintA objects;
  getObjects(objects);
  uint i;
  FOR1D(objects, i) {
    cout <<objects(i) <<" " <<convertObjectID2name(objects(i)) <<endl;
  }
}

void ActionInterface::indicateFailure() {
  // drop object
  for_list(rai::Joint,  e,  C->getBodyByName("fing1c")->children) {
    NIY;
    //C->del_edge(e); //otherwise: no object in hand
  }
  cout <<"ActionInterface: CONTROL FAILURE" <<endl;
  relaxPosition();
}

// if z-value of objects is beneath THRESHOLD
bool ActionInterface::onBottom(uint id) {
  double THRESHOLD = 0.15;
  rai::Body* obj=C->bodies(id);
  if(obj->X.pos.z < THRESHOLD)
    return true;
  else
    return false;
}

void ActionInterface::getObservableObjects(uintA& objs) {
  getManipulableObjects(objs);
  objs.setAppend(getTableID());
  uint i;
  FOR1D_DOWN(objs, i) {
    if(onBottom(objs(i)))
      objs.remove(i);
  }
}

bool ActionInterface::freePosition(double x, double y, double radius) {
  uintA manipObjs;
  getManipulableObjects(manipObjs);
//     cout <<"Asking for pos " <<x <<"/" <<y <<" within radius " <<radius<<endl;
  uint i;
  FOR1D(manipObjs, i) {
    double* pos = getPosition(manipObjs(i));
//         cout <<manipObjs(i) <<" has pos " <<pos[0] <<"/" <<pos[1] <<endl;
    if(fabs(pos[0] - x) < radius)
      return false;
    if(fabs(pos[1] - y) < radius)
      return false;
  }
  return true;
}
double ActionInterface::highestPosition(double x, double y, double radius, uint id_ignored) {
  uint DEBUG = 0;
  uintA manipObjs;
  getManipulableObjects(manipObjs);
  if(DEBUG>0) {
    cout <<"highestPosition:" <<endl;
    cout <<"Asking for pos " <<x <<"/" <<y <<" within radius " <<radius<<endl;
  }
  uint i;
  double max_z;
  double* table_pos = getPosition(getTableID());
  max_z = table_pos[2];
  if(DEBUG>0) {cout <<"table_z: " <<max_z <<endl;}
  FOR1D(manipObjs, i) {
    if(manipObjs(i) == id_ignored)
      continue;
    double* pos = getPosition(manipObjs(i));
//         cout <<manipObjs(i) <<" has pos " <<pos[0] <<"/" <<pos[1] <<endl;
    if(fabs(pos[0] - x) < radius  &&  fabs(pos[1] - y) < radius) {
      if(DEBUG>0) cout <<"Object " <<manipObjs(i) <<" within radius at height " <<pos[2] <<endl;
      if(pos[2] > max_z)
        max_z = pos[2];
    }
  }
  if(DEBUG>0) cout <<"max_z = " <<max_z <<endl;
  return max_z;
}
