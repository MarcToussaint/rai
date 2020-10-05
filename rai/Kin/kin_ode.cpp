/*  ------------------------------------------------------------------
    Copyright (c) 2011-2020 Marc Toussaint
    email: toussaint@tu-berlin.de

    This code is distributed under the MIT License.
    Please see <root-path>/LICENSE for details.
    --------------------------------------------------------------  */

#include "kin_ode.h"

#ifdef RAI_ODE

#ifndef dDOUBLE
#  define dDOUBLE
#endif

#  include <ode/ode.h>
#  include <ode/internal/objects.h>
#  include <ode/internal/joints/joints.h>
#  include <ode/internal/collision_kernel.h>
#  include <ode/internal/collision_transform.h>

#  ifdef RAI_MSVC
#    undef HAVE_UNISTD_H
#    undef HAVE_SYS_TIME_H
#  endif

#define OUTs(x) x <<' '
#define OUTv(x) '(' <<OUTs(x[0]) <<OUTs(x[1]) <<OUTs(x[2]) <<')'
#define OUTq(x) OUTs(x[0]) <<OUTs(x[1]) <<OUTs(x[2]) <<OUTs(x[3])
#define CP4(x, y) memmove(x, y, 4*sizeof(double));
#define CP3(x, y) memmove(x, y, 3*sizeof(double));

static bool ODEinitialized=false;

//===========================================================================
//
// Ode implementations
//

OdeInterface::OdeInterface(rai::Configuration& _C):C(_C) {
  time=0.;

  noGravity=noContactJoints=false;

  ERP= 0.2;     //in [0, 1]: rate of error correction (makes more brittle) [default=.2]
  CFM=1e-5;  // >0: softness (makes more robust) [default=1e-10]

  coll_bounce = .0;
  coll_ERP = 0.2;   //usually .2!! stiffness (time-scale of contact reaction)
  coll_CFM = 1e-5;  //softness
  friction = 0.1;   //alternative: dInfinity;

  world=nullptr;
  space=nullptr;
  contactgroup=0;

  if(!ODEinitialized) {  dInitODE();  ODEinitialized=true; }
  clear();

  dBodyID b;
  dMass odeMass;
  dGeomID geom, trans;
  dSpaceID myspace;
  dxJointHinge* jointH=0;
  dxJointSlider* jointS=0;
  dxJointUniversal* jointU=0;
  dxJointFixed* jointF=0;
  dxJointAMotor* jointM=0;
  dJointFeedback* jointFB=0;
  rai::Vector a;
  //double *mass, *shape, *type, *fixed, *cont, typeD=ST_capsule;
  //, *inertiaTensor, *realMass, *centerOfMass;

  clear();

  bodies.resize(C.bodies.N); bodies=0;
  geoms .resize(C.shapes.N); geoms =0;
  joints.resize(C.joints.N); joints=0;
  motors.resize(C.joints.N); motors=0;

  for_list(rai::Body,  n,  C.bodies) {
    b=dBodyCreate(world);

    bodies(n->index)=b;
    b->userdata=n;

    //n->copyFrameToOde();
    CHECK(n->X.rot.isNormalized(), "quaternion is not normalized!");
    CP3(b->posr.pos, n->X.pos.p());                // dxBody changed in ode-0.6 ! 14. Jun 06 (hh)
    CP4(b->q, n->X.rot.p()); dQtoR(b->q, b->posr.R);
    CP3(b->lvel, n->X.vel.p());
    CP3(b->avel, n->X.angvel.p());

    // need to fix: "mass" is not a mass but a density (in dMassSetBox)
    for_list(rai::Shape,  s,  n->shapes) {
      if(!(s->rel.rot.isZero) || !(s->rel.pos.isZero)) { //we need a relative transformation
        trans = dCreateGeomTransform(space);
        myspace = nullptr; //the object is added to no space, but (below) associated with the transform
      } else {
        trans = nullptr;
        myspace = space; //the object is added normally to the main space
      }

      switch(s->type) {
        default:
          for(int i=0; i<3; ++i) {
            if(s->size[i] == 0) s->size[i] = 0.001;
          }
        case rai::ST_box:
          dMassSetBox(&odeMass, n->mass, s->size(0), s->size(1), s->size(2));
          dBodySetMass(b, &odeMass);
          geom=dCreateBox(myspace, s->size(0), s->size(1), s->size(2));
          break;
        case rai::ST_sphere:
          dMassSetSphere(&odeMass, n->mass, s->size(-1));
          dBodySetMass(b, &odeMass);
          geom=dCreateSphere(myspace, s->size(-1));
          break;
        case rai::ST_cylinder:
          dMassSetCylinder(&odeMass, n->mass, 3, s->size(-1), s->size(-2));
          dBodySetMass(b, &odeMass);
          geom=dCreateCylinder(myspace, s->size(-1), s->size(-2));
          break;
        case rai::ST_capsule:
          dMassSetCylinder(&odeMass, n->mass, 3, s->size(-1), s->size(-2));
          //                 RAI_MSG("ODE: setting Cylinder instead of capped cylinder mass");
          dBodySetMass(b, &odeMass);
          geom=dCreateCCylinder(myspace, s->size(-1), s->size(-2));
          break;
        case rai::ST_mesh: {
#if 0
          NIY;
#else
          s->mesh.computeNormals();
          // get inertia tensor and REAL mass (careful no density)
          //n->ats.get("I", inertiaTensor, 9);
          //n->ats.get("w", realMass, 1);
          //n->ats.get("X", centerOfMass, 3);

          // transform the mesh to ODE trimesh format;
          //i=0; j=0;

#if 0 //correct mass/density stuff
          trimeshPhysics triPhys;
          triPhys.reset(s->mesh.T.N);
          if(inertiaTensor && realMass && centerOfMass) { // are all important params set in the dcg file
            triPhys._mass = *realMass;
            triPhys.r[0]=centerOfMass[0];
            triPhys.r[1]=centerOfMass[1];
            triPhys.r[2]=centerOfMass[2];
            triPhys.J[0][0]= inertiaTensor[0];
            triPhys.J[1][1]= inertiaTensor[4];
            triPhys.J[2][2]= inertiaTensor[8];
            triPhys.J[0][1]= inertiaTensor[1];
            triPhys.J[0][2]= inertiaTensor[2];
            triPhys.J[1][2]= inertiaTensor[5];
          } else { // not all parametrs specified in dcg file....need to calculate them
            triPhys.calculateODEparams(&s->mesh, s->mass); // note: 2nd parameter is the density
          }

          dMassSetParameters(&odeMass, triPhys._mass,
                             triPhys.r[0], triPhys.r[1], triPhys.r[2],
                             triPhys.J[0][0], triPhys.J[1][1], triPhys.J[2][2],
                             triPhys.J[0][1], triPhys.J[0][2], triPhys.J[1][2]);

          dBodySetMass(b, &odeMass);
#else //don't care about mass...
          n->mass = .001;
          dMassSetBox(&odeMass, n->mass, s->size(0), s->size(1), s->size(2));
          dBodySetMass(b, &odeMass);
#endif

          dTriMeshDataID TriData;
          TriData = dGeomTriMeshDataCreate();
          dGeomTriMeshDataBuildDouble(TriData,
                                      s->mesh.V.p, 3*sizeof(double), s->mesh.V.d0,
                                      s->mesh.T.p, s->mesh.T.d0, 3*sizeof(uint));
          dGeomTriMeshDataPreprocess(TriData);

          geom = dCreateTriMesh(myspace, TriData, 0, 0, 0);

          dGeomTriMeshClearTCCache(geom);
#endif
        } break; //end of mesh
      }

      geoms(s->index) = geom;
      if(trans) {
        //geoms(s->index) = trans;
        dGeomTransformSetGeom(trans, geom);
        dGeomSetPosition(geom, s->rel.pos.x, s->rel.pos.y, s->rel.pos.z);
        dGeomSetQuaternion(geom, s->rel.rot.p());
        dGeomSetBody(trans, b); //attaches the geom to the body
      } else {
        dGeomSetBody(geom, b); //attaches the geom to the body
      }
    }//loop through shapes

    if(n->type==rai::BT_static) {
      jointF=(dxJointFixed*)dJointCreateFixed(world, 0);
      dJointAttach(jointF, b, 0);
      dJointSetFixed(jointF);
    }
  }
#ifndef RAI_ode_nojoints
  for(rai::Body* n: C.bodies) {
    for_list(rai::Joint,  e,  n->inLinks) {
      switch(e->type) {
        case rai::JT_rigid:
          jointF=(dxJointFixed*)dJointCreateFixed(world, 0);
          dJointAttach(jointF, bodies(e->from->index), bodies(e->to->index));
          dJointSetFixed(jointF);
          joints(e->index)=jointF;
          //    e->fixed=true;
          break;
        case rai::JT_hingeX:
          jointH=(dxJointHinge*)dJointCreateHinge(world, 0);
          /*if(e->p[1]!=e->p[0]){
            dJointSetHingeParam(jointH, dParamLoStop, e->p[0]);
            dJointSetHingeParam(jointH, dParamHiStop, e->p[1]);

            //dJointSetHingeParam(jointH, dParamCFM, CFM);
            }*/
          dJointAttach(jointH, bodies(e->from->index), bodies(e->to->index));
          joints(e->index)=jointH;
          //e->copyFramesToOdeHinge();
          break;
        case rai::JT_universal:
          jointU=(dxJointUniversal*)dJointCreateUniversal(world, 0);
          dJointAttach(jointU, bodies(e->from->index), bodies(e->to->index));
          joints(e->index)=jointU;
          //e->copyFramesToOdeUniversal();
          break;
        case rai::JT_transX:
          jointS=(dxJointSlider*)dJointCreateSlider(world, 0);
          dJointAttach(jointS, bodies(e->from->index), bodies(e->to->index));
          joints(e->index)=jointS;
          //e->copyFramesToOdeSlider();
          break;
        default: NIY;
      }
      if(e->type==rai::JT_hingeX) {
        jointM = (dxJointAMotor*)dJointCreateAMotor(world, 0);
        dJointSetAMotorNumAxes(jointM, 1);
        dJointAttach(jointM, bodies(e->from->index), bodies(e->to->index));
        motors(e->index)=jointM;
        a=e->from->X.rot*e->A.rot*rai::Vector(1, 0, 0);
        dJointSetAMotorAxis(jointM, 0, 1, a.x, a.y, a.z);
        //dJointSetAMotorParam(jointM, dParamFMax, 1.);
        jointFB = new dJointFeedback;
        dJointSetFeedback(jointM, jointFB);
      }
    }
  }
#endif

  exportStateToOde();
}

OdeInterface::~OdeInterface() {
  if(contactgroup) dJointGroupDestroy(contactgroup);
  if(space) dSpaceDestroy(space);
  if(world) dWorldDestroy(world);
}

void OdeInterface::clear() {
  if(contactgroup) dJointGroupDestroy(contactgroup);
  if(space) dSpaceDestroy(space);
  if(world) dWorldDestroy(world);

  world=dWorldCreate();
  space=dSimpleSpaceCreate(0);
  contactgroup=dJointGroupCreate(0);
  //std::cout <<"default ERP=" <<dWorldGetERP(world) <<"default CFM=" <<dWorldGetCFM(world) <<std::endl;
  if(noGravity) {
    dWorldSetGravity(world, 0, 0, 0);
  } else {
    dWorldSetGravity(world, 0, 0, -9.81);
  }
  dWorldSetCFM(world, CFM);
  dWorldSetERP(world, ERP);
  dWorldSetContactSurfaceLayer(world, .0); //necessary penetration depth to trigger forces..
  double DAMPING_LINEAR_SCALE = 0.03;
  double DAMPING_ANGULAR_SCALE = 0.03;
  dWorldSetDamping(world, DAMPING_LINEAR_SCALE, DAMPING_ANGULAR_SCALE);

  plane0=dCreatePlane(space, 0, 0, 1, 0);

  // removed vertical planes,  5. Mar 06 (hh)
  //planex1=dCreatePlane(space, 1, 0, 0, -10);
  //planex2=dCreatePlane(space, -1, 0, 0, -10);
  //planey1=dCreatePlane(space, 0, 1, 0, -10);
  //planey2=dCreatePlane(space, 0, -1, 0, -10);
}

void OdeInterface::staticCallback(void* classP, dGeomID g1, dGeomID g2) {
  static dContact contacts[100]; // array with maximum number of contacts
  uint i, n;
  dJointID c;

  dBodyID b1 = dGeomGetBody(g1);
  dBodyID b2 = dGeomGetBody(g2);

  rai::Body* db1 = b1?(rai::Body*)b1->userdata:0;
  rai::Body* db2 = b2?(rai::Body*)b2->userdata:0;

  //-- rule out irrelevant contacts
  // exit without doing anything if the geoms stem from the same body
  if(b1==b2) return;

  // exit without doing anything if the two bodies are connected by a joint
  if(b1 && b2 && dAreConnectedExcluding(b1, b2, dJointTypeSlider)) return;

  // exit if fixed body intersects with earth,  4. Mar 06 (hh)
  if(b1==0 && db2->type==rai::BT_static) return;
  if(b2==0 && db1->type==rai::BT_static) return;

  // exit if we have two fixed bodies,  6. Mar 06 (hh)
  if(db1 && db2 && db1->type==rai::BT_static && db2->type==rai::BT_static) return;

  // exit if none of the bodies have cont enabled (mt)
  //if(db1 && !db1->cont && db2 && !db2->cont) return;
  if(db1 && db2 && (!db1->shapes(0)->cont || !db2->shapes(0)->cont)) return;

  //-- ok, now compute the contact exactly
  // get contacts:
  n=dCollide(g1, g2, 100, &(contacts[0].geom), sizeof(dContact));

  if(!n) return;

  for(i=0; i<n; i++)((OdeInterface*)classP)->conts.append(&contacts[i].geom);

  if(((OdeInterface*)classP)->noContactJoints) return;

  for(i=0; i<n; i++) {
    contacts[i].surface.mode = dContactSoftCFM | dContactSoftERP | dContactBounce;
    //if(b1 && b2)
    contacts[i].surface.mu   = ((OdeInterface*)classP)->friction; //dInfinity; //friction
    //else contacts[i].surface.mu  = .01; //dInfinity; //friction
    contacts[i].surface.bounce   = ((OdeInterface*)classP)->coll_bounce;
    contacts[i].surface.soft_erp = ((OdeInterface*)classP)->coll_ERP;   //usually .2!! stiffness (time-scale of contact reaction)
    contacts[i].surface.soft_cfm = ((OdeInterface*)classP)->coll_CFM;  //softness

    if(!db1 || !db2 || (db1->shapes(0)->cont && db2->shapes(0)->cont)) {
      //normal contact:
      c=dJointCreateContact(((OdeInterface*)classP)->world, ((OdeInterface*)classP)->contactgroup, &contacts[i]);
      dJointAttach(c, b1, b2);
    } else {
      //special contact:
      //one of them ignores the contact
      //hence we consider it (kinematically) coupled to the background
      //.. we simulated relative motion for(parallel) friction
      rai::Vector no; no.set(contacts[i].geom.normal);
      rai::Vector d1; d1=db1->X.vel; d1.makeNormal(no); //bug: we should have sorted b1 and b2 before such that b1!=0
      double v=d1.length();
      if(v>1e-4) {
        d1 /= v;
        CP3(contacts[i].fdir1, d1.p());
        contacts[i].surface.mode = contacts[i].surface.mode | dContactMotion1 | dContactFDir1;
        contacts[i].surface.motion1 = v;
      }
      c=dJointCreateContact(((OdeInterface*)classP)->world, ((OdeInterface*)classP)->contactgroup, &contacts[i]);
      if(!db1->shapes(0)->cont) {
        dJointAttach(c, 0, b2);
      }
      if(!db2->shapes(0)->cont) {
        dJointAttach(c, b1, 0);
      }
    }
  }
}

void OdeInterface::setForceFree(bool free) {
  noGravity=free;
  if(free) {
    dWorldSetGravity(world, 0, 0, 0);
    dWorldSetCFM(world, CFM);
    dWorldSetERP(world, ERP);
  } else {
    dWorldSetGravity(world, 0, 0, -9.81);
    dWorldSetCFM(world, CFM);
    dWorldSetERP(world, ERP);
  }
}

void OdeInterface::step(double dtime) {
  conts.clear();
  dSpaceCollide(space, this, &staticCallback);
  //if(noContactJoints) contactForces();
  //dWorldStep(world, dtime);
  //dWorldQuickStep(world, dtime);
  dWorldStep(world, dtime);
  dJointGroupEmpty(contactgroup);
  time+=dtime;
}

void OdeInterface::reportContacts() {
  std::cout <<"contacts: " <<conts.N <<std::endl;
  dContactGeom* c;
  for(uint i=0; i<conts.N; i++) {
    c = conts(i);
    dBodyID b1 = dGeomGetBody(c->g1);
    dBodyID b2 = dGeomGetBody(c->g2);
    rai::Body* db1 = b1?(rai::Body*)b1->userdata:0;
    rai::Body* db2 = b2?(rai::Body*)b2->userdata:0;
    std::cout
        <<i <<": " <<(b1?db1->name.p:"GROUND") <<'-' <<(b2?db2->name.p:"GROUND")
        <<"\nposition=" <<OUTv(c->pos)
        <<"\nnormal=" <<OUTv(c->normal)
        <<"\npenetration depth=" <<OUTs(c->depth)
        <<std::endl;
  }
}

// - compute depth of penetration and normal for
//   body with touch sensor, 6. Mar 06 (hh).
// - fixed bug (I did not assume that each contact is
//   just represented once), 12. Mar 06 (hh).
// - copy penetration data to Body, 22. May 06 (hh)
void OdeInterface::penetration(rai::Vector& p) {
  NIY;
#if 0
  dContactGeom* c;
  double d;
  dBodyID b1, b2;

  for(uint i=0; i<conts.N; i++) {
    c = conts(i);
    b1 = dGeomGetBody(c->g1);
    b2 = dGeomGetBody(c->g2);
    //get body attributes
    double* touch1=0, *touch2=0;
    if(b1) touch1=anyListGet<double>(((rai::Body*)b1->userdata)->ats, "touchsensor", 0);
    if(b2) touch2=anyListGet<double>(((rai::Body*)b2->userdata)->ats, "touchsensor", 0);

    if(touch1 || touch2) {
      d = c->depth;
      p.set(c->normal);
      p *= d;
      if(touch1)((rai::Body*)b1->userdata)->touch = p;
      else((rai::Body*)b2->userdata)->touch = -p;
      //printf("%lf\n", d);
      std::cout <<"contact " <<i <<": penetration: " <<OUTv(p.v) <<"\n";
    }
  }
#endif
}

void OdeInterface::contactForces() {
  uint i;
  dContactGeom* c;
  dBodyID b1, b2;
  rai::Vector pos, normal, v1, v2, vrel;
  dMass mass;
  double force, d, m1, m2;
  for(i=0; i<conts.N; i++) {
    c = conts(i);
    pos.set(c->pos);
    d=c->depth*20.;
    normal.set(c->normal);
    normal *=-1.; //vector from b1 -> b2
    b1 = dGeomGetBody(c->g1);
    b2 = dGeomGetBody(c->g2);
    if(b1) { v1.set(b1->lvel); dBodyGetMass(b1, &mass); m1=mass.mass; } else { v1.setZero(); m1=0.; }
    if(b2) { v2.set(b1->lvel); dBodyGetMass(b2, &mass); m2=mass.mass; } else { v2.setZero(); m2=0.; }

    // ** normal force:
    force=0.;
    vrel=v2-v1; //inaccurate: should also include rotational velocities at contact point!
    vrel.makeColinear(normal);
    //spring force
    force = .03*d*d*d;
    if(b2) dBodyAddForceAtPos(b2, force*normal.x, force*normal.y, force*normal.z, pos.x, pos.y, pos.z);
    force *= -1.; //invert on other body
    if(b1) dBodyAddForceAtPos(b1, force*normal.x, force*normal.y, force*normal.z, pos.x, pos.y, pos.z);

    //inward viscosity
    if(vrel * normal <= 0) {
      force += 10.*vrel.length();
      if(b2) dBodyAddForceAtPos(b2, force*m2*normal.x, force*m2*normal.y, force*m2*normal.z, pos.x, pos.y, pos.z);
      force *= -1.; //invert on other body
      if(b1) dBodyAddForceAtPos(b1, force*m1*normal.x, force*m1*normal.y, force*m1*normal.z, pos.x, pos.y, pos.z);
    }

    if(!b2) std::cout <<"bodyForce = " <<force* normal <<std::endl;

#if 1
    // ** parallel (slip) force:
    force=0.;
    vrel=v2-v1; //inaccurate: should also include rotational velocities at contact point!
    vrel.makeNormal(normal);
    //viscosity
    force += 10.*vrel.length();

    vrel.normalize();
    force *= -1.;
    if(b2) dBodyAddForceAtPos(b2, force*m2*vrel.x, force*m2*vrel.y, force*m2*vrel.z, pos.x, pos.y, pos.z);
    force *= -1.;
    if(b1) dBodyAddForceAtPos(b1, force*m1*vrel.x, force*m1*vrel.y, force*m1*vrel.z, pos.x, pos.y, pos.z);

    if(!b2) std::cout <<"bodyForce = " <<force* normal <<std::endl;
    std::cout <<"slip force " <<force <<std::endl;
#endif
  }
}

//===========================================================================
//
// graph
//

void OdeInterface::exportStateToOde() {
  dBodyID b;

  for_list(rai::Body,  n,  C.bodies) {
    CHECK(n->X.rot.isNormalized(), "quaternion is not normalized!");
    b = bodies(n->index);
    CP3(b->posr.pos, n->X.pos.p());                // dxBody changed in ode-0.6 ! 14. Jun 06 (hh)
    CP4(b->q, n->X.rot.p()); dQtoR(b->q, b->posr.R);
    CP3(b->lvel, n->X.vel.p());
    CP3(b->avel, n->X.angvel.p());
    //n->copyFrameToOde();
#ifndef RAI_ode_nojoints
    for_list(rai::Joint,  e,  n->inLinks) if(e->type!=rai::JT_glue) {
      dxJointHinge* hj=(dxJointHinge*)joints(e->index);
      dxJointUniversal* uj=(dxJointUniversal*)joints(e->index);
      dxJointSlider* sj=(dxJointSlider*)joints(e->index);
      switch(e->type) { //16. Mar 06 (hh)
        case rai::JT_rigid:
          break;
        case rai::JT_hingeX:
          CP4(hj->qrel, (e->A.rot*e->B.rot).p());
          CP3(hj->anchor1, (e->A.pos).p());
          CP3(hj->anchor2, (-(e->B.rot/e->B.pos)).p());
          CP3(hj->axis1, (e->A.rot*rai::Vector(1, 0, 0)).p());
          CP3(hj->axis2, (e->B.rot/rai::Vector(1, 0, 0)).p());
          break;
        case rai::JT_universal:
          CP4(uj->qrel1, (e->A.rot).p());
          CP4(uj->qrel2, (e->B.rot).p());
          CP3(uj->anchor1, (e->A.pos).p());
          CP3(uj->anchor2, (-(e->B.rot/e->B.pos)).p());
          CP3(uj->axis1, (e->A.rot*rai::Vector(1, 0, 0)).p());
          CP3(uj->axis2, (e->B.rot/rai::Vector(1, 0, 0)).p());
          break;
        case rai::JT_transX:
          CP4(sj->qrel, (e->A.rot*e->B.rot).p());
          CP3(sj->offset, (e->Q.pos).p());
          //printf("offset: (%lf; %lf; %lf)\n", (e->X.pos)[0], (e->X.pos)[1], (e->X.pos)[2]);
          CP3(sj->axis1, (e->A.rot*rai::Vector(1, 0, 0)).p());
          break;
        default: NIY;
      }
    }
#endif
  }
}

void OdeInterface::pushPoseForShape(rai::Shape* s) {
  dGeomID geom = geoms(s->index);
  dGeomSetQuaternion(geom, *((dQuaternion*)s->rel.rot.p()));
  dGeomSetPosition(geom, s->rel.pos.x, s->rel.pos.y, s->rel.pos.z);
}

void OdeInterface::exportForcesToOde() {
  dBodyID b;
  for_list(rai::Body,  n,  C.bodies) {
    b=bodies(n->index);
    CP3(b->facc, n->force.p());
    CP3(b->tacc, n->torque.p());
  }
}

void OdeInterface::importStateFromOde() {
  dBodyID b;
  for_list(rai::Body,  n,  C.bodies) {
    //n->getFrameFromOde();
    b=bodies(n->index);
    CP3(n->X.pos.p(), b->posr.pos);
    CP4(n->X.rot.p(), b->q);
    CP3(n->X.vel.p(), b->lvel);
    CP3(n->X.angvel.p(), b->avel);
    CHECK(n->X.rot.isNormalized(), "quaternion is not normalized!");
  }
  C.calc_Q_from_BodyFrames();
  C.calc_fwdPropagateShapeFrames();
}

void OdeInterface::addJointForce(rai::Joint* e, double f1, double f2) {
  switch(e->type) {
    case rai::JT_hingeX:
      dJointAddHingeTorque(joints(e->index), -f1);
      break;
    case rai::JT_rigid: // no torque
      break;
    case rai::JT_universal:
      dJointAddUniversalTorques(joints(e->index), -f1, -f2);
      break;
    case rai::JT_transX:
      dJointAddSliderForce(joints(e->index), -f1);
      break;
    default: NIY;
  }
}

void OdeInterface::addJointForce(doubleA& x) {
  uint n=0;

  for_list(rai::Joint,  e,  C.joints) { // loop over edges, 16. Mar 06 (hh)
    switch(e->type) { //  3. Apr 06 (hh)
      case rai::JT_hingeX:
        dJointAddHingeTorque(joints(e->index), -x(n));
        n++;
        break;
      case rai::JT_rigid: // no torque
        break;
      case rai::JT_universal:
        dJointAddUniversalTorques(joints(e->index), -x(n), -x(n+1));
        n+=2;
        break;
      case rai::JT_transX:
        dJointAddSliderForce(joints(e->index), -x(n));
        n++;
        break;
      default: NIY;
    }
  }
  CHECK_EQ(n, x.N, "wrong dimensionality");
}

void OdeInterface::setMotorVel(const arr& qdot, double maxF) {
  uint n=0;

  for_list(rai::Joint,  e,  C.joints) {
    switch(e->type) {
      case rai::JT_hingeX:
        dJointSetAMotorParam(motors(e->index), dParamVel, -qdot(e->qIndex));
        dJointSetAMotorParam(motors(e->index), dParamFMax, maxF);
        n++;
        break;
      case rai::JT_rigid:
        break;
      case rai::JT_universal:
        n+=2;
        break;
      case rai::JT_transX:
        n++;
        break;
      default: NIY;
    }
  }
  CHECK_EQ(n, qdot.N, "wrong dimensionality");
}

uint OdeInterface::getJointMotorDimension() {
  NIY;
  uint i=0;
  for_list(rai::Body,  n,  C.bodies) {
    for_list(rai::Joint,  e,  n->inLinks) { // if(!e->fixed && e->motor){ // 16. Mar 06 (hh)
      if(e->type==rai::JT_universal) i+=2;
      else i++;
    }
  }
  return i;
}

void OdeInterface::setJointMotorPos(rai::Joint* e, double x0, double maxF, double tau) {
  double x=-dJointGetHingeAngle(joints(e->index));
  setJointMotorVel(e, .1*(x0-x)/tau, maxF);
}

void OdeInterface::setJointMotorVel(rai::Joint* e, double v0, double maxF) {
  dJointSetAMotorParam(motors(e->index), dParamVel, -v0);
  dJointSetAMotorParam(motors(e->index), dParamFMax, maxF);
}

void OdeInterface::setJointMotorPos(doubleA& x, double maxF, double tau) {
  //CHECK_EQ(x.N,E, "given joint state has wrong dimension, " <<x.N <<"=" <<E);
  NIY;
  uint i=0;
  for_list(rai::Body,  n,  C.bodies) {
    for_list(rai::Joint,  e,  n->inLinks) { // if(e->motor)
      setJointMotorPos(e, x(i), maxF, tau);
      i++;
      break;
    }
  }
  CHECK_EQ(x.N, i, "joint motor array had wrong dimension " <<x.N <<"!=" <<i);
}

void OdeInterface::setJointMotorVel(doubleA& x, double maxF) {
  //CHECK_EQ(x.N,E, "given joint state has wrong dimension, " <<x.N <<"=" <<E);
  NIY;
  uint i=0;
  for_list(rai::Body,  n,  C.bodies) {
    for_list(rai::Joint,  e,  n->inLinks) { // if(e->motor){
      setJointMotorVel(e, x(i), maxF);
      i++;
      break;
    }
  }
  CHECK_EQ(x.N, i, "joint motor array had wrong dimension " <<x.N <<"!=" <<i);
}

void OdeInterface::unsetJointMotors() {
  NIY;
  for_list(rai::Body,  n,  C.bodies) for(rai::Joint* e: n->inLinks) { // if(e->motor){
    unsetJointMotor(e);
    break;
  }
}

void OdeInterface::unsetJointMotor(rai::Joint* e) {
  dJointSetAMotorParam(motors(e->index), dParamFMax, 0.);
}

void OdeInterface::getJointMotorForce(rai::Joint* e, double& f) {
  dJointFeedback* fb=dJointGetFeedback(motors(e->index));
  CHECK(fb, "no feedback buffer set for this joint");
  //std::cout <<OUTv(fb->f1) <<' ' <<OUTv(fb->t1) <<' ' <<OUTv(fb->f2) <<' ' <<OUTv(fb->t2) <<std::endl;
  rai::Vector t; t.x=fb->t1[0]; t.y=fb->t1[1]; t.z=fb->t1[2];
  f=t * (e->from->X.rot*(e->A.rot*rai::Vector(1, 0, 0)));
  f=-f;
}

void OdeInterface::getJointMotorForce(doubleA& f) {
  NIY;
  uint i=0;
  for_list(rai::Body,  n,  C.bodies) {
    for_list(rai::Joint,  e,  n->inLinks) { // if(!e->fixed && e->motor){
      getJointMotorForce(e, f(i));
      i++;
      break;
    }
  }
  CHECK_EQ(f.N, i, "joint motor array had wrong dimension " <<f.N <<"!=" <<i);
}

void OdeInterface::pidJointPos(rai::Joint* e, double x0, double v0, double xGain, double vGain, double iGain, double* eInt) {
  double x, v, f;
  //double a, b, c, dAcc;

  x=-dJointGetHingeAngle(joints(e->index));
  v=-dJointGetHingeAngleRate(joints(e->index));
  f = xGain*(x0-x) + vGain*(v0-v);
  if(eInt) {
    f+=iGain* (*eInt);
    (*eInt) = .99*(*eInt) + .01 *(x0-x);
  }
  std::cout <<"PID:" <<x0 <<' ' <<x <<' ' <<v <<" -> " <<f <<std::endl;
  dJointSetAMotorParam(motors(e->index), dParamFMax, 0);
  dJointAddHingeTorque(joints(e->index), -f);
}

void OdeInterface::pidJointVel(rai::Joint* e, double v0, double vGain) {
  double v, f;

  v=-dJointGetHingeAngleRate(joints(e->index));
  f = vGain*(v0-v);
  if(fabs(f)>vGain) f=rai::sign(f)*vGain;
  std::cout <<"PIDv:" <<v0 <<' ' <<v <<" -> " <<f <<std::endl;
  dJointSetAMotorParam(motors(e->index), dParamFMax, 0);
  dJointAddHingeTorque(joints(e->index), -f);
}

//===========================================================================
//
// higher level C
//

void OdeInterface::step(doubleA& force, uint steps, double tau) {
  addJointForce(force);
  for(; steps--;) step(tau);
  importStateFromOde();
}

void OdeInterface::step(uint steps, double tau) {
  for(; steps--;) step(tau);
  importStateFromOde();
}

void OdeInterface::getGroundContact(boolA& cts) {
  cts.resize(C.bodies.N);
  cts=false;
  //reportContacts();
  uint i;
  dContactGeom* c;
  for(i=0; i<conts.N; i++) {
    c = conts(i);
    dBodyID b1, b2;
    b1 = dGeomGetBody(c->g1);
    b2 = dGeomGetBody(c->g2);

    if(!b1 && b2) cts(((rai::Body*)b2->userdata)->index)=true;
    if(b1 && !b2) cts(((rai::Body*)b1->userdata)->index)=true;
  }
}

/*struct dContactGeom {
  dVector3 pos;       // contact position
  dVector3 normal;    // normal vector
  dReal depth;        // penetration depth
  dGeomID g1, g2;      // the colliding geoms
  };*/

void OdeInterface::importProxiesFromOde() {
  uint i;
  C.proxies.resizeCopy(conts.N);
  for(i=0; i<conts.N; i++) C.proxies(i) = new rai::Proxy;
  dContactGeom* c;
  int a, b;
  rai::Vector d, p;
  for(i=0; i<conts.N; i++) {
    c = conts(i);
    dBodyID b1, b2;
    b1 = dGeomGetBody(c->g1);
    b2 = dGeomGetBody(c->g2);

    a = b1 ? (((rai::Body*)b1->userdata)->index) : (uint)-1;
    b = b2 ? (((rai::Body*)b2->userdata)->index) : (uint)-1;

    d.set(c->normal);
    d *= -c->depth/2.;
    p.set(c->pos);

    C.proxies(i)->a = a;
    C.proxies(i)->b = b;
    C.proxies(i)->d = -c->depth;
    C.proxies(i)->normal.set(c->normal);
    C.proxies(i)->posA=p+d;
    C.proxies(i)->posB=p-d;
    //if(a!=-1) C.proxies(i)->velA=C.bodies(a)->X.vel + (C.bodies(a)->X.angvel^(p+d-(C.bodies(a)->X.pos))); else C.proxies(i)->velA.setZero();
    //if(b!=-1) C.proxies(i)->velB=C.bodies(b)->X.vel + (C.bodies(b)->X.angvel^(p-d-(C.bodies(b)->X.pos))); else C.proxies(i)->velB.setZero();
    C.proxies(i)->posB=p-d;
//    if(a!=-1 && b!=-1) C.proxies(i)->rel.setDifference(C.bodies(a)->X, C.bodies(b)->X);
//    else if(a!=-1) C.proxies(i)->rel.setInverse(C.bodies(a)->X);
//    else if(b!=-1) C.proxies(i)->rel = C.bodies(b)->X;
//    else           C.proxies(i)->rel.setZero();
//    C.proxies(i)->age=0;
  }
}

void OdeInterface::reportContacts2() {
  uint i;
  dBodyID b1, b2;
  rai::Body* b;
  dContactGeom* c;
  rai::Vector x;
  std::cout <<"contacts=" <<conts.N <<std::endl;
  for(i=0; i<conts.N; i++) {
    c = conts(i);
    std::cout <<i <<' ';
    b1=dGeomGetBody(c->g1);
    b2=dGeomGetBody(c->g2);
    if(b1) {
      b=(rai::Body*)b1->userdata;
      std::cout <<b->name <<' ';
    } else std::cout <<"NIL ";
    if(b2) {
      b=(rai::Body*)b2->userdata;
      std::cout <<b->name <<' ';
    } else std::cout <<"NIL ";
    x.set(c->pos);    std::cout <<"pos=" <<x <<' ';
    x.set(c->normal); std::cout <<"normal=" <<x <<' ';
    std::cout <<"depth=" <<c->depth <<std::endl;
  }
}

struct Bound { rai::Vector p, n; };
bool OdeInterface::inFloorContacts(rai::Vector& x) {
  uint i, j, k;
  dBodyID b1, b2;
  dContactGeom* c;
  rai::Vector y;

  x.z=0.;

  rai::Array<rai::Vector> v;
  //collect list of floor contacts
  for(i=0; i<conts.N; i++) {
    c = conts(i);
    b1=dGeomGetBody(c->g1);
    b2=dGeomGetBody(c->g2);
    if((!b1 && b2) || (b1 &&!b2)) {
      y.set(c->pos);  y.z=0.;
      v.append(y);
    }
  }

  std::cout <<"\nfloor points: ";
  for(i=0; i<v.N; i++) std::cout <<v(i) <<'\n';

  if(!v.N) return false;

  //construct boundaries
  Bound b;
  rai::Array<Bound> bounds;
  double s;
  bool f;
  for(i=0; i<v.N; i++) for(j=0; j<i; j++) {
      b.p=v(i); b.n=(v(j)-v(i)) ^ rai::Vector(0, 0, 1);
      f=false;
      for(k=0; k<v.N; k++) if(k!=j && k!=i) {
          s=(v(k)-b.p) * b.n ;
          if(s<0) {
            if(f) break;
            b.n=-b.n;
          }
          if(s!=0.) f=true;
        }
      if(k==v.N) bounds.append(b);
    }

  std::cout <<"\nbounds: ";
  for(i=0; i<bounds.N; i++) std::cout <<bounds(i).p <<' ' <<bounds(i).n <<'\n';

  std::cout <<"\nquery: " <<x <<std::endl;
  //check for internal
  for(i=0; i<bounds.N; i++) {
    if((x-bounds(i).p) * bounds(i).n < 0) return false;
  }
  return true;
}

void OdeInterface::slGetProxies() {
  //C.setJointState(x);
  //dJointGroupEmpty(contactgroup);
  //exportStateToOde(C, ode);
  conts.clear();
  noContactJoints=true;
  dSpaceCollide(space, this, OdeInterface::staticCallback);
  importProxiesFromOde();
}

/*void OdeInterface::slGetProxyGradient(arr &dx, const arr &x, rai::Configuration &C){
  if(C.proxies.N){
    arr dp, J;
    C.getContactGradient(dp);
    C.jacobianAll(J, x);
    dx = 2.* (~J) * dp;
  }else{
    dx.resize(x.N);
    dx.setZero();
  }
}

void OdeInterface::slGetProxyGradient(arr &dx, const arr &x, rai::Configuration &C, OdeInterface &ode){
  slGetProxies(x, C, ode);
  slGetProxyGradient(dx, x, C);
}*/

#undef OUTs
#undef OUTv
#undef OUTq
#undef CP4
#undef CP3

//===========================================================================
//
// communication with Ode
//

#if 0 //documentation...
/** @brief copy all frame variables (positions, velocities, etc)
into the ODE engine (createOde had to be called before) */
void OdeInterface::exportStateToOde();

/** @brief copy the current state of the ODE engine into
the frame variables (positions, velocities, etc) */
void OdeInterface::importStateFromOde();

/** @brief copy all frame variables (positions, velocities, etc)
into the ODE engine (createOde had to be called before) */
void OdeInterface::exportForcesToOde();

void OdeInterface::setJointForce(rai::Joint* e, double f1, double f2);

/** @brief applies forces on the configuration as given by the force vector x */
void OdeInterface::setJointForce(arr& f);

/** @brief returns the number of motors attached to joints */
uint getJointMotorDimension();

/** @brief set the desired positions of all motor joints */
void OdeInterface::setJointMotorPos(arr& x, double maxF, double tau);
/** @brief set the desired position of a specific motor joint */
void OdeInterface::setJointMotorPos(rai::Joint* e, double x0, double maxF, double tau);

/** @brief set the desired velocities of all motor joints */
void OdeInterface::setJointMotorVel(arr& v, double maxF=1.);
/** @brief set the desired velocity of a specific motor joint */
void OdeInterface::setJointMotorVel(rai::Joint* e, double v0, double maxF=1.);

/** @brief disable motors by setting maxForce parameter to zero */
void OdeInterface::unsetJointMotors(OdeInterface& ode);
/** @brief disable motor by setting maxForce parameter to zero */
void OdeInterface::unsetJointMotor(rai::Joint* e);

/** @brief get the forces induced by all motor motor joints */
void OdeInterface::getJointMotorForce(arr& f);
/** @brief get the force induced by a specific motor joint */
void OdeInterface::getJointMotorForce(rai::Joint* e, double& f);

void OdeInterface::pidJointPos(rai::Joint* e, double x0, double v0, double xGain, double vGain, double iGain=0, double* eInt=0);

void OdeInterface::pidJointVel(rai::Joint* e, double v0, double vGain);

/// [obsolete] \ingroup sl
void OdeInterface::getGroundContact(boolA& cts);

/// import the information from ODE's contact list into the proximity list \ingroup sl
void OdeInterface::importProxiesFromOde(OdeInterface& ode);

/** @brief simulates one time step with the ODE engine, starting from state
    vector in, returns state vector out \ingroup sl */
void OdeInterface::step(arr& in, arr& force, arr& out, uint steps=1);

/** @brief simulates one time step with the ODE engine, starting from state
    vector in, returns state vector out \ingroup sl */
void OdeInterface::step(arr& force, arr& out, uint steps=1);

/// \ingroup sl
void OdeInterface::step(uint steps=1, double tau=.01);

/** @brief instantiate the configuration in an ODE engine (first clears the
    ODE engine from all other objects) \ingroup sl */
void OdeInterface::createOde(OdeInterface& ode);

/// \ingroup sl
void OdeInterface::slGetProxies(OdeInterface& ode);

/// \ingroup sl
//void OdeInterface::slGetProxyGradient(arr &dx, const arr &x, rai::Configuration &C, OdeInterface &ode);

/// \ingroup sl
void OdeInterface::reportContacts(OdeInterface& ode);
/// \ingroup sl
bool inFloorContacts(rai::Vector& x);

#endif

#else
OdeInterface::OdeInterface(rai::Configuration& _C):C(_C) { RAI_MSG("WARNING - creating dummy OdeInterface"); }
OdeInterface::~OdeInterface() {}
void OdeInterface::step(double dtime) {}
void OdeInterface::clear() {}
void OdeInterface::slGetProxies() {}
void OdeInterface::unsetJointMotors() {}
void OdeInterface::exportStateToOde() {}
void OdeInterface::importStateFromOde() {}
void OdeInterface::importProxiesFromOde() {}
void OdeInterface::addJointForce(arr& f) {}
void OdeInterface::pushPoseForShape(rai::Shape* s) {};
#endif
