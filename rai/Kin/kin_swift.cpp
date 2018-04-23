/*  ------------------------------------------------------------------
    Copyright (c) 2017 Marc Toussaint
    email: marc.toussaint@informatik.uni-stuttgart.de
    
    This code is distributed under the MIT License.
    Please see <root-path>/LICENSE for details.
    --------------------------------------------------------------  */

/**
 * @file
 * @ingroup group_ors
 */
/**
 * @ingroup group_ors
 * @{
 */


#include "kin_swift.h"
#include "proxy.h"
#include "frame.h"
#include <Algo/ann.h>

#ifdef RAI_extern_SWIFT

#ifdef RAI_SINGLE
#  define SWIFT_USE_FLOAT
#endif

#include "SWIFT/SWIFT.h"
#undef min
#undef max

ANN *global_ANN=NULL;
rai::Shape *global_ANN_shape;

SwiftInterface::~SwiftInterface() {
  if(scene) delete scene;
  if(global_ANN) delete global_ANN;
  scene=NULL;
  //cout <<" -- SwiftInterface closed" <<endl;
}

SwiftInterface::SwiftInterface(const rai::KinematicWorld& world, double _cutoff)
  : scene(NULL), cutoff(_cutoff) {
  bool r, add;
  
  if(scene) delete scene;

  scene = new SWIFT_Scene(false, false);

  INDEXswift2frame.resize(world.frames.N);  INDEXswift2frame=-1;
  INDEXshape2swift.resize(world.frames.N);  INDEXshape2swift=-1;
  
  //cout <<" -- SwiftInterface init";
  rai::Shape *s;
  for(rai::Frame *f: world.frames) if((s=f->shape) && s->cont){
    //cout <<'.' <<flush;
    add=true;
    switch(s->type()) {
      case rai::ST_none: HALT("shapes should have a type - somehow wrong initialization..."); break;
      case rai::ST_mesh: {
        //check if there is a specific swiftfile!
        rai::FileToken *file = s->frame.ats.find<rai::FileToken>("swiftfile");
        if(false && file) {
          r=scene->Add_General_Object(file->name, INDEXshape2swift(f->ID), false);
          CHECK(INDEXshape2swift(f->ID)>=0, "no object generated from swiftfile");
          INDEXswift2frame(INDEXshape2swift(f->ID)) = f->ID;
          add=false;
          if(!r) HALT("--failed!");
        }
      } break;
      case rai::ST_pointCloud: {
        //for now, assume there is only ONE pointCloudObject!
        CHECK(s->mesh().V.N, "");
        global_ANN=new ANN;
        global_ANN_shape=s;
        global_ANN->setX(s->mesh().V);
        global_ANN->calculate();
        add=false;
      } break;
      case rai::ST_marker:
        add=false; // ignore (no collisions)
        break;
      default:
        break;
    }
    if(add) {
      if(!s->mesh().V.d0){
        s->getGeom().createMeshes();
        CHECK(s->mesh().V.d0, "the mesh must have been created earlier -- has size zero!");
      }
      rai::Mesh *mesh = &s->mesh();
//      if(s->sscCore().V.d0) mesh = &s->sscCore();

      CHECK(mesh->V.d0,"no mesh to add to SWIFT, something was wrongly initialized");
      r=scene->Add_Convex_Object(
          mesh->V.p, (int*)mesh->T.p,
          mesh->V.d0, mesh->T.d0, INDEXshape2swift(f->ID), false,
          DEFAULT_ORIENTATION, DEFAULT_TRANSLATION, DEFAULT_SCALE,
          DEFAULT_BOX_SETTING, DEFAULT_BOX_ENLARGE_REL, 2.);
      if(!r) HALT("--failed!");
      
      INDEXswift2frame(INDEXshape2swift(f->ID)) = f->ID;
    }
  }
  
  initActivations(world, 4);
  
  pushToSwift(world);
  //cout <<"...done" <<endl;
}

void SwiftInterface::reinitShape(const rai::Frame *f) {
  HALT("why?");
  int sw = INDEXshape2swift(f->ID);
  scene->Delete_Object(sw);
  INDEXswift2frame(sw) = -1;
  
  rai::Shape *s = f->shape;
  CHECK(s,"");
  bool r=scene->Add_Convex_Object(s->mesh().V.p, (int*)s->mesh().T.p,
                                  s->mesh().V.d0, s->mesh().T.d0, sw, false,
                                  DEFAULT_ORIENTATION, DEFAULT_TRANSLATION, DEFAULT_SCALE,
                                  DEFAULT_BOX_SETTING, DEFAULT_BOX_ENLARGE_REL, cutoff);
  if(!r) HALT("--failed!");
  
  INDEXshape2swift(f->ID) = sw;
  INDEXswift2frame(sw) = f->ID;
  
  if(s->cont) scene->Activate(sw);
}

void SwiftInterface::initActivations(const rai::KinematicWorld& world, uint parentLevelsToDeactivate) {
  /* deactivate some collision pairs:
    -- no `cont' -> no collisions with this object at all
    -- no collisions between shapes of same body
    -- no collisions between linked bodies
    -- no collisions between bodies liked via the tree via 3 links
  */
  
  //cout <<"collision active shapes: ";
  //for_list(Type,  s,  world.shapes) if(s->cont) cout <<s->name <<' ';
  
  for(rai::Frame *f: world.frames) if(f->shape){
    if(!f->shape->cont) {
      if(INDEXshape2swift(f->ID)!=-1) scene->Deactivate(INDEXshape2swift(f->ID));
    } else {
      if(INDEXshape2swift(f->ID)!=-1) scene->Activate(INDEXshape2swift(f->ID));
//      cout <<"activating " <<f->name <<endl;
    }
  }
  //shapes within a body
//  for(rai::Frame *b: world.frames) deactivate(b->shapes);
  //deactivate along edges...
  for(rai::Frame *f: world.frames) if(f->parent){
//    cout <<"deactivating edge pair " <<f->parent->name <<"--" <<f->name <<endl;
    deactivate({ f->parent, f });
  }
  //deactivate along trees...
  for(rai::Frame *b: world.frames) {
    FrameL group, children;
    group.append(b);
    //all rigid links as well
    for(uint i=0;i<group.N;i++){
      for(rai::Frame *b2: group(i)->outLinks) if(!b2->joint) group.setAppend(b2);
    }
    for(uint l=0; l<parentLevelsToDeactivate; l++){
      children.clear();
      for(rai::Frame *b2: group) for(rai::Frame *b2to: b2->outLinks) children.setAppend(b2to);
      group.setAppend(children);
      //all rigid links as well
      for(uint i=0;i<group.N;i++){
        for(rai::Frame *b2: group(i)->outLinks) if(!b2->joint) group.setAppend(b2);
      }
    }
    deactivate(group);
  }
}

void SwiftInterface::deactivate(const FrameL& shapes) {
  //cout <<"deactivating shape group "; listWriteNames(shapes, cout); cout <<endl;
  for(rai::Frame *s1: shapes){
    for(rai::Frame *s2: shapes) {
      if(s1->ID > s2->ID) deactivate(s1, s2);
    }
  }
}

void SwiftInterface::deactivate(rai::Frame *s1, rai::Frame *s2) {
  if(INDEXshape2swift(s1->ID)==-1 || INDEXshape2swift(s2->ID)==-1) return;
  //cout <<"deactivating shape pair " <<s1->name <<'-' <<s2->name <<endl;
  scene->Deactivate(INDEXshape2swift(s1->ID), INDEXshape2swift(s2->ID));
}

void SwiftInterface::activate(rai::Frame *s1, rai::Frame *s2) {
  if(INDEXshape2swift(s1->ID)==-1 || INDEXshape2swift(s2->ID)==-1) return;
  //cout <<"deactivating shape pair " <<s1->name <<'-' <<s2->name <<endl;
  scene->Activate(INDEXshape2swift(s1->ID), INDEXshape2swift(s2->ID));
}

void SwiftInterface::activate(rai::Frame *s) {
  if(INDEXshape2swift(s->ID)==-1) return;
  scene->Activate(INDEXshape2swift(s->ID));
}

void SwiftInterface::deactivate(rai::Frame *s) {
  if(INDEXshape2swift(s->ID)==-1) return;
  scene->Deactivate(INDEXshape2swift(s->ID));
}

void SwiftInterface::pushToSwift(const rai::KinematicWorld& world) {
  //CHECK_EQ(INDEXshape2swift.N,world.shapes.N,"the number of shapes has changed");
  CHECK(INDEXshape2swift.N <= world.frames.N, "the number of shapes has changed");
  rai::Matrix rot;
  for(rai::Frame *f: world.frames) {
    if(f->shape){
      if(f->ID<INDEXshape2swift.N && INDEXshape2swift(f->ID)!=-1) {
        rot = f->X.rot.getMatrix();
        scene->Set_Object_Transformation(INDEXshape2swift(f->ID), rot.p(), f->X.pos.p());
        if(!f->shape->cont) scene->Deactivate(INDEXshape2swift(f->ID));
        //else         scene->Activate( INDEXshape2swift(f->ID) );
      }
    }
  }
}

void SwiftInterface::pullFromSwift(rai::KinematicWorld& world, bool dumpReport) {
  int i, j, k, np;
  int *oids, *num_contacts;
  SWIFT_Real *dists, *nearest_pts, *normals;
  
  try {
    scene->Query_Contact_Determination(
      false, cutoff, np,
      &oids, &num_contacts,
      &dists,
      &nearest_pts,
      &normals);
  } catch(const char *msg) {
    world.proxies.clear();
    cout <<"... catching error '" <<msg <<"' -- SWIFT failed! .. no proxies for this posture!!..." <<endl;
    return;
  }
  
  if(dumpReport) {
    cout <<"contacts: np=" <<np <<endl;
    for(k=0, i=0; i<np; i++) {
      cout <<"* Shape '" <<world.frames(INDEXswift2frame(oids[i <<1]))->name <<"' vs. Shape '" <<world.frames(INDEXswift2frame(oids[(i <<1)+1]))->name <<"'" <<endl;
      cout <<"    distance= " <<dists[i] <<endl;
      cout <<"  #contacts = " <<num_contacts[i] <<endl;
      for(j=0; j<num_contacts[i]; j++, k++) {
        cout <<"  - contact " <<j <<endl;
        cout <<"    points  = " <<nearest_pts[6*k+0] <<' ' <<nearest_pts[6*k+1] <<' ' <<nearest_pts[6*k+2] <<' ' <<nearest_pts[6*k+3] <<' ' <<nearest_pts[6*k+4] <<' ' <<nearest_pts[6*k+5] <<endl;
        cout <<"    normals = " <<normals[3*k+0] <<' ' <<normals[3*k+1] <<' ' <<normals[3*k+2] <<endl;
      }
      if(num_contacts[i]==-1) k++;
    }
  }
  
  
  for(rai::Proxy& p:world.proxies) p.del_coll();
  world.proxies.resize(np);
  
  //add contacts to list

  for(k=0, i=0; i<np; i++) {
    rai::Proxy &proxy = world.proxies.elem(i);
    //CHECK(ids(a)==a && ids(b)==b, "shape index does not coincide with swift index");
    proxy.a = world.frames(INDEXswift2frame(oids[i <<1]));
    proxy.b = world.frames(INDEXswift2frame(oids[(i <<1)+1]));
    proxy.d = dists[i];

    //non-penetrating pair of objects
    if(num_contacts[i]>0) { //only add one proxy!for(j=0; j<num_contacts[i]; j++, k++) {
      CHECK(num_contacts[i]==1,"");
      if(proxy.d < 1e-10){
        proxy.posA = proxy.a->X.pos;
        proxy.posB = proxy.b->X.pos;
        proxy.normal = proxy.posA - proxy.posB; //normal always points from b to a
        if(!proxy.normal.isZero) proxy.normal.normalize();
      }else{
        proxy.normal.set(&normals[3*k+0]);
        proxy.normal.normalize();
        //swift returns nearest points in the local frame -> transform them
        proxy.posA.set(&nearest_pts[6*k+0]);  proxy.posA = proxy.a->X * proxy.posA;
        proxy.posB.set(&nearest_pts[6*k+3]);  proxy.posB = proxy.b->X * proxy.posB;
      }
      k += num_contacts[i];
    }else if(num_contacts[i]==-1) { //penetrating pair of objects
      proxy.d = -.0;
      proxy.posA = proxy.a->X.pos;
      proxy.posB = proxy.b->X.pos;
      proxy.normal = proxy.posA - proxy.posB; //normal always points from b to a
      if(!proxy.normal.isZero) proxy.normal.normalize();
      k++;
    }else if(num_contacts[i]==0){
      RAI_MSG("what is this?");
    }

  }

  //add pointClound stuff to list
  if(global_ANN) {
    HALT("deprecated");
    uint i, _i=0;
    arr R(3, 3), t(3);
    arr v, dists, _dists;
    intA idx, _idx;
    rai::Shape *s;
    for(rai::Frame *f: world.frames) if((s=f->shape)){
      if(!s->cont || s==global_ANN_shape) continue;
      
      //relative rotation and translation of shapes
      rai::Transformation rel;
      rel.setDifference(global_ANN_shape->frame.X, s->frame.X);
      rel.rot.getMatrix(R.p);
      t = conv_vec2arr(rel.pos);
      
      //check for each vertex
      for(i=0; i<s->mesh().V.d0; i++) {
        v = s->mesh().V[i];
        v = R*v + t;
        global_ANN->getkNN(dists, idx, v, 1);
        if(!i || dists(0)<_dists(0)) {
          _i=i;  _dists=dists;  _idx=idx;
        }
      }
      if(_dists(0)>cutoff) continue;
      
      rai::Proxy *proxy = new rai::Proxy();
      proxy->a = &global_ANN_shape->frame;
      proxy->b = &s->frame;
      proxy->d = _dists(0);
      proxy->posA.set(&global_ANN_shape->mesh().V(_idx(0), 0));  proxy->posA = global_ANN_shape->frame.X * proxy->posA;
      proxy->posB.set(&s->mesh().V(_i, 0));                      proxy->posB = s->frame.X * proxy->posB;
      proxy->normal = proxy->posA - proxy->posB;
      proxy->normal.normalize();
    }
  }
}

void SwiftInterface::step(rai::KinematicWorld& world, bool dumpReport) {
  pushToSwift(world);
  pullFromSwift(world, dumpReport);
}

void SwiftInterface::swiftQueryExactDistance() {
  int i, np;
  int *oids;
  SWIFT_Real *dists;
  
  scene->Query_Exact_Distance(false, SWIFT_INFINITY, np, &oids, &dists);
  
  cout <<"exact distances: np=" <<np <<endl;
  for(i=0; i<np; i++) {
    cout <<"    Object " <<oids[i <<1] <<" vs. Object "
         <<oids[(i <<1)+1] <<" = " <<dists[i] <<endl;
  }
}

uint SwiftInterface::countObjects(){
  uint n=0;
  for(int& i : INDEXshape2swift) if(i>=0) n++;
  return n;
}

#else
#include <Core/util.h>
void SwiftInterface::step(rai::KinematicWorld &world, bool dumpReport=false){}
void SwiftInterface::pushToSwift() {}
  void SwiftInterface::pullFromSwift(const KinematicWorld &world, bool dumpReport) {}

  void SwiftInterface::reinitShape(const rai::Shape *s) {}
//  void close();
  void SwiftInterface::deactivate(rai::Shape *s1, rai::Shape *s2) {}
  void SwiftInterface::deactivate(const rai::Array<rai::Shape*>& shapes) {}
  void SwiftInterface::deactivate(const rai::Array<rai::Frame*>& frames) {}
  void SwiftInterface::initActivations(const KinematicWorld &world, uint parentLevelsToDeactivate=3) {}
  void SwiftInterface::swiftQueryExactDistance() {}
#endif
/** @} */
