/*  ------------------------------------------------------------------
    Copyright (c) 2011-2020 Marc Toussaint
    email: toussaint@tu-berlin.de

    This code is distributed under the MIT License.
    Please see <root-path>/LICENSE for details.
    --------------------------------------------------------------  */

#include "kin_swift.h"
#include "proxy.h"
#include "frame.h"
#include "../Algo/ann.h"

#ifdef RAI_SWIFT

#ifdef RAI_SINGLE
#  define SWIFT_USE_FLOAT
#endif

#include "SWIFT/SWIFT.h"
#undef min
#undef max

ANN* global_ANN=nullptr;
rai::Shape* global_ANN_shape;

SwiftInterface::~SwiftInterface() {
  if(scene) delete scene;
  if(global_ANN) delete global_ANN;
  scene=nullptr;
  //cout <<" -- SwiftInterface closed" <<endl;
}

SwiftInterface::SwiftInterface(const FrameL& frames, double _cutoff, int verbose)
  : scene(nullptr), cutoff(_cutoff) {
  bool r, add;

  if(scene) delete scene;

  scene = new SWIFT_Scene(false, true); //false, false);

  INDEXswift2frame.resize(frames.N);  INDEXswift2frame=-1;
  swiftID.resize(frames.N);  swiftID=-1;

  if(verbose>0) cout <<" -- SwiftInterface init";
  rai::Shape* s;
  for(rai::Frame* f: frames) if((s=f->shape) && s->cont) {
      if(verbose>0) cout <<'.' <<flush;
      if(verbose>1) cout <<f->name <<flush;
      add=true;
      switch(s->type()) {
        case rai::ST_none: HALT("shapes should have a type - somehow wrong initialization..."); break;
        case rai::ST_mesh: {
          //check if there is a specific swiftfile!
          rai::FileToken* file = s->frame.ats.find<rai::FileToken>("swiftfile");
          if(false && file) {
            r=scene->Add_General_Object(file->name, swiftID(f->ID), false);
            CHECK_GE(swiftID(f->ID), 0, "no object generated from swiftfile");
            INDEXswift2frame(swiftID(f->ID)) = f->ID;
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
        if(!s->mesh().V.d0) {
          s->createMeshes();
          CHECK(s->mesh().V.d0, "the mesh must have been created earlier -- has size zero!");
        }
        rai::Mesh* mesh = &s->mesh();
//      if(s->sscCore().V.d0) mesh = &s->sscCore();

        CHECK(mesh->V.d0, "no mesh to add to SWIFT, something was wrongly initialized");
        r=scene->Add_Convex_Object(
            mesh->V.p, (int*)mesh->T.p,
            mesh->V.d0, mesh->T.d0, swiftID(f->ID), false,
            DEFAULT_ORIENTATION, DEFAULT_TRANSLATION, DEFAULT_SCALE,
            DEFAULT_BOX_SETTING, DEFAULT_BOX_ENLARGE_REL, 2.);
        if(!r) HALT("--failed!");

        INDEXswift2frame(swiftID(f->ID)) = f->ID;
      }
    }

//  initActivations(frames);

//  pushToSwift(frames);
  if(verbose>0) cout <<"...done" <<endl;
}

void SwiftInterface::reinitShape(const rai::Frame* f) {
  HALT("why?");
  int sw = swiftID(f->ID);
  scene->Delete_Object(sw);
  INDEXswift2frame(sw) = -1;

  rai::Shape* s = f->shape;
  CHECK(s, "");
  bool r=scene->Add_Convex_Object(s->mesh().V.p, (int*)s->mesh().T.p,
                                  s->mesh().V.d0, s->mesh().T.d0, sw, false,
                                  DEFAULT_ORIENTATION, DEFAULT_TRANSLATION, DEFAULT_SCALE,
                                  DEFAULT_BOX_SETTING, DEFAULT_BOX_ENLARGE_REL, cutoff);
  if(!r) HALT("--failed!");

  swiftID(f->ID) = sw;
  INDEXswift2frame(sw) = f->ID;

  if(s->cont) scene->Activate(sw);
}

void SwiftInterface::initActivations(const FrameL& frames) {
  HALT("obsolete");
  /* deactivate some collision pairs:
    -- no `cont' -> no collisions with this object at all
    -- no collisions between shapes of same body
    -- no collisions between linked bodies
    -- no collisions between bodies liked via the tree via 3 links
  */

//  cout <<"collision active shapes: ";
//  for(auto* f:frames) if(f->shape && f->shape->cont) cout <<f->name <<' ';

  for(rai::Frame* f: frames) if(f->shape) {
      if(!f->shape->cont) {
        if(swiftID(f->ID)!=-1) scene->Deactivate(swiftID(f->ID));
      } else {
        if(swiftID(f->ID)!=-1) scene->Activate(swiftID(f->ID));
//        cout <<"activating " <<f->name <<endl;
      }
    }
  //shapes within a link
  FrameL links; NIY// = world.getLinks();
  for(rai::Frame* f: links) {
    FrameL F = {f};
    f->getRigidSubFrames(F);
    for(uint i=F.N; i--;) if(!F(i)->shape || !F(i)->shape->cont) F.remove(i);
    deactivate(F);
  }
  //deactivate upward, depending on cont parameter (-1 indicates deactivate with parent)
  for(rai::Frame* f: frames) if(f->shape && f->shape->cont<0) {
      FrameL F, P;
      rai::Frame* p = f->getUpwardLink();
      F = {p};
      p->getRigidSubFrames(F);
      for(uint i=F.N; i--;) if(!F(i)->shape || !F(i)->shape->cont) F.remove(i);

      for(char i=0; i<-f->shape->cont; i++) {
        p = p->parent;
        if(!p) break;
        p = p->getUpwardLink();
        P = {p};
        p->getRigidSubFrames(P);
        for(uint i=P.N; i--;) if(!P(i)->shape || !P(i)->shape->cont) P.remove(i);

        if(F.N && P.N) deactivate(F, P);
      }
    }
}

void SwiftInterface::deactivate(rai::Frame* s1, rai::Frame* s2) {
  if(swiftID(s1->ID)==-1 || swiftID(s2->ID)==-1) return;
  //cout <<"deactivating shape pair " <<s1->name <<'-' <<s2->name <<endl;
  scene->Deactivate(swiftID(s1->ID), swiftID(s2->ID));
}

void SwiftInterface::deactivate(const FrameL& shapes1, const FrameL& shapes2) {
//  cout <<"deactivating shapes {"; listWriteNames(shapes1, cout);
//  cout <<"} versus {"; listWriteNames(shapes2, cout); cout <<"}" <<endl;
  for(rai::Frame* s1: shapes1) {
    for(rai::Frame* s2: shapes2) {
      deactivate(s1, s2);
    }
  }
}

void SwiftInterface::deactivate(const FrameL& shapes) {
  deactivate(shapes, shapes);
}

void SwiftInterface::activate(rai::Frame* s1, rai::Frame* s2) {
  if(swiftID(s1->ID)==-1 || swiftID(s2->ID)==-1) return;
  //cout <<"deactivating shape pair " <<s1->name <<'-' <<s2->name <<endl;
  scene->Activate(swiftID(s1->ID), swiftID(s2->ID));
}

void SwiftInterface::activate(rai::Frame* s) {
  if(swiftID(s->ID)==-1) return;
  scene->Activate(swiftID(s->ID));
}

void SwiftInterface::deactivate(rai::Frame* s) {
  if(swiftID(s->ID)==-1) return;
  scene->Deactivate(swiftID(s->ID));
}

void SwiftInterface::pushToSwift(const arr& X) {
  CHECK_EQ(X.nd, 2, "");
  CHECK_EQ(X.d0, swiftID.N, "");
  CHECK_EQ(X.d1, 7, "");

  //CHECK_EQ(INDEXshape2swift.N,world.shapes.N,"the number of shapes has changed");
//  CHECK_LE(swiftID.N,  frames.N, "the number of shapes has changed");
  rai::Transformation t;
  rai::Matrix rot;
  for(uint i=0;i<X.d0;i++){
    int a = swiftID.elem(i);
    if(a==-1) continue;
    if(i<X_lastQuery.d0 && maxDiff(X_lastQuery[i], X[i])<1e-8) continue;
    t.set(X[i]);

    rot = t.rot.getMatrix();
    scene->Set_Object_Transformation(a, rot.p(), &t.pos.x);
  }
}

uintA SwiftInterface::pullFromSwift(bool dumpReport) {
  int i, j, k, np;
  int* oids=0, *num_contacts=0;
  SWIFT_Real* dists=0, *nearest_pts=0, *normals=0;

  try {
//    scene->Query_Contact_Determination(
//      false, cutoff, np,
//      &oids, &num_contacts,
//      &dists,
//      &nearest_pts,
//      &normals);
    scene->Query_Tolerance_Verification(false, cutoff, np, &oids);
//    scene->Query_Intersection(false, np, &oids);
  } catch(const char* msg) {
    std::cerr <<"... catching error '" <<msg <<"' -- SWIFT failed! .. no proxies for this posture!!..." <<endl;
    return uintA();
  } catch(std::exception& e) {
    cout <<"... catching error '" <<e.what() <<"' -- SWIFT failed! .. no proxies for this posture!!..." <<endl;
    return uintA();
  } catch(std::pair<int, int>& e) {
    cout <<"... catching error at pair ("
         <<e.first <<'(' <<INDEXswift2frame(e.first) <<") "
         <<e.second <<'(' <<INDEXswift2frame(e.second) <<") -- SWIFT failed! .. no proxies for this posture!!..." <<endl;
    return uintA();
  }

  if(dumpReport) {
    cout <<"contacts: np=" <<np <<endl;
    for(k=0, i=0; i<np; i++) {
      cout <<"* Shape '" <<INDEXswift2frame(oids[i <<1]) <<"' vs. Shape '" <<INDEXswift2frame(oids[(i <<1)+1]) <<"'" <<endl;
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

  uintA collisionPairs(np,2);
  for(k=0, i=0; i<np; i++) {
    collisionPairs(i,0) = INDEXswift2frame(oids[i <<1]);
    collisionPairs(i,1) = INDEXswift2frame(oids[(i <<1)+1]);
  }
  return collisionPairs;

#if 0
  proxies.clear();
  proxies.resize(np);

  //add contacts to list

  for(k=0, i=0; i<np; i++) {
    rai::Proxy& proxy = proxies.elem(i);
    //CHECK(ids(a)==a && ids(b)==b, "shape index does not coincide with swift index");
    proxy.a = frames(INDEXswift2frame(oids[i <<1]));
    proxy.b = frames(INDEXswift2frame(oids[(i <<1)+1]));
    proxy.d = -.0; //dists[i];
    proxy.posA = proxy.a->ensure_X().pos;
    proxy.posB = proxy.b->ensure_X().pos;

#if 0
    //non-penetrating pair of objects
    if(num_contacts[i]>0) { //only add one proxy!for(j=0; j<num_contacts[i]; j++, k++) {
      CHECK_EQ(num_contacts[i], 1, "");
      if(proxy.d < 1e-10) {
        proxy.posA = proxy.a->X.pos;
        proxy.posB = proxy.b->X.pos;
        proxy.normal = proxy.posA - proxy.posB; //normal always points from b to a
        if(!proxy.normal.isZero) proxy.normal.normalize();
      } else {
        proxy.normal.set(&normals[3*k+0]);
        proxy.normal.normalize();
        //swift returns nearest points in the local frame -> transform them
        proxy.posA.set(&nearest_pts[6*k+0]);  proxy.posA = proxy.a->X * proxy.posA;
        proxy.posB.set(&nearest_pts[6*k+3]);  proxy.posB = proxy.b->X * proxy.posB;
      }
      k += num_contacts[i];
    } else if(num_contacts[i]==-1) { //penetrating pair of objects
      proxy.d = -.0;
      proxy.posA = proxy.a->X.pos;
      proxy.posB = proxy.b->X.pos;
      proxy.normal = proxy.posA - proxy.posB; //normal always points from b to a
      if(!proxy.normal.isZero) proxy.normal.normalize();
      k++;
    } else if(num_contacts[i]==0) {
      RAI_MSG("what is this?");
    }
#endif

  }

  //add pointClound stuff to list
  if(global_ANN) {
    HALT("deprecated");
    uint i, _i=0;
    arr R(3, 3), t(3);
    arr v, dists, _dists;
    intA idx, _idx;
    rai::Shape* s;
    for(rai::Frame* f: frames) if((s=f->shape)) {
        if(!s->cont || s==global_ANN_shape) continue;

        //relative rotation and translation of shapes
        rai::Transformation rel;
        rel.setDifference(global_ANN_shape->frame.ensure_X(), s->frame.ensure_X());
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

        rai::Proxy* proxy = new rai::Proxy();
        proxy->a = &global_ANN_shape->frame;
        proxy->b = &s->frame;
        proxy->d = _dists(0);
        proxy->posA.set(&global_ANN_shape->mesh().V(_idx(0), 0));  proxy->posA = global_ANN_shape->frame.ensure_X() * proxy->posA;
        proxy->posB.set(&s->mesh().V(_i, 0));                      proxy->posB = s->frame.ensure_X() * proxy->posB;
        proxy->normal = proxy->posA - proxy->posB;
        proxy->normal.normalize();
      }
  }
#endif
}

void SwiftInterface::deactivate(const uintA& collisionExcludeIDs){
  if(!collisionExcludeIDs.N) return;
  CHECK_EQ(collisionExcludeIDs.nd, 1, "");
  for(uint i=0;i<collisionExcludeIDs.d0;i++){
    int a=swiftID(collisionExcludeIDs(i));
    if(a==-1) continue;
    //cout <<"deactivating shape pair " <<s1->name <<'-' <<s2->name <<endl;
    scene->Deactivate(a);
  }
}

void SwiftInterface::deactivatePairs(const uintA& collisionExcludePairIDs){
  if(!collisionExcludePairIDs.N) return;
  CHECK_EQ(collisionExcludePairIDs.nd, 2, "");
  for(uint i=0;i<collisionExcludePairIDs.d0;i++){
    int a=swiftID(collisionExcludePairIDs(i,0));
    int b=swiftID(collisionExcludePairIDs(i,1));
    if(a==-1 || b==-1) continue;
    //cout <<"deactivating shape pair " <<s1->name <<'-' <<s2->name <<endl;
    scene->Deactivate(a, b);
  }
}

uintA SwiftInterface::step(const arr& X, bool dumpReport) {
  pushToSwift(X);
  return pullFromSwift(dumpReport);
}

void SwiftInterface::swiftQueryExactDistance() {
  int i, np;
  int* oids;
  SWIFT_Real* dists;

  scene->Query_Exact_Distance(false, SWIFT_INFINITY, np, &oids, &dists);

  cout <<"exact distances: np=" <<np <<endl;
  for(i=0; i<np; i++) {
    cout <<"    Object " <<oids[i <<1] <<" vs. Object "
         <<oids[(i <<1)+1] <<" = " <<dists[i] <<endl;
  }
}

uint SwiftInterface::countObjects() {
  uint n=0;
  for(int& i : swiftID) if(i>=0) n++;
  return n;
}

#else //RAI_SWIFT

#include "../Core/util.h"

SwiftInterface(const FrameL& frames, double _cutoff=.2, int verbose=0){ NICO }
~SwiftInterface(){ NICO }

uintA step(const arr& X, bool dumpReport=false){ NICO }

void pushToSwift(const arr& X){ NICO }
uintA pullFromSwift(bool dumpReport){ NICO }

void deactivate(const uintA& collisionExcludePairIDs){ NICO }



void SwiftInterface::reinitShape(const rai::Frame* s) { NICO }
//  void close(){ NICO }
void SwiftInterface::activate(rai::Frame* s) { NICO }
void SwiftInterface::deactivate(rai::Frame* s) { NICO }
void SwiftInterface::activate(rai::Frame* s1, rai::Frame* s2) { NICO }
void SwiftInterface::deactivate(rai::Frame* s1, rai::Frame* s2) { NICO }
void SwiftInterface::deactivate(const FrameL& shapes1, const FrameL& shapes2) { NICO }
void SwiftInterface::deactivate(const FrameL& shapes) { NICO }

void SwiftInterface::initActivations(const FrameL& frames) { NICO }
void SwiftInterface::swiftQueryExactDistance() { NICO }
uint SwiftInterface::countObjects() { NICO }

#endif
