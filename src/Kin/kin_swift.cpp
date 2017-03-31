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

/**
 * @file
 * @ingroup group_ors
 */
/**
 * @ingroup group_ors
 * @{
 */


#include "kin_swift.h"
#include <Algo/ann.h>

#ifdef MLR_extern_SWIFT

#ifdef MLR_SINGLE
#  define SWIFT_USE_FLOAT
#endif

#include "SWIFT/SWIFT.h"
#undef min
#undef max

ANN *global_ANN=NULL;
mlr::Shape *global_ANN_shape;

SwiftInterface::~SwiftInterface() {
  if(scene) delete scene;
  if(global_ANN) delete global_ANN;
  scene=NULL;
  cout <<" -- SwiftInterface closed" <<endl;
}

SwiftInterface::SwiftInterface(const mlr::KinematicWorld& world, double _cutoff)
  : scene(NULL), cutoff(_cutoff) {
  bool r, add;
  
  if(scene) delete scene;

  scene=new SWIFT_Scene(false, false);

  INDEXswift2shape.resize(world.shapes.N);  INDEXswift2shape=-1;
  INDEXshape2swift.resize(world.shapes.N);  INDEXshape2swift=-1;
  
  //cout <<" -- SwiftInterface init";
  for_list(mlr::Shape, s,  world.shapes) if(s->cont){
    //cout <<'.' <<flush;
    add=true;
    switch(s->type) {
      case mlr::ST_none: HALT("shapes should have a type - somehow wrong initialization..."); break;
      case mlr::ST_mesh: {
        //check if there is a specific swiftfile!
        mlr::String *filename;
        filename=s->ats.find<mlr::String>("swiftfile");
        if(!filename)
          filename=s->body->ats.find<mlr::String>("swiftfile");
        if(filename) {
          r=scene->Add_General_Object(*filename, INDEXshape2swift(s->index), false);
          if(!r) HALT("--failed!");
        }
      } break;
      case mlr::ST_pointCloud: {
        //for now, assume there is only ONE pointCloudObject!
        CHECK(s->mesh.V.N, "");
        global_ANN=new ANN;
        global_ANN_shape=s;
        global_ANN->setX(s->mesh.V);
        global_ANN->calculate();
        add=false;
      } break;
      case mlr::ST_marker:
        add=false; // ignore (no collisions)
        break;
      default:
        break;
    }
    if(add) {
      if(!s->mesh.V.d0){
        switch(s->type) {
          case mlr::ST_box:
            s->mesh.setBox();
            s->mesh.scale(s->size(0), s->size(1), s->size(2));
            break;
          case mlr::ST_sphere:
            s->mesh.setSphere();
            s->mesh.scale(s->size(3), s->size(3), s->size(3));
            break;
          case mlr::ST_cylinder:
            CHECK(s->size(3)>1e-10,"");
            s->mesh.setCylinder(s->size(3), s->size(2));
            break;
          case mlr::ST_capsule:
            CHECK(s->size(3)>1e-10,"");
            s->mesh.setCappedCylinder(s->size(3), s->size(2));
            break;
          case mlr::ST_retired_SSBox:
            s->mesh.setSSBox(s->size(0), s->size(1), s->size(2), s->size(3));
            break;
          default:
            break;
        }
        s->mesh_radius = s->mesh.getRadius();
      }
      CHECK(s->mesh.V.d0,"no mesh to add to SWIFT, something was wrongly initialized");
      r=scene->Add_Convex_Object(
          s->mesh.V.p, (int*)s->mesh.T.p,
          s->mesh.V.d0, s->mesh.T.d0, INDEXshape2swift(s->index), false,
          DEFAULT_ORIENTATION, DEFAULT_TRANSLATION, DEFAULT_SCALE,
          DEFAULT_BOX_SETTING, DEFAULT_BOX_ENLARGE_REL, 2.);
      if(!r) HALT("--failed!");
      
      INDEXswift2shape(INDEXshape2swift(s->index)) = s->index;
    }
  }
  
  initActivations(world, 4);
  
  pushToSwift(world);
  //cout <<"...done" <<endl;
}

void SwiftInterface::reinitShape(const mlr::Shape *s) {
  int sw = INDEXshape2swift(s->index);
  scene->Delete_Object(sw);
  INDEXswift2shape(sw) = -1;
  
  bool r=scene->Add_Convex_Object(s->mesh.V.p, (int*)s->mesh.T.p,
                                  s->mesh.V.d0, s->mesh.T.d0, sw, false,
                                  DEFAULT_ORIENTATION, DEFAULT_TRANSLATION, DEFAULT_SCALE,
                                  DEFAULT_BOX_SETTING, DEFAULT_BOX_ENLARGE_REL, cutoff);
  if(!r) HALT("--failed!");
  
  INDEXshape2swift(s->index) = sw;
  INDEXswift2shape(sw) = s->index;
  
  if(s->cont) scene->Activate(sw);
}

void SwiftInterface::initActivations(const mlr::KinematicWorld& world, uint parentLevelsToDeactivate) {
  /* deactivate some collision pairs:
    -- no `cont' -> no collisions with this object at all
    -- no collisions between shapes of same body
    -- no collisions between linked bodies
    -- no collisions between bodies liked via the tree via 3 links
  */
  
  //cout <<"collision active shapes: ";
  //for_list(Type,  s,  world.shapes) if(s->cont) cout <<s->name <<' ';
  
  for_list(mlr::Shape, s, world.shapes) {
    if(!s->cont) {
      if(INDEXshape2swift(s->index)!=-1) scene->Deactivate(INDEXshape2swift(s->index));
    } else {
      if(INDEXshape2swift(s->index)!=-1) scene->Activate(INDEXshape2swift(s->index));
    }
  }
  //shapes within a body
  for(mlr::Body *b: world.bodies) deactivate(b->shapes);
  //deactivate along edges...
  for_list(mlr::Joint, e, world.joints) {
    //cout <<"deactivating edge pair"; listWriteNames({e->from, e->to}, cout); cout <<endl;
    deactivate(mlr::Array<mlr::Body*>({ e->from, e->to }));
  }
  //deactivate along trees...
  for_list(mlr::Body,  b,  world.bodies) {
    mlr::Array<mlr::Body*> group, children;
    group.append(b);
    for(uint l=0; l<parentLevelsToDeactivate; l++) {
      //listWriteNames(group, cout);
      children.clear();
      for_list(mlr::Body,  b2,  group) {
        for_list(mlr::Joint,  e,  b2->outLinks) {
          children.setAppend(e->to);
          //listWriteNames(children, cout);
        }
      }
      group.setAppend(children);
    }
    deactivate(group);
  }
}

void SwiftInterface::deactivate(const mlr::Array<mlr::Body*>& bodies) {
  //cout <<"deactivating body group "; listWriteNames(bodies, cout); cout <<endl;
  mlr::Array<mlr::Shape*> shapes;
  for_list(mlr::Body, b, bodies) shapes.setAppend(b->shapes);
  deactivate(shapes);
}

void SwiftInterface::deactivate(const mlr::Array<mlr::Shape*>& shapes) {
  //cout <<"deactivating shape group "; listWriteNames(shapes, cout); cout <<endl;
  for_list(mlr::Shape, s1, shapes){
    for_list(mlr::Shape, s2, shapes) {
      if(s1_COUNT>s2_COUNT) deactivate(s1, s2);
    }
  }
}

void SwiftInterface::deactivate(mlr::Shape *s1, mlr::Shape *s2) {
  if(INDEXshape2swift(s1->index)==-1 || INDEXshape2swift(s2->index)==-1) return;
  //cout <<"deactivating shape pair " <<s1->name <<'-' <<s2->name <<endl;
  scene->Deactivate(INDEXshape2swift(s1->index), INDEXshape2swift(s2->index));
}

void SwiftInterface::activate(mlr::Shape *s1, mlr::Shape *s2) {
  if(INDEXshape2swift(s1->index)==-1 || INDEXshape2swift(s2->index)==-1) return;
  //cout <<"deactivating shape pair " <<s1->name <<'-' <<s2->name <<endl;
  scene->Activate(INDEXshape2swift(s1->index), INDEXshape2swift(s2->index));
}

void SwiftInterface::activate(mlr::Shape *s) {
  if(INDEXshape2swift(s->index)==-1) return;
  scene->Activate(INDEXshape2swift(s->index));
}

void SwiftInterface::deactivate(mlr::Shape *s) {
  if(INDEXshape2swift(s->index)==-1) return;
  scene->Deactivate(INDEXshape2swift(s->index));
}

void SwiftInterface::pushToSwift(const mlr::KinematicWorld& world) {
  //CHECK_EQ(INDEXshape2swift.N,world.shapes.N,"the number of shapes has changed");
  CHECK(INDEXshape2swift.N <= world.shapes.N, "the number of shapes has changed");
  mlr::Matrix rot;
  for_list(mlr::Shape,  s,  world.shapes) {
    rot = s->X.rot.getMatrix();
    if(s->index<INDEXshape2swift.N && INDEXshape2swift(s->index)!=-1) {
      scene->Set_Object_Transformation(INDEXshape2swift(s->index), rot.p(), s->X.pos.p());
      if(!s->cont) scene->Deactivate(INDEXshape2swift(s->index));
      //else         scene->Activate( INDEXshape2swift(s->index) );
    }
  }
}

void SwiftInterface::pullFromSwift(mlr::KinematicWorld& world, bool dumpReport) {
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
    listResize(world.proxies, 0);
    cout <<"... catching error '" <<msg <<"' -- SWIFT failed! .. no proxies for this posture!!..." <<endl;
    return;
  }
  
  if(dumpReport) {
    cout <<"contacts: np=" <<np <<endl;
    for(k=0, i=0; i<np; i++) {
      cout <<"* Shape '" <<world.shapes(oids[i <<1])->name <<"' vs. Shape '" <<world.shapes(oids[(i <<1)+1])->name <<"'" <<endl;
      cout <<"  #contacts = " <<num_contacts[i] <<endl;
      for(j=0; j<num_contacts[i]; j++, k++) {
        cout <<"  - contact " <<j <<endl;
        cout <<"    distance= " <<dists[k] <<endl;
        cout <<"    points  = " <<nearest_pts[6*k+0] <<' ' <<nearest_pts[6*k+1] <<' ' <<nearest_pts[6*k+2] <<' ' <<nearest_pts[6*k+3] <<' ' <<nearest_pts[6*k+4] <<' ' <<nearest_pts[6*k+5] <<endl;
        cout <<"    normals = " <<normals[3*k+0] <<' ' <<normals[3*k+1] <<' ' <<normals[3*k+2] <<endl;
      }
    }
  }
  
  //count total number of new proxies:
  for(k=0, i=0; i<np; i++) {
    if(num_contacts[i]>=0) k+=num_contacts[i];
    if(num_contacts[i]==-1) k++;
  }
  
  listResize(world.proxies, k);
  
  //add contacts to list
  mlr::Proxy *proxy;
  int a, b;
  for(k=0, i=0; i<np; i++) {
    a=INDEXswift2shape(oids[i <<1]);
    b=INDEXswift2shape(oids[(i <<1)+1]);
    //CHECK(ids(a)==a && ids(b)==b, "shape index does not coincide with swift index");
    
    //non-penetrating pair of objects
    if(num_contacts[i]>=0) for(j=0; j<num_contacts[i]; j++, k++) {
        proxy=world.proxies(k);
        proxy->a=a;
        proxy->b=b;
        proxy->d = dists[k];
        proxy->normal.set(&normals[3*k+0]);
        proxy->normal.normalize();
        //swift returns nearest points in the local frame -> transform them
        proxy->posA.set(&nearest_pts[6*k+0]);  proxy->posA = world.shapes(a)->X * proxy->posA;
        proxy->posB.set(&nearest_pts[6*k+3]);  proxy->posB = world.shapes(b)->X * proxy->posB;
        proxy->cenA = world.shapes(a)->X.pos;
        proxy->cenB = world.shapes(b)->X.pos;
//        if(world.shapes(a)->type==mlr::ST_mesh) proxy->cenA = world.shapes(a)->X * world.shapes(a)->mesh.getMeanVertex(); else proxy->cenA = world.shapes(a)->X.pos;
//        if(world.shapes(b)->type==mlr::ST_mesh) proxy->cenB = world.shapes(b)->X * world.shapes(b)->mesh.getMeanVertex(); else proxy->cenB = world.shapes(b)->X.pos;
        proxy->cenN = proxy->cenA - proxy->cenB; //normal always points from b to a
        proxy->cenD = proxy->cenN.length();
        proxy->cenN /= proxy->cenD;
      }
      
    //penetrating pair of objects
    if(num_contacts[i]==-1) {
      proxy=world.proxies(k);
      proxy->a=a;
      proxy->b=b;
      proxy->d = -.0;
//      if(world.shapes(a)->type==mlr::ST_mesh) proxy->cenA = world.shapes(a)->X * world.shapes(a)->mesh.getMeanVertex(); else proxy->cenA = world.shapes(a)->X.pos;
//      if(world.shapes(b)->type==mlr::ST_mesh) proxy->cenB = world.shapes(b)->X * world.shapes(b)->mesh.getMeanVertex(); else proxy->cenB = world.shapes(b)->X.pos;
      proxy->cenA = world.shapes(a)->X.pos;
      proxy->cenB = world.shapes(b)->X.pos;
      proxy->cenN = proxy->cenA - proxy->cenB; //normal always points from b to a
      proxy->cenD = proxy->cenN.length();
      proxy->cenN /= proxy->cenD;
      
      //copy to pos..
      proxy->posA = proxy->cenA;
      proxy->posB = proxy->cenB;
      proxy->normal = proxy->cenN;
      
      ///! IN PENETRATION we measure d as -1+(distance between object centers) - that gives a well-defined (though non-smooth) gradient!
//      proxy->d += -1.+(proxy->posA-proxy->posB).length();
      k++;
    }

    double ab_radius = mlr::MAX(proxy->d,0.) + 1.1*(world.shapes(a)->mesh_radius + world.shapes(b)->mesh_radius);
    if(proxy->cenD>ab_radius){
      //MLR_MSG("shit");
    }
  }
  CHECK_EQ(k , (int)world.proxies.N, "");
  
  //add pointClound stuff to list
  if(global_ANN) {
    uint i, _i=0;
    arr R(3, 3), t(3);
    arr v, dists, _dists;
    intA idx, _idx;
    for_list(mlr::Shape,  s,  world.shapes) {
      if(!s->cont || s==global_ANN_shape) continue;
      
      //relative rotation and translation of shapes
      mlr::Transformation rel;
      rel.setDifference(global_ANN_shape->X, s->X);
      rel.rot.getMatrix(R.p);
      t = conv_vec2arr(rel.pos);
      
      //check for each vertex
      for(i=0; i<s->mesh.V.d0; i++) {
        v = s->mesh.V[i];
        v = R*v + t;
        global_ANN->getkNN(dists, idx, v, 1);
        if(!i || dists(0)<_dists(0)) {
          _i=i;  _dists=dists;  _idx=idx;
        }
      }
      if(_dists(0)>cutoff) continue;
      
      proxy = new mlr::Proxy;
      world.proxies.append(proxy);
      proxy->a=global_ANN_shape->index;
      proxy->b=s->index;
      proxy->d = _dists(0);
      proxy->posA.set(&global_ANN_shape->mesh.V(_idx(0), 0));  proxy->posA = global_ANN_shape->X * proxy->posA;
      proxy->posB.set(&s->mesh.V(_i, 0));                      proxy->posB = s->X * proxy->posB;
      proxy->normal = proxy->posA - proxy->posB;
      proxy->normal.normalize();
    }
  }
}

void SwiftInterface::step(mlr::KinematicWorld& world, bool dumpReport) {
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

#else
#include <Core/util.h>
  void SwiftInterface::step(mlr::KinematicWorld &world, bool dumpReport=false){}
  void SwiftInterface::pushToSwift() {}
  void SwiftInterface::pullFromSwift(const KinematicWorld &world, bool dumpReport) {}

  void SwiftInterface::reinitShape(const mlr::Shape *s) {}
//  void close();
  void SwiftInterface::deactivate(mlr::Shape *s1, mlr::Shape *s2) {}
  void SwiftInterface::deactivate(const mlr::Array<mlr::Shape*>& shapes) {}
  void SwiftInterface::deactivate(const mlr::Array<mlr::Body*>& bodies) {}
  void SwiftInterface::initActivations(const KinematicWorld &world, uint parentLevelsToDeactivate=3) {}
  void SwiftInterface::swiftQueryExactDistance() {}
#endif
/** @} */
