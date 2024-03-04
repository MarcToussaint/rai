/*  ------------------------------------------------------------------
    Copyright (c) 2011-2024 Marc Toussaint
    email: toussaint@tu-berlin.de

    This code is distributed under the MIT License.
    Please see <root-path>/LICENSE for details.
    --------------------------------------------------------------  */

#include "initFol.h"

#include "../Kin/frame.h"

namespace rai {

void initFolStateFromKin(FOL_World& L, const Configuration& C) {
  boolA isSymbol;
  isSymbol.resize(C.frames.N) = false;
  for(Frame* a:C.frames) if(a->ats) {
      if((*a->ats)["logical"]) { //-- explicit setting of logic predicates
        const Graph& G = (*a->ats)["logical"]->graph();
        for(Node* n:G) L.addFact({n->key, a->name});
        isSymbol(a->ID)=true;
      } else if(a->joint && a->joint->type==JT_rigid) { //-- implicit object
        L.addFact({"is_object", a->name});
        isSymbol(a->ID)=true;
        if(a->shape && a->shape->type()==ST_ssBox) L.addFact({"is_box", a->name});
        if(a->shape && a->shape->type()==ST_sphere) L.addFact({"is_sphere", a->name});
        if(a->shape && a->shape->type()==ST_capsule) L.addFact({"is_capsule", a->name});
      }
    }
  for(Frame* a:C.frames) if(isSymbol(a->ID)) {
      Frame* p = a->getUpwardLink();
      if(p && p!=a && isSymbol(p->ID)) {
        L.addFact({"partOf", p->name, a->name});
      }
//      FrameL F;
//      p->getRigidSubFrames(F);
//      for(Frame* b:F) if(b!=a && b->shape && a->ats && (*b->ats)["logical"]) {
//          L.addFact({"partOf", a->name, b->name});
//        }
    }
  for(Frame* a:C.frames) if(isSymbol(a->ID)) {
      Frame* p = a->getUpwardLink();
      if(!p) continue;
      p = p->parent;
      if(p && isSymbol(p->ID)) {
//      FrameL F;
//      while(p) {
//        F.append(p);
//        if(p->joint) break;
//        p=p->parent;
//      }
//      for(Frame* b:F) if(b!=a && b->shape && b->ats && (*b->ats)["logical"]) {
        L.addFact({"on", p->name, a->name});
      }
    }
}

} //namespace
