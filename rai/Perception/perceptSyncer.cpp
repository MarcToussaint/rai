/*  ------------------------------------------------------------------
    Copyright (c) 2011-2020 Marc Toussaint
    email: toussaint@tu-berlin.de

    This code is distributed under the MIT License.
    Please see <root-path>/LICENSE for details.
    --------------------------------------------------------------  */

#include "perceptSyncer.h"
#include "../Kin/frame.h"

SyncFiltered::SyncFiltered(Var<PerceptL>& _percepts, Var<rai::Configuration>& _kin)
  : Thread("SyncFiltered", -1.),
    percepts(this, _percepts, true),
    kin(this, _kin, false) {
  threadOpen();
}

SyncFiltered::~SyncFiltered() {
  threadClose();
}

void SyncFiltered::step() {
  uintA existingIDs;

  percepts.writeAccess();
  for(PerceptPtr& p:percepts()) {
    p->syncWith(kin.set());
    existingIDs.append(p->id);
  }
  percepts.deAccess();

  // delete non-existing bodies
  kin.writeAccess();
  for(uint i=kin().frames.N; i--;) {
    rai::Frame* b = kin().frames.elem(i);
    if(b->name.startsWith("perc_")) {
      uint id;
      b->name.resetIstream();
      b->name >>PARSE("perc_") >>id;
      if(!existingIDs.contains(id)) {
        LOG(-1) <<"DELETING" <<*b;
        delete b;
      }
    }
  }
  kin.deAccess();

}

