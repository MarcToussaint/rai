/*  ------------------------------------------------------------------
    Copyright (c) 2017 Marc Toussaint
    email: marc.toussaint@informatik.uni-stuttgart.de

    This code is distributed under the MIT License.
    Please see <root-path>/LICENSE for details.
    --------------------------------------------------------------  */

#include "syncFiltered.h"
#include <Kin/frame.h>

SyncFiltered::SyncFiltered(const char* outputWorld_name)
  : Thread("SyncFiltered", -1.),
    percepts_filtered(this, "percepts_filtered", true),
    outputWorld(this, outputWorld_name) {
  threadOpen();
}

SyncFiltered::~SyncFiltered() {
  threadClose();
}

void SyncFiltered::open() {
//  outputWorld.set() = modelWorld.get();
}

void SyncFiltered::step() {
  uintA existingIDs;
  
  percepts_filtered.writeAccess();
  for(PerceptPtr& p:percepts_filtered()) {
    p->syncWith(outputWorld.set());
    existingIDs.append(p->id);
  }
  percepts_filtered.deAccess();
  
  // delete non-existing bodies
  outputWorld.writeAccess();
  for(rai::Frame *b:outputWorld().frames) {
    if(b->name.startsWith("perc_")) {
      uint id;
      b->name.resetIstream();
      b->name >>PARSE("perc_") >>id;
      if(!existingIDs.contains(id)) {
        delete b;
      }
    }
  }
  outputWorld.deAccess();
  
}

