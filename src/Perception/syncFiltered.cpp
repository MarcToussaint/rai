/*  ---------------------------------------------------------------------
    Copyright 2014 Marc Toussaint
    email: marc.toussaint@informatik.uni-stuttgart.de
    
    This program is free software: you can redistribute it and/or modify
    it under the terms of the GNU General Public License as published by
    the Free Software Foundation, either version 3 of the License, or
    (at your option) any later version.
    
    This program is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
    GNU General Public License for more details.
    
    You should have received a COPYING file of the GNU General Public License
    along with this program. If not, see <http://www.gnu.org/licenses/>
    -----------------------------------------------------------------  */
#include "syncFiltered.h"

SyncFiltered::SyncFiltered()
  : Thread("SyncFiltered", -1),
    percepts_filtered(this, "percepts_filtered", true) {
  threadOpen();
}

SyncFiltered::~SyncFiltered(){
  threadClose();
}


void SyncFiltered::step(){
  percepts_filtered.writeAccess();

  PerceptL database = percepts_filtered();

  uintA existingIDs;

  for(Percept *p:database){
    p->syncWith(percWorld.set());
    existingIDs.append(p->id);
  }

  // delete non-existing bodies
  percWorld.writeAccess();
  for(mlr::Body *b:percWorld().bodies){
    if(b->name.startsWith("perc_")){
      uint id;
      b->name >>PARSE("perc_") >>id;
      if(!existingIDs.contains(id)){
        delete b;
      }
    }
  }
  percWorld.deAccess();
}


