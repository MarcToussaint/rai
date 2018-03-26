/*  ------------------------------------------------------------------
    Copyright (c) 2017 Marc Toussaint
    email: marc.toussaint@informatik.uni-stuttgart.de
    
    This code is distributed under the MIT License.
    Please see <root-path>/LICENSE for details.
    --------------------------------------------------------------  */

#include "roopi_Perception.h"

#include <Perception/filter.h>
#include <Perception/syncFiltered.h>
#include <Perception/percViewer.h>

ThreadL newPerceptionFilter(bool view){
  ThreadL threads;
  threads.append(new Filter());
//  if(view) threads.append(new PercViewer("percepts_filtered"));
  if(view) threads.append(new PercViewer("percepts_input"));
  return threads;
}
