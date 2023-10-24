/*  ------------------------------------------------------------------
    Copyright (c) 2011-2020 Marc Toussaint
    email: toussaint@tu-berlin.de

    This code is distributed under the MIT License.
    Please see <root-path>/LICENSE for details.
    --------------------------------------------------------------  */

#include "roopi_Perception.h"

#include "filter.h"
#include "syncFiltered.h"
#include "percViewer.h"

ThreadL newPerceptionFilter(bool view) {
  ThreadL threads;
  threads.append(new Filter());
//  if(view) threads.append(new PercViewer("percepts_filtered"));
  if(view) threads.append(new PerceptViewer("percepts_input"));
  return threads;
}
