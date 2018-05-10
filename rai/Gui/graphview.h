/*  ------------------------------------------------------------------
    Copyright (c) 2017 Marc Toussaint
    email: marc.toussaint@informatik.uni-stuttgart.de

    This code is distributed under the MIT License.
    Please see <root-path>/LICENSE for details.
    --------------------------------------------------------------  */

#pragma once

#include <Core/graph.h>

struct GraphView {
  struct sGraphView *s;
  bool verbose;
  
  GraphView(Graph& G, const char* title="rai::GraphvizGtk", void *container=NULL);
  ~GraphView();
  
  void writeFile(const char* filename);
  void update();
  void watch();
};
