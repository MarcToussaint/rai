/*  ------------------------------------------------------------------
    Copyright (c) 2011-2020 Marc Toussaint
    email: toussaint@tu-berlin.de

    This code is distributed under the MIT License.
    Please see <root-path>/LICENSE for details.
    --------------------------------------------------------------  */

#pragma once

#include "../Core/graph.h"

namespace rai {
struct GraphView {
  unique_ptr<struct sGraphView> self;
  bool verbose;

  GraphView(Graph& G, const char* title="rai::GraphvizGtk", void* container=nullptr);
  ~GraphView();

  void writeFile(const char* filename);
  void update();
  void watch();
};
}
