#pragma once

#include <Core/thread.h>
#include "percept.h"

struct PercViewer : Thread {
  Access_typed<PerceptL> percepts;
  PerceptL copy;
  struct OpenGL *gl;

  PercViewer(const char* percepts_name="percepts_input");
  ~PercViewer();
  void open();
  void step();
  void close();
};
