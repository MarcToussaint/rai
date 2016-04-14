#pragma once

#include <Core/module.h>

struct RosCom_Spinner:Module{
  bool useRos;
  RosCom_Spinner(const char* nodeName="MLRnode");
  void open(){}
  void step();
  void close(){}
};
