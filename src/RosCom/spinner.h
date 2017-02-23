#pragma once

#include <Core/thread.h>

struct RosCom_Spinner : Thread {
  bool useRos;
  RosCom_Spinner(const char* nodeName="MLRnode");
  ~RosCom_Spinner();
  void open(){}
  void step();
  void close(){}
};
