#pragma once

#include <Core/array.h>

struct G4Data {
  struct sG4Data *s;

  G4Data();
  ~G4Data();

  void loadData(const char *meta_fname, const char *poses_fname);

  int getNumTimesteps();
  int getNumSensors(const char *key = NULL);

  arr query(int t, const char *key = NULL);
  arr query(const char *key = NULL);
};
