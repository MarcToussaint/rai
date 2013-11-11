#pragma once

#include <Core/array.h>

struct G4Data {
  struct sG4Data *s;

  G4Data();
  ~G4Data();

  void loadData(const char *meta_fname, const char *poses_fname, bool interpolate = false);

  int getNumTimesteps() const;
  int getNumSensors(const char *key = NULL) const;

  arr query(uint t, const char *key = NULL) const;
  arr query(const char *key = NULL) const;

  arr queryPos(uint t, const char *key = NULL) const;
  arr queryPos(const char *key = NULL) const;

  arr queryQuat(uint t, const char *key = NULL) const;
  arr queryQuat(const char *key = NULL) const;
};
