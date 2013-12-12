#pragma once

#include <Core/array.h>

struct G4Data {
  struct sG4Data *s;

  G4Data();
  ~G4Data();

  void loadData(const char *meta_fname, const char *poses_fname, bool interpolate = false);

  StringA& getNames() const;
  String& getName(uint i) const;

  uint getNumFrames() const;
  uint getNumSensors(const char *key = NULL) const;

  boolA getMissing() const;
  MT::Array<intA> getMissingNo() const;
  MT::Array<intA> getMissingF() const;

  arr query(uint t, const char *key = NULL) const;
  arr query(const char *key = NULL) const;

  arr queryPos(uint t, const char *key = NULL) const;
  arr queryPos(const char *key = NULL) const;

  arr queryQuat(uint t, const char *key = NULL) const;
  arr queryQuat(const char *key = NULL) const;
};
