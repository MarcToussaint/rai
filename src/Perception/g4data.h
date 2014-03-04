#pragma once

#include <Core/array.h>
#include <Core/keyValueGraph.h>

struct G4Data {
  KeyValueGraph kvg;
  uint numS, numF, numT;

  G4Data();
 ~G4Data();

  void load(const char *data_fname, const char *meta_fname, const char *poses_fname, bool interpolate = false);
  void save(const char *data_fname);

  StringA getNames();
  String getName(uint i);

  uint getNumTypes();
  uint getNumFrames();
  uint getNumSensors();
  uint getNumDim(const char *type);
  uint getNum(const char *key = NULL);

  template<typename T>
  void appendMeta(const char *name, const T &data);
  void appendBam(const char *name, const arr &data);

  bool hasBAM(const char *type);
  arr query(const char *type);
  arr query(const char *type, const char *sensor);
  arr query(const char *type, const char *sensor, uint f);
};

#include "g4data_t.h"
