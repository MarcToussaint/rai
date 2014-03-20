#pragma once

#include <Core/array.h>
#include <Core/keyValueGraph.h>

struct G4Data {
  KeyValueGraph kvg;
  StringA nameS, nameA, nameO;
  uint numS, numA, numO, numF, numT;

  G4Data();
 ~G4Data();

  void load(const char *data_fname, const char *meta_fname, const char *poses_fname, bool interpolate = false);
  void save(const char *data_fname);

  bool isAgent(uint i);
  bool isAgent(const String &b);
  bool isObject(uint i);
  bool isObject(const String &b);

  StringA getNames();
  String getName(uint i);
  StringA getAgentNames();
  String getAgentName(uint i);
  StringA getObjectNames();
  String getObjectName(uint i);

  uint getIndex(const String &s);
  uint getAgentIndex(const String &a);
  uint getObjectIndex(const String &o);

  uint getNumTypes();
  uint getNumFrames();
  uint getNumSensors();
  uint getNumAgents();
  uint getNumObjects();
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
