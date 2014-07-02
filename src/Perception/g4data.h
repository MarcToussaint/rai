#pragma once

#include <Core/array.h>
#include <Core/keyValueGraph.h>
#include "g4id.h"

struct G4Data {
  struct sG4Data;
  sG4Data *s;

  G4Data();
 ~G4Data();

  void load(const char *dir, bool interpolate = false);
  void save(const char *data_fname);
  void clear();

  G4ID &id();

  uint numFrames();
  uint numDim(const char *bam);

  /* template<typename T> */
  /* void appendMeta(const char *name, const T &data); */
  void appendBam(const char *name, const arr &data);

  bool hasBam(const char *type);
  arr query(const char *type);
  arr query(const char *type, const char *sensor);
  arr query(const char *type, const char *sensor, uint f);
  /* arr query(const char *type, const char *sensor1, const char *sensor2); */
  /* arr query(const char *type, const char *sensor1, const char *sensor2, uint f); */

  void computeVar(const StringA &types, uint wlen, bool force = false);
  void computeVar(const String &type, uint wlen, bool force = false);
  void computeDPos(const String &b, bool force = false);
  void computeDQuat(const String &b, bool force = false);

  void write(std::ostream &os = std::cout) const;
};
stdOutPipe(G4Data);

