#pragma once

#include <Core/array.h>
#include <Core/keyValueGraph.h>

struct G4Data {
  struct sG4Data;
  sG4Data *s;

  G4Data();
 ~G4Data();

  void load(const char *data_fname, const char *meta_fname, const char *poses_fname, bool interpolate = false);
  void save(const char *data_fname);

  const StringA& sensors();
  const StringA& subjects();
  const StringA& objects();
  const StringA& agents();
  const StringA& limbs();
  const StringA& digits();

  const StringA& digitsof(const String &limb);
  const StringA& sublimbs(const String &limb);
  const String& suplimb(const String &limb);

  uint numFrames();
  uint numDim(const char *bam);

  template<typename T>
  void appendMeta(const char *name, const T &data);
  void appendBam(const char *name, const arr &data);

  bool hasBAM(const char *type);
  arr query(const char *type);
  arr query(const char *type, const char *sensor);
  arr query(const char *type, const char *sensor, uint f);
};

