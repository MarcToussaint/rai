#pragma once

#include <Core/array.h>
#include <Core/keyValueGraph.h>
#include <Gui/opengl.h>

struct G4ID;
struct G4Rec;
struct G4Data;
struct G4RawSeq;
struct G4FeatSeq;

typedef MT::Array<G4RawSeq*> G4RawSeqL;
typedef MT::Array<G4FeatSeq*> G4FeatSeqL;


// =============================================================================
// G4ID
//

struct G4ID {
  struct sG4ID;
  sG4ID *s;

  G4ID();
  ~G4ID();

  void clear();
  void load(const char *meta);

  // TODO change some strings into const char * (mostly the input ones)
  const StringA& sensors();
  const StringA& struct_sensors();
  const StringA& unstruct_sensors();
  const StringA& sensorsof(const char *obj);

  const StringA& subjects();
  const StringA& objects();

  const StringA& agents();
  const StringA& limbs();
  const StringA& digits();

  const StringA& digitsof(const String &limb);
  const StringA& sublimbs(const String &limb);
  const String& suplimb(const String &limb);

  uint hsitoi(uint hsi);
  uint itohsi(uint i);

  int i(const char *sensor);
  int hsi(const char *sensor);
  const char *sensor(uint hsi);

  template <class T>
  MT::Array<T> query(const MT::Array<T> &data, const String &sensor);
  template <class T>
  MT::Array<T> query(const MT::Array<T> &data, const StringA &sensors);

  /* arr query(const String &sensor) jjjj */
  /*   // TODO */
  /*   // check that you have an instance of G4Data, to get the data.. */
  /* } */

  void write(std::ostream &os = std::cout) const;
};
stdOutPipe(G4ID);

// =============================================================================
// G4Rec
//

struct G4Rec {
  String dir;
  G4ID g4id;
  KeyValueGraph kvg, kvgann, params; // TODO ideally, I'd want to use the param class here..

  uint nsensors;
  uint nframes;

  G4Rec();
  ~G4Rec();

  // TODO ideally, I'd want to use the param class here..
  void setDefaultParams();

  template<class T>
  void set(const char *key, const T &value);

  template<class T>
  bool get(const char *key, T &value);

  template<class T>
  T* get(const char *key);

  void load(const char *recdir, bool interpolate = true);

  G4ID &id();
  bool hasAnn(const char *sensor1, const char *sensor2);
  arr ann(const char *sensor1, const char *sensor2);

  uint numSensors() const;
  uint numFrames() const;
  uint numDim(const char *bam);

  void appendBam(const char *bam, const arr &data);
  bool hasBam(const char *type);
  arr query(const char *type);
  arr query(const char *type, const char *sensor);
  arr query(const char *type, const char *sensor, uint f);
  /* arr query(const char *type, const char *sensor1, const char *sensor2); */
  /* arr query(const char *type, const char *sensor1, const char *sensor2, uint f); */

  void computeDPos(const char *sensor);
  void computeDQuat(const char *sensor);
  void computeVar(const char *type);
  void computeVar(const StringA &types);

  G4RawSeq rawseq(const char *sens1, const char *sens2);
  G4RawSeqL rawseqlist(const char *obj1 = nullptr, const char *obj2 = nullptr);

  G4FeatSeq featseq(const char *sens1, const char *sens2);
  G4FeatSeqL featseqlist(bool with_ann, const char *obj1 = nullptr, const char *obj2 = nullptr);

  void write(std::ostream &os = std::cout) const;
};
stdOutPipe(G4Rec);

// =============================================================================
// G4Data
//

struct G4Data {
  String basedir;
  KeyValueGraph kvg;

  G4Data();
 ~G4Data();

  String &base();
  void load(const char *recdir, bool interpolate = true);
  G4Rec &rec(const char *recdir);

#if 0
  void save(const char *data_fname);
  void clear();
#endif

  void write(std::ostream &os = std::cout) const;
};
stdOutPipe(G4Data);

// =============================================================================
// G4RawSeq
//

struct G4RawSeq {
  String sensor1, sensor2;
  arr data1, data2;
  uint nframes;
  arr ann;

  G4RawSeq();
  G4RawSeq(const G4RawSeq &g4rawseq);
  ~G4RawSeq();

  void set(G4Rec &g4rec, const char *sens1, const char *sens2);
  bool hasAnn();

  void write(std::ostream &os = std::cout) const;
};
stdOutPipe(G4RawSeq);


// =============================================================================
// G4FeatSeq
//

struct G4FeatSeq: Parametric {
  String sensor1, sensor2;
  arr data;
  uint nframes, nframes_thin;
  arr ann, ann_thin;

  G4FeatSeq();
  G4FeatSeq(const G4FeatSeq &g4featseq);
  ~G4FeatSeq();

  void set(G4Rec &g4rec, const char *sens1, const char *sens2);
  bool hasAnn();

  void write(std::ostream &os = std::cout) const;
};
stdOutPipe(G4FeatSeq);

#include "g4data_t.h"
