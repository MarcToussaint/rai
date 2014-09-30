#include <Core/array.h>
#include <Core/util.h>
#include <Core/geo.h>
#include <Core/keyValueGraph.h>
#include <Core/registry.h>
#include "g4data.h"

// =============================================================================
// G4ID
//
#define HSI(hid, sid) (3 * (hid) + (sid))

struct G4ID::sG4ID {
  KeyValueGraph kvg, kvg_sensors, kvg_sublimbs, kvg_suplimbs, kvg_digitsof, kvg_sensorsof;
  StringA sensors, struct_sensors, unstruct_sensors,
          subjects, objects,
          agents, limbs, digits;
  uintA hsitoi, itohsi;
};

G4ID::G4ID(): s(new sG4ID()) { }
G4ID::~G4ID() { delete s; }

void G4ID::clear() {
  s->kvg.clear();
  s->kvg_sensors.clear();
  s->kvg_sublimbs.clear();
  s->kvg_suplimbs.clear();
  s->kvg_digitsof.clear();
  s->kvg_sensorsof.clear();

  s->sensors.clear();
  s->struct_sensors.clear();
  s->unstruct_sensors.clear();

  s->subjects.clear();
  s->objects.clear();

  s->agents.clear();
  s->limbs.clear();
  s->digits.clear();
}

void readItem(KeyValueGraph *i, uintA &hsitoi, uintA &itohsi, int ind) {
  uint hid, sid, hsi, hsitoiN;
  
  hid = (uint)*i->getValue<double>("hid");
  sid = (uint)*i->getValue<double>("sid");
  hsi = HSI(hid, sid);

  if(hsi >= hsitoi.N) {
    hsitoiN = hsitoi.N;
    hsitoi.resizeCopy(hsi+1);
    hsitoi.subRange(hsitoiN, hsi)() = -1;
  }
  hsitoi(hsi) = ind;
  itohsi.append(hsi);

  // cout << "mapping hsi " << hsi << " i " << *i << endl;
}

void G4ID::load(const char *meta) {
  String name_agent, name_limb, name_digit, name_object, name_part;
  MT::Array<KeyValueGraph*> kvg_agents, kvg_limbs, kvg_digits, kvg_objects, kvg_parts;
  uint i = 0;

  bool structured;

  FILE(meta) >> s->kvg;

  kvg_agents = s->kvg.getTypedValues<KeyValueGraph>("agent");
  for(KeyValueGraph *a: kvg_agents) {
    a->getValue(name_agent, "name");
    s->agents.append(name_agent);
    s->subjects.append(name_agent);

    s->kvg_suplimbs.append(name_agent, new String(""));
    s->kvg_sublimbs.append(name_agent, new StringA());

    s->kvg_digitsof.append(name_agent, new StringA());
    s->kvg_sensorsof.append(name_agent, new StringA());

    kvg_limbs = a->getTypedValues<KeyValueGraph>("limb");
    for(KeyValueGraph *l: kvg_limbs) {
      l->getValue(name_limb, "name");
      s->limbs.append(name_limb);
      s->subjects.append(name_limb);

      s->kvg_suplimbs.append(name_limb, new String(name_agent));
      s->kvg_sublimbs.append(name_limb, new StringA());
      s->kvg_sublimbs.getValue<StringA>(name_agent)->append(name_limb);

      s->kvg_digitsof.append(name_limb, new StringA());
      s->kvg_sensorsof.append(name_limb, new StringA());

      kvg_digits = l->getTypedValues<KeyValueGraph>("digit");
      for(KeyValueGraph *d: kvg_digits) {
        d->getValue(name_digit, "name");
        s->digits.append(name_digit);
        s->subjects.append(name_digit);
        s->sensors.append(name_digit);
        s->unstruct_sensors.append(name_digit);

        s->kvg_suplimbs.append(name_digit, new String(name_limb));
        s->kvg_sublimbs.append(name_digit, new StringA());
        s->kvg_sublimbs.getValue<StringA>(name_limb)->append(name_digit);

        s->kvg_digitsof.append(name_digit, new StringA());
        s->kvg_digitsof.getValue<StringA>(name_digit)->append(name_digit);
        s->kvg_digitsof.getValue<StringA>(name_limb)->append(name_digit);
        s->kvg_digitsof.getValue<StringA>(name_agent)->append(name_digit);

        s->kvg_sensorsof.append(name_digit, new StringA());
        s->kvg_sensorsof.getValue<StringA>(name_digit)->append(name_digit); 
        s->kvg_sensorsof.getValue<StringA>(name_limb)->append(name_digit); 
        s->kvg_sensorsof.getValue<StringA>(name_agent)->append(name_digit); 

        s->kvg_sensors.append(name_digit, d);

        readItem(d, s->hsitoi, s->itohsi, i++);
      }
    }
  }

  kvg_objects = s->kvg.getTypedValues<KeyValueGraph>("object");
  for(KeyValueGraph *o: kvg_objects) {
    o->getValue(name_object, "name");
    s->objects.append(name_object);
    if(o->getValue(structured, "structured") && structured) {
      kvg_parts = o->getTypedValues<KeyValueGraph>("part");
      s->kvg_sensorsof.append(name_object, new StringA());
      for(KeyValueGraph *p: kvg_parts) {
        p->getValue(name_part, "name");
        s->struct_sensors.append(name_part);
        s->sensors.append(name_part);

        s->kvg_sensorsof.getValue<StringA>(name_object)->append(name_part); 
        s->kvg_sensors.append(name_part, p);

        readItem(p, s->hsitoi, s->itohsi, i++);
      }
    }
    else {
      s->unstruct_sensors.append(name_object);
      s->sensors.append(name_object);

      s->kvg_sensors.append(name_object, o);
      s->kvg_sensorsof.append(name_object, new StringA());
      s->kvg_sensorsof.getValue<StringA>(name_object)->append(name_object); 

      readItem(o, s->hsitoi, s->itohsi, i++);
    }
  }
}

const StringA& G4ID::sensors() { return s->sensors; }
const StringA& G4ID::struct_sensors() { return s->struct_sensors; }
const StringA& G4ID::unstruct_sensors() { return s->unstruct_sensors; }
const StringA& G4ID::sensorsof(const char *obj) { return *s->kvg_sensorsof.getValue<StringA>(STRING(obj)); }

const StringA& G4ID::subjects() { return s->subjects; }
const StringA& G4ID::objects() { return s->objects; }

const StringA& G4ID::agents() { return s->agents; }
const StringA& G4ID::limbs() { return s->limbs; }
const StringA& G4ID::digits() { return s->digits; }

const StringA& G4ID::digitsof(const String &limb) { return *s->kvg_digitsof.getValue<StringA>(limb); }
const StringA& G4ID::sublimbs(const String &limb) { return *s->kvg_sublimbs.getValue<StringA>(limb); }
const String& G4ID::suplimb(const String &limb) { return *s->kvg_suplimbs.getValue<String>(limb); }

uint G4ID::hsitoi(uint hsi) {
  return s->hsitoi(hsi);
}

uint G4ID::itohsi(uint i) {
  return s->itohsi(i);
}

int G4ID::i(const char *sensor) {
  return s->hsitoi(hsi(sensor));
}

int G4ID::hsi(const char *sensor) {
  KeyValueGraph *skvg = s->kvg_sensors.getValue<KeyValueGraph>(sensor);

  uint hid = *skvg->getValue<double>("hid");
  uint sid = *skvg->getValue<double>("sid");

  return HSI(hid, sid);
}

const char *G4ID::sensor(uint hsi) {
  return s->sensors(s->hsitoi(hsi));
}

void G4ID::write(std::ostream &os) const {
  os << "G4ID" << endl;
}

// =============================================================================
// G4Rec
//

G4Rec::G4Rec() { setDefaultParams(); }
G4Rec::~G4Rec() { }

void G4Rec::setDefaultParams() {
  params.clear();
  // params.append("wlen", new uint(120u));
  // params.append("thinning", new uint(12u));
  params.append("wlen", new uint(120u));
  params.append("thinning", new uint(12u));
}

void G4Rec::load(const char *recdir, bool interpolate) {
  dir = STRING(recdir);
  g4id.load(STRING(recdir << "meta.kvg"));

  ifstream datafin, tstampfin;
  MT::open(datafin, STRING(recdir << "poses.dat"));
  tstampfin.open(STRING(recdir << "poses.dat.times"));
  arr dataframe;

  nsensors = g4id.sensors().N;

  bool m;
  boolA pm(nsensors);
  pm.setZero(false);
  uint currfnum;
  int hsi;
  double currtstamp;
  arr data, tstamp;
  boolA missing;
  MT::Array<intA> missingno(nsensors), missingf(nsensors);
  for(nframes = 0;; nframes++) {
    datafin >> dataframe;
    if(tstampfin.good()) {
      tstampfin >> currfnum >> currtstamp;
      tstamp.append(currtstamp);
    }
    if(!dataframe.N || !datafin.good()) break;
    for(uint i = 0; i < nsensors; i++) {
      hsi = g4id.itohsi(i);
      if(hsi != -1) {
        data.append(dataframe[hsi]);
        m = length(dataframe[hsi]) == 0;
        missing.append(m);
        if(m && pm(i))
          missingno(i).last()++;
        else if(m && !pm(i)) {
          missingno(i).append(1);
          missingf(i).append(nframes);
        }
        pm(i) = m;
      }
    }
  }
  data.reshape(nframes, nsensors, 7);
  missing.reshape(nframes, nsensors);

  // setting quaternions as a continuous path on the 4d sphere
  arr dataquat, dataquatprev;
  dataquatprev = data[0].sub(0, -1, 3, -1);
  for(uint f = 1; f < data.d0; f++) {
    for(uint i = 0; i < data.d1; i++) {
      dataquat.referToSubRange(data.subDim(f, i)(), 3, -1);
      if(sum(dataquat % dataquatprev[i]) < 0)
        dataquat *= -1.;
      if(!length(dataquatprev[i]) || length(dataquat))
        dataquatprev[i]() = dataquat;
    }
  }
  if(interpolate) { // interpolating missing measures
    for(uint i = 0; i < nsensors; i++) {
      for(uint j = 0; j < missingno(i).N; j++) {
        uint t = missingf(i).elem(j);
        uint no = missingno(i).elem(j);
        if(t == 0) // set all equal to first
          for(uint tt = 0; tt < no; tt++)
            data[tt][i] = data[no][i];
        else if(t+no < nframes) { // interpolate between t-1 and t+missingno(i)
          arr s0 = data[t-1][i];
          arr sF = data[t+no][i];
          ors::Quaternion q0(s0(3), s0(4), s0(5), s0(6));
          ors::Quaternion qF(sF(3), sF(4), sF(5), sF(6));
          ors::Quaternion qt;

          arr diff = sF - s0;
          for(uint tt = 0; tt < no; tt++) {
            data[t+tt][i] = s0 + diff*(tt+1.)/(no+1.);
            qt.setInterpolate((tt+1.)/(no+1.), q0, qF);
            data(t+tt, i, 3) = qt.w;
            data(t+tt, i, 4) = qt.x;
            data(t+tt, i, 5) = qt.y;
            data(t+tt, i, 6) = qt.z;
          }
        }
        else // set all equal to last
          for(uint tt = 0; tt < no; tt++)
            data[nframes-tt-1][i] = data[nframes-no-1][i];
      }
    }
  }
  // setting up default BAMs
  arr datatmp;
  tensorPermutation(datatmp, data, {1, 0, 2});
  arr pos = datatmp.sub(0, -1, 0, -1, 0, 2);
  arr quat = datatmp.sub(0, -1, 0, -1, 3, -1);
  arr pose = datatmp;

  // organizing data about this dir
  appendBam("pos", pos);
  appendBam("quat", quat);
  appendBam("pose", pose);

  // loading annotation, if any..
  try {
    FILE(STRING(recdir << "ann_kvg.kvg")) >> kvgann;
    uint from, to;

    arr *ann;
    for(Item *pair: kvgann) {
      ann = new arr(nframes);
      ann->setZero();
      for(Item *lock: *pair->getValue<KeyValueGraph>()) {
        from = (uint)*lock->getValue<KeyValueGraph>()->getValue<double>("from");
        to = (uint)*lock->getValue<KeyValueGraph>()->getValue<double>("to");
        ann->subRange(from, to) = 1;
      }
      pair->getValue<KeyValueGraph>()->append("ann", ann);
    }
  }
  catch(const char *e) {
    cout << "No annotations in " << recdir << "." << endl;
  }
}

G4ID &G4Rec::id() { return g4id; }

bool G4Rec::hasAnn(const char *sensor1, const char *sensor2) {
  for(Item *pair: kvgann)
    if(g4id.sensorsof(pair->keys(0)).contains(STRING(sensor1))
    && g4id.sensorsof(pair->keys(1)).contains(STRING(sensor2)))
      return true;
  return false;
}

arr G4Rec::ann(const char *sensor1, const char *sensor2) {
  for(Item *pair: kvgann)
    if(g4id.sensorsof(pair->keys(0)).contains(STRING(sensor1))
    && g4id.sensorsof(pair->keys(1)).contains(STRING(sensor2)))
      return *pair->getValue<KeyValueGraph>()->getValue<arr>("ann");
  return arr();
}

uint G4Rec::numSensors() const { return nsensors; }
uint G4Rec::numFrames() const { return nframes; }
uint G4Rec::numDim(const char *bam) { return kvg.getValue<arr>(STRINGS("bam", bam))->d2; }

void G4Rec::appendBam(const char *bam, const arr &data) {
  Item *i = kvg.getItem("bam", bam);

  if(!i)
    kvg.append("bam", bam, new arr(data));
  else
    *i->getValue<arr>() = data; // replacing
}

bool G4Rec::hasBam(const char *bam) {
  return kvg.getItem(STRINGS("bam", bam)) != NULL;
}

arr G4Rec::query(const char *bam) {
  Item *i = kvg.getItem(STRINGS("bam", bam));
  CHECK(i != nullptr, STRING("BAM '" << bam << "' does not exist."));

  if(0 == strcmp(bam, "pose")) {
    arr data, dataPos, dataQuat;

    dataPos.referTo(*kvg.getValue<arr>(STRINGS("bam", "pos")));
    dataQuat.referTo(*kvg.getValue<arr>(STRINGS("bam", "quat")));
    data.append(dataPos);
    data.append(dataQuat);
    data.reshape(nsensors, nframes, 7);
    return data;
  }
  return *kvg.getValue<arr>(STRINGS("bam", bam));
}

arr G4Rec::query(const char *type, const char *sensor) {
  Item *i = kvg.getItem(STRINGS("bam", type));
  CHECK(i != nullptr, STRING("BAM '" << type << "' does not exist."));

  int is = g4id.i(sensor);
  CHECK(is >= 0, STRING("Sensor '" << sensor << "' does not exist."));

  if(0 == strcmp(type, "pose")) {
    arr xPos, xQuat;
    xPos.referTo(*kvg.getValue<arr>("pos"));
    xQuat.referTo(*kvg.getValue<arr>("quat"));

    uint nframes = numFrames();

    arr x;
    /* x.append(xPos); */
    /* x.append(xQuat); */
    /* x.reshape(s->g4id.sensors().N, s->nframes, 7); */
    /* return x[i]; */
    x.append(xPos[is]);
    x.append(xQuat[is]);
    x.reshape(nframes, 7);
    return x;
  }
  return i->getValue<arr>()->operator[](is);
}

arr G4Rec::query(const char *type, const char *sensor, uint f) {
  Item *i = kvg.getItem(STRINGS("bam", type));
  CHECK(i != nullptr, STRING("BAM '" << type << "' does not exist."));

  int is = g4id.i(sensor);
  CHECK(is >= 0, STRING("Sensor '" << sensor << "' does not exist."));

  if(0 == strcmp(type, "pose")) {
    arr x;
    x.append(kvg.getValue<arr>("pos")->subDim(is, f));
    x.append(kvg.getValue<arr>("quat")->subDim(is, f));
    return x;
  }
  return i->getValue<arr>()->subDim(is, f);
}

/* arr G4Rec::query(const char *type, const char *sensor1, const char *sensor2) { */
/*   // TODO check that the type works for 2 sensors.... */
/*   // e.g. check that it is not "poses" */

/*   KeyValueGraph *skvg1 = s->kvg_sensors.getValue<KeyValueGraph>(sensor1); */
/*   KeyValueGraph *skvg2 = s->kvg_sensors.getValue<KeyValueGraph>(sensor2); */
/*   CHECK(s->kvg.getItem(type) != NULL, STRING("BAM '" << type << "' does not exist.")); */
/*   CHECK(skvg1, STRING("Sensor '" << sensor1 << "' does not exist.")); */
/*   CHECK(skvg2, STRING("Sensor '" << sensor2 << "' does not exist.")); */

/*   uint hid1, sid1, i1; */
/*   uint hid2, sid2, i2; */
  
/*   hid1 = *skvg1->getValue<double>("hid"); */
/*   sid1 = *skvg1->getValue<double>("sid"); */
/*   i1 = s->kvg.getValue<arr>("hsitoi")->elem(HSI(hid1, sid1)); */
  
/*   hid2 = *skvg2->getValue<double>("hid"); */
/*   sid2 = *skvg2->getValue<double>("sid"); */
/*   i2 = s->kvg.getValue<arr>("hsitoi")->elem(HSI(hid2, sid2)); */

/*   return s->kvg.getValue<arr>(type)->subDim(i1, i2); */
/* } */

/* arr G4Data::query(const char *type, const char *sensor1, const char *sensor2, uint f) { */
/*   KeyValueGraph *skvg1 = s->kvg_sensors.getValue<KeyValueGraph>(sensor1); */
/*   KeyValueGraph *skvg2 = s->kvg_sensors.getValue<KeyValueGraph>(sensor2); */
/*   CHECK(s->kvg.getItem(type) != NULL, STRING("BAM '" << type << "' does not exist.")); */
/*   CHECK(skvg1, STRING("Sensor '" << sensor1 << "' does not exist.")); */
/*   CHECK(skvg2, STRING("Sensor '" << sensor2 << "' does not exist.")); */

/*   uint hid1, sid1, i1; */
/*   uint hid2, sid2, i2; */
  
/*   hid1 = *skvg1->getValue<double>("hid"); */
/*   sid1 = *skvg1->getValue<double>("sid"); */
/*   i1 = s->kvg.getValue<arr>("hsitoi")->elem(HSI(hid1, sid1)); */
  
/*   hid2 = *skvg2->getValue<double>("hid"); */
/*   sid2 = *skvg2->getValue<double>("sid"); */
/*   i2 = s->kvg.getValue<arr>("hsitoi")->elem(HSI(hid2, sid2)); */

/*   return s->kvg.getValue<arr>("bam", type)->subDim(i1, i2, f); */
/* } */

void G4Rec::computeDPos(const char *sensor) {
  String typeDPos;
  typeDPos << sensor << "_dPos";

  arr bamDPos(nsensors, nframes, numDim("pos"));
  bamDPos.setZero();

  arr posX, quatX, posY;
  posX = query("pos", sensor);
  posY = query("pos");
  quatX = query("quat", sensor);
  ors::Vector pX, pY, p, A;
  ors::Quaternion qX;
  for(uint is = 0; is < nsensors; is++) {
    for(uint f = 0; f < nframes; f++) {
      pX.set(posX[f].p);
      pY.set(posY[is][f].p);
      qX.set(quatX[f].p);
      if(f == 0)
        A = qX * (pY - pX);
      p = qX * (pY - pX) - A;
      bamDPos[is][f]() = ARR(p.x, p.y, p.z);
    }
  }

  appendBam(typeDPos, bamDPos);
}

void G4Rec::computeDQuat(const char *sensor) {
  String typeDQuat;
  typeDQuat << sensor << "_dQuat";

  arr bamDQuat(nsensors, nframes, numDim("quat"));
  bamDQuat.setZero();

  arr quatX, quatY;
  quatX = query("quat", sensor);
  quatY = query("quat");
  ors::Quaternion qX, qY, quat, A;
  for(uint j = 0; j < nsensors; j++) {
    for(uint f = 0; f < nframes; f++) {
      qX.set(quatX[f].p);
      qY.set(quatY[j][f].p);
      if(f == 0)
        A = qX / qY;
      quat = qX / ( A * qY );
      bamDQuat[j][f]() = { quat.w, quat.x, quat.y, quat.z };
    }
  }

  appendBam(typeDQuat, bamDQuat);
}

// void G4Rec::computeVar(const char *type) {
//   String typeVar = STRING(type << "Var");
  
//   arr bam, bamVar, window, windowMean;
//   uint wlen, nframes;
  
//   wlen = *get<uint>("wlen");
//   nframes = numFrames();

//   bam = query(type);
//   bamVar.resize(bam.d0, bam.d1);
//   bamVar.setZero();

//   uint ff = wlen / 2;
//   uint ft = nframes - ff;
//   for(uint i = 0; i < bam.d0; i++) {
//     for(uint fi = ff; fi < ft; fi++) {
//       uint wi = fi - ff;
//       window.referToSubRange(bam[i], wi, wi + wlen - 1);
//       windowMean = sum(window, 0) / (double)wlen;
//       windowMean = ~repmat(windowMean, 1, wlen);
//       bamVar(i, fi) = sumOfSqr(window - windowMean);
//     }
//   }
//   bamVar /= (double)wlen;

//   appendBam(typeVar, bamVar);
// }

void G4Rec::computeVar(const char *type) {
  String typeVar = STRING(type << "Var");
  
  arr bam, bamVar, window, windowMean, windowMeanRep;
  uint wlen, thinning, nframes, nframes_thin;
  
  wlen = *get<uint>("wlen");
  thinning = *get<uint>("thinning");

  nframes = numFrames();
  nframes_thin = nframes / thinning;

  bam = query(type);
  bamVar.resize(bam.d0, nframes_thin);
  bamVar.setZero();

  // uint ff = wlen / 2;
  // uint ft = nframes - ff;
  uint ff = wlen - 1;
  // TODO center the window again!!
  // uint ft = nframes - ff;
  // NB changed the window. Not more centered, but all on the left.
  for(uint i = 0; i < bam.d0; i++) {
    for(uint f_thin = 0; f_thin < nframes_thin; f_thin++) {
      uint f = (f_thin + 1) * thinning - 1;
      if(f < ff) continue;
      // if(f > ft) break;
      window.referToSubRange(bam[i], f - wlen + 1, f);
      windowMean = sum(window, 0) / (double)wlen;
      // windowMeanRep = ~repmat(windowMean, 1, wlen);
      // bamVar(i, f_thin) = sumOfSqr(window - windowMeanRep);
      bamVar(i, f_thin) = sumOfSqr(window - ~repmat(windowMean, 1, wlen));
    }
  }
  bamVar /= (double)wlen;

  appendBam(typeVar, bamVar);
}

void G4Rec::computeVar(const StringA &types) {
  for(const String &type: types)
    computeVar(type);
}

G4RawSeq G4Rec::rawseq(const char *sens1, const char *sens2) {
  G4RawSeq seq;
  seq.set(*this, sens1, sens2);
  return seq;
}

G4RawSeqL G4Rec::rawseqlist(const char *obj1, const char *obj2) {
  G4RawSeqL seqlist;
  G4RawSeq *seq;

  const StringA sensors1 = (obj1? g4id.sensorsof(obj1): g4id.sensors());
  const StringA sensors2 = (obj2? g4id.sensorsof(obj2): g4id.sensors());
  for(const String &sens1: sensors1) {
    for(const String &sens2: sensors2) {
      seq = new G4RawSeq();
      seq->set(*this, sens1, sens2);
      seqlist.append(seq);
    }
  }
  return seqlist;
}

G4FeatSeq G4Rec::featseq(const char *sens1, const char *sens2) {
  G4FeatSeq seq;
  seq.set(*this, sens1, sens2);
  return seq;
}

G4FeatSeqL G4Rec::featseqlist(bool with_ann, const char *obj1, const char *obj2) {
  G4FeatSeqL seqlist;
  G4FeatSeq *seq;

  bool checklist[nsensors][nsensors];
  uint i1, i2;
  for(i1 = 0; i1 < nsensors; i1++)
    for(i2 = 0; i2 < nsensors; i2++)
      checklist[i1][i2] = false;

  StringA sensors1 = (obj1? g4id.sensorsof(obj1): g4id.sensors());
  StringA sensors2 = (obj2? g4id.sensorsof(obj2): g4id.sensors());
  for(const String &sens1: sensors1) {
    for(const String &sens2: sensors2) {
      i1 = g4id.i(sens1);
      i2 = g4id.i(sens2);
      if(i1 != i2 && !checklist[i1][i2] && (!with_ann || hasAnn(sens1, sens2))) {
        checklist[i1][i2] = checklist[i2][i1] = true;
        seq = new G4FeatSeq();
        seq->set(*this, sens1, sens2);
        seqlist.append(seq);
      }
    }
  }
  return seqlist;
}

void G4Rec::write(std::ostream &os) const {
  os << "G4Rec: " << dir << endl;
}

// =============================================================================
// G4Data
//

G4Data::G4Data() { }
G4Data::~G4Data() { }

String &G4Data::base() { return basedir; }

void G4Data::load(const char *recdir, bool interpolate) {
  G4Rec *g4rec = new G4Rec();
  g4rec->load(STRING(basedir << recdir << "/"), interpolate);
  kvg.append(recdir, g4rec);
}

G4Rec &G4Data::rec(const char *recdir) {
  Item *i = kvg.getItem(recdir);
  CHECK(i, STRING("No recording named '" << recdir << "'."));
  return *i->getValue<G4Rec>();
}

#if 0

void G4Data::clear() {
  for(G4ID *g4id: s->kvg.getTypedValues<G4ID>("g4id"))
    delete g4id;
  for(arr *bam: s->kvg.getTypedValues<arr>("bam"))
    delete bam;
  s->kvg.clear();
}

void G4Data::save(const char *data_fname) {
  /* cout << " * Saving.." << flush; */
  /* s->kvg >> FILE(data_fname); */
  /* cout << " DONE!" << endl; */
}

#endif

void G4Data::write(std::ostream &os) const {
  os << "G4Data" << endl;
  for(Item *i: kvg) {
    os << " * " << i->keys << endl;
  }
}

// =============================================================================
// G4RawSeq
//

G4RawSeq::G4RawSeq() { }
G4RawSeq::G4RawSeq(const G4RawSeq &g4rawseq) {
  // TODO maybe better copy constructor?
  sensor1 = g4rawseq.sensor1;
  sensor2 = g4rawseq.sensor2;
  data1 = g4rawseq.data1;
  data2 = g4rawseq.data2;
  ann = g4rawseq.ann;
  nframes = g4rawseq.nframes;
}

G4RawSeq::~G4RawSeq() { }

void G4RawSeq::set(G4Rec &g4rec, const char *sens1, const char *sens2) {
  G4ID &g4id = g4rec.id();

  sensor1 = STRING(sens1);
  sensor2 = STRING(sens2);
  CHECK(g4id.sensors().contains(sensor1), STRING("Sensor '" << sens1 << "' does not exist in G4Rec"));
  CHECK(g4id.sensors().contains(sensor2), STRING("Sensor '" << sens2 << "' does not exist in G4Rec"));

  data1 = g4rec.query("pose", sensor1);
  data2 = g4rec.query("pose", sensor2);
  ann = g4rec.ann(sensor1, sensor2);
  nframes = ann.d0;
}

bool G4RawSeq::hasAnn() {
  return ann.N;
}

void G4RawSeq::write(std::ostream &os) const {
  os << "G4RawSeq: " << endl;
  os << " * sensors: " << sensor1 << ", " << sensor2 << endl;
  os << " * dims: " << data1.getDim() << ", " << data2.getDim() << endl;
  os << " * ann: " << (ann.N? "yes": "no") << endl;
}


// =============================================================================
// G4FeatSeq
//

G4FeatSeq::G4FeatSeq() { }
G4FeatSeq::G4FeatSeq(const G4FeatSeq &g4featseq) {
  // TODO maybe better copy constructor?
  sensor1 = g4featseq.sensor1;
  sensor2 = g4featseq.sensor2;
  data = g4featseq.data;
  ann = g4featseq.ann;
  ann_thin = g4featseq.ann_thin;
  nframes = g4featseq.nframes;
  nframes_thin = g4featseq.nframes_thin;
}

G4FeatSeq::~G4FeatSeq() { }

void G4FeatSeq::set(G4Rec &g4rec, const char *sens1, const char *sens2) {
  sensor1 = STRING(sens1);
  sensor2 = STRING(sens2);

  g4rec.computeVar("pos");
  g4rec.computeVar("quat");

  String sens1_dPos = STRING(sens1 << "_dPos");
  String sens1_dQuat = STRING(sens1 << "_dQuat");
  String sens1_dPosVar = STRING(sens1_dPos << "Var");
  String sens1_dQuatVar = STRING(sens1_dQuat << "Var");

  g4rec.computeDPos(sens1);
  g4rec.computeDQuat(sens1);
  g4rec.computeVar(sens1_dPos);
  g4rec.computeVar(sens1_dQuat);

  arr sens1_posVar = g4rec.query("posVar", sens1);
  arr sens1_quatVar = g4rec.query("quatVar", sens1);
  arr sens2_posVar = g4rec.query("posVar", sens2);
  arr sens2_quatVar = g4rec.query("quatVar", sens2);
  arr delta_posVar = g4rec.query(sens1_dPosVar, sens2);
  arr delta_quatVar = g4rec.query(sens1_dQuatVar, sens2);
  
  arr g4ann = g4rec.ann(sens1, sens2);
  uint thinning = *g4rec.get<uint>("thinning");
  params.set("thinning", thinning);
  nframes = g4rec.numFrames();
  nframes_thin = sens1_posVar.N;
  for(uint f_thin = 0; f_thin < nframes_thin; f_thin++) {
    data.append({ 1.,
                  sens1_posVar(f_thin),
                  sens1_quatVar(f_thin),
                  sens2_posVar(f_thin),
                  sens2_quatVar(f_thin),
                  delta_posVar(f_thin),
                  delta_quatVar(f_thin)
                  });
  }
  data.reshape(nframes_thin, data.N/nframes_thin);
  ann = g4ann;
  if(ann.N) {
    ann_thin.resize(nframes_thin);
    for(uint f_thin = 0; f_thin < nframes_thin; f_thin++)
      ann_thin(f_thin) = ann((f_thin+1) * thinning - 1);
  }

  // arr g4ann = g4rec.ann(sens1, sens2);
  // uint thinning = *g4rec.get<uint>("thinning");
  // params.set("thinning", thinning);
  // nframes = g4rec.numFrames();
  // nframes_thin = 0;
  // for(uint f = 0; f < nframes; f++) {
  //   ann.append(g4ann(f));
  //   if((f+1) % thinning == 0) {
  //     ann_thin.append(g4ann(f));
  //     data.append({ 1.,
  //                   sens1_posVar(f),
  //                   sens1_quatVar(f),
  //                   sens2_posVar(f),
  //                   sens2_quatVar(f),
  //                   delta_posVar(f),
  //                   delta_quatVar(f)
  //                   });
  //     nframes_thin++;
  //   }
  // }
  // data.reshape(nframes_thin, data.N/nframes_thin);
}

bool G4FeatSeq::hasAnn() {
  return ann.N;
}

void G4FeatSeq::write(std::ostream &os) const {
  os << "G4FeatSeq: " << endl;
  os << " * sensors: " << sensor1 << ", " << sensor2 << endl;
  os << " * dim: " << data.getDim() << endl;
  os << " * ann: " << (ann.N? "yes": "no") << endl;
}
