/*  ------------------------------------------------------------------
    Copyright (c) 2011-2020 Marc Toussaint
    email: toussaint@tu-berlin.de

    This code is distributed under the MIT License.
    Please see <root-path>/LICENSE for details.
    --------------------------------------------------------------  */

#include "g4data.h"
#include "../Core/array.h"
#include "../Core/util.h"
#include "../Geo/geo.h"
#include "../Core/graph.h"

// =============================================================================
// G4ID
//
#define HSI(hid, sid) (3 * (hid) + (sid))

struct sG4ID {
  rai::Graph kvg, kvg_sensors, kvg_sublimbs, kvg_suplimbs, kvg_digitsof, kvg_sensorsof;
  StringA sensors, struct_sensors, unstruct_sensors,
          subjects, objects,
          agents, limbs, digits;
  uintA hsitoi, itohsi;
};

G4ID::G4ID() : self(new sG4ID()) { }
G4ID::~G4ID() {}

void G4ID::clear() {
  self->kvg.clear();
  self->kvg_sensors.clear();
  self->kvg_sublimbs.clear();
  self->kvg_suplimbs.clear();
  self->kvg_digitsof.clear();
  self->kvg_sensorsof.clear();

  self->sensors.clear();
  self->struct_sensors.clear();
  self->unstruct_sensors.clear();

  self->subjects.clear();
  self->objects.clear();

  self->agents.clear();
  self->limbs.clear();
  self->digits.clear();
}

void readNode(rai::Graph* i, uintA& hsitoi, uintA& itohsi, int ind) {
  uint hid, sid, hsi, hsitoiN;

  hid = (uint)i->get<double>("hid");
  sid = (uint)i->get<double>("sid");
  hsi = HSI(hid, sid);

  if(hsi >= hsitoi.N) {
    hsitoiN = hsitoi.N;
    hsitoi.resizeCopy(hsi+1);
    hsitoi({hsitoiN, hsi})() = -1;
  }
  hsitoi(hsi) = ind;
  itohsi.append(hsi);

  // cout << "mapping hsi " << hsi << " i " << *i << endl;
}

void G4ID::load(const char* meta) {
  rai::String name_agent, name_limb, name_digit, name_object, name_part;
  rai::Array<rai::Graph*> kvg_agents, kvg_limbs, kvg_digits, kvg_objects, kvg_parts;
  uint i = 0;

  bool structured;

  FILE(meta) >> self->kvg;

  kvg_agents = self->kvg.getValuesOfType<rai::Graph>("agent");
  for(rai::Graph* a: kvg_agents) {
    a->get(name_agent, "name");
    self->agents.append(name_agent);
    self->subjects.append(name_agent);

    self->kvg_suplimbs.newNode<rai::String>({name_agent}, {}, rai::String(""));
    self->kvg_sublimbs.newNode<StringA>({name_agent}, {}, StringA());

    self->kvg_digitsof.newNode<StringA>({name_agent}, {}, StringA());
    self->kvg_sensorsof.newNode<StringA>({name_agent}, {}, StringA());

    kvg_limbs = a->getValuesOfType<rai::Graph>("limb");
    for(rai::Graph* l: kvg_limbs) {
      l->get(name_limb, "name");
      self->limbs.append(name_limb);
      self->subjects.append(name_limb);

      self->kvg_suplimbs.newNode<rai::String>({name_limb}, {}, rai::String(name_agent));
      self->kvg_sublimbs.newNode<StringA>({name_limb}, {}, StringA());
      self->kvg_sublimbs.get<StringA>(name_agent).append(name_limb);

      self->kvg_digitsof.newNode({name_limb}, {}, StringA());
      self->kvg_sensorsof.newNode({name_limb}, {}, StringA());

      kvg_digits = l->getValuesOfType<rai::Graph>("digit");
      for(rai::Graph* d: kvg_digits) {
        d->get(name_digit, "name");
        self->digits.append(name_digit);
        self->subjects.append(name_digit);
        self->sensors.append(name_digit);
        self->unstruct_sensors.append(name_digit);

        self->kvg_suplimbs.newNode({name_digit}, {}, rai::String(name_limb));
        self->kvg_sublimbs.newNode({name_digit}, {}, StringA());
        self->kvg_sublimbs.get<StringA>(name_limb).append(name_digit);

        self->kvg_digitsof.newNode({name_digit}, {}, StringA());
        self->kvg_digitsof.get<StringA>(name_digit).append(name_digit);
        self->kvg_digitsof.get<StringA>(name_limb).append(name_digit);
        self->kvg_digitsof.get<StringA>(name_agent).append(name_digit);

        self->kvg_sensorsof.newNode({name_digit}, {}, StringA());
        self->kvg_sensorsof.get<StringA>(name_digit).append(name_digit);
        self->kvg_sensorsof.get<StringA>(name_limb).append(name_digit);
        self->kvg_sensorsof.get<StringA>(name_agent).append(name_digit);

        self->kvg_sensors.newNode<rai::Graph*>({name_digit}, {}, d);

        readNode(d, self->hsitoi, self->itohsi, i++);
      }
    }
  }

  kvg_objects = self->kvg.getValuesOfType<rai::Graph>("object");
  for(rai::Graph* o: kvg_objects) {
    o->get(name_object, "name");
    self->objects.append(name_object);
    if(o->get(structured, "structured") && structured) {
      kvg_parts = o->getValuesOfType<rai::Graph>("part");
      self->kvg_sensorsof.newNode({name_object}, {}, StringA());
      for(rai::Graph* p: kvg_parts) {
        p->get(name_part, "name");
        self->struct_sensors.append(name_part);
        self->sensors.append(name_part);

        self->kvg_sensorsof.get<StringA>(name_object).append(name_part);
        self->kvg_sensors.newNode<rai::Graph*>({name_part}, {}, p);

        readNode(p, self->hsitoi, self->itohsi, i++);
      }
    } else {
      self->unstruct_sensors.append(name_object);
      self->sensors.append(name_object);

      self->kvg_sensors.newNode<rai::Graph*>({name_object}, {}, o);
      self->kvg_sensorsof.newNode({name_object}, {}, StringA());
      self->kvg_sensorsof.get<StringA>(name_object).append(name_object);

      readNode(o, self->hsitoi, self->itohsi, i++);
    }
  }
}

const StringA& G4ID::sensors() { return self->sensors; }
const StringA& G4ID::struct_sensors() { return self->struct_sensors; }
const StringA& G4ID::unstruct_sensors() { return self->unstruct_sensors; }
const StringA& G4ID::sensorsof(const char* obj) { return self->kvg_sensorsof.get<StringA>(STRING(obj)); }

const StringA& G4ID::subjects() { return self->subjects; }
const StringA& G4ID::objects() { return self->objects; }

const StringA& G4ID::agents() { return self->agents; }
const StringA& G4ID::limbs() { return self->limbs; }
const StringA& G4ID::digits() { return self->digits; }

const StringA& G4ID::digitsof(const rai::String& limb) { return self->kvg_digitsof.get<StringA>(limb); }
const StringA& G4ID::sublimbs(const rai::String& limb) { return self->kvg_sublimbs.get<StringA>(limb); }
const rai::String& G4ID::suplimb(const rai::String& limb) { return self->kvg_suplimbs.get<rai::String>(limb); }

uint G4ID::hsitoi(uint hsi) {
  return self->hsitoi(hsi);
}

uint G4ID::itohsi(uint i) {
  return self->itohsi(i);
}

int G4ID::i(const char* sensor) {
  return self->hsitoi(hsi(sensor));
}

int G4ID::hsi(const char* sensor) {
  rai::Graph* skvg = &self->kvg_sensors.get<rai::Graph>(sensor);

  uint hid = skvg->get<double>("hid");
  uint sid = skvg->get<double>("sid");

  return HSI(hid, sid);
}

const char* G4ID::sensor(uint hsi) {
  return self->sensors(self->hsitoi(hsi));
}

void G4ID::write(std::ostream& os) const {
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
  params.newNode({"wlen"}, {}, uint(120u));
  params.newNode({"thinning"}, {}, uint(12u));
}

void G4Rec::load(const char* recdir, bool interpolate) {
  dir = STRING(recdir);
  g4id.load(STRING(recdir << "meta.kvg"));

  ifstream datafin, tstampfin;
  rai::open(datafin, STRING(recdir << "poses.dat"));
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
  rai::Array<intA> missingno(nsensors), missingf(nsensors);
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
      dataquat.referToRange(data(f, i, {})(), 3, -1);
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
          rai::Quaternion q0(s0(3), s0(4), s0(5), s0(6));
          rai::Quaternion qF(sF(3), sF(4), sF(5), sF(6));
          rai::Quaternion qt;

          arr diff = sF - s0;
          for(uint tt = 0; tt < no; tt++) {
            data[t+tt][i] = s0 + diff*(tt+1.)/(no+1.);
            qt.setInterpolate((tt+1.)/(no+1.), q0, qF);
            data(t+tt, i, 3) = qt.w;
            data(t+tt, i, 4) = qt.x;
            data(t+tt, i, 5) = qt.y;
            data(t+tt, i, 6) = qt.z;
          }
        } else // set all equal to last
          for(uint tt = 0; tt < no; tt++)
            data[nframes-tt-1][i] = data[nframes-no-1][i];
      }
    }
  }
  // setting up default BAMs
  arr datatmp;
  tensorPermutation(datatmp, data, {1u, 0u, 2u});
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

    arr* ann;
    for(rai::Node* pair: kvgann) {
      ann = new arr(nframes);
      ann->setZero();
      for(rai::Node* lock: pair->graph()) {
        from = (uint)lock->graph().get<double>("from");
        to = (uint)lock->graph().get<double>("to");
        ann->operator()({from, to}) = 1;
      }
      NIY; //don't get the following
      //pair->graph().append("ann", ann);
    }
  } catch(const char* e) {
    cout << "No annotations in " << recdir << "." << endl;
  }
}

G4ID& G4Rec::id() { return g4id; }

bool G4Rec::hasAnn(const char* sensor1, const char* sensor2) {
//  for(rai::Node* pair: kvgann)
//    if(g4id.sensorsof(pair->keys(0)).contains(STRING(sensor1))
//        && g4id.sensorsof(pair->keys(1)).contains(STRING(sensor2)))
//      return true;
  NIY; //the keys(0) (1) above is broken...
  return false;
}

arr G4Rec::ann(const char* sensor1, const char* sensor2) {
//  for(rai::Node* pair: kvgann)
//    if(g4id.sensorsof(pair->keys(0)).contains(STRING(sensor1))
//        && g4id.sensorsof(pair->keys(1)).contains(STRING(sensor2)))
//      return pair->graph().get<arr>("ann");
  NIY; //the keys(0) (1) above is broken...
  return arr();
}

uint G4Rec::numSensors() const { return nsensors; }
uint G4Rec::numFrames() const { return nframes; }
uint G4Rec::numDim(const char* bam) { return kvg.get<arr>(bam).d2; }

void G4Rec::appendBam(const char* bam, const arr& data) {
  rai::Node* i = kvg.getNode(bam);

  if(!i)
    kvg.newNode(bam, {}, arr(data));
  else
    i->get<arr>() = data; // replacing
}

bool G4Rec::hasBam(const char* bam) {
  return kvg.getNode(bam) != nullptr;
}

arr G4Rec::query(const char* bam) {
  rai::Node* i = kvg.getNode(bam);
  CHECK(i != nullptr, STRING("BAM '" << bam << "' does not exist."));

  if(0 == strcmp(bam, "pose")) {
    arr data, dataPos, dataQuat;

    dataPos.referTo(kvg.get<arr>("pos"));
    dataQuat.referTo(kvg.get<arr>("quat"));
    data.append(dataPos);
    data.append(dataQuat);
    data.reshape(nsensors, nframes, 7);
    return data;
  }
  return kvg.get<arr>(bam);
}

arr G4Rec::query(const char* type, const char* sensor) {
  rai::Node* i = kvg.getNode(type);
  CHECK(i != nullptr, STRING("BAM '" << type << "' does not exist."));

  int is = g4id.i(sensor);
  CHECK_GE(is,  0, STRING("Sensor '" << sensor << "' does not exist."));

  if(0 == strcmp(type, "pose")) {
    arr xPos, xQuat;
    xPos.referTo(kvg.get<arr>("pos"));
    xQuat.referTo(kvg.get<arr>("quat"));

    uint nframes = numFrames();

    arr x;
    /* x.append(xPos); */
    /* x.append(xQuat); */
    /* x.reshape(self->g4id.sensors().N, self->nframes, 7); */
    /* return x[i]; */
    x.append(xPos[is]);
    x.append(xQuat[is]);
    x.reshape(nframes, 7);
    return x;
  }
  return i->get<arr>().operator[](is);
}

arr G4Rec::query(const char* type, const char* sensor, uint f) {
  rai::Node* i = kvg.getNode(type);
  CHECK(i != nullptr, STRING("BAM '" << type << "' does not exist."));

  int is = g4id.i(sensor);
  CHECK_GE(is,  0, STRING("Sensor '" << sensor << "' does not exist."));

  if(0 == strcmp(type, "pose")) {
    arr x;
    x.append(kvg.get<arr>("pos")(is, f, {}));
    x.append(kvg.get<arr>("quat")(is, f, {}));
    return x;
  }
  return i->get<arr>()(is, f, {});
}

/* arr G4Rec::query(const char *type, const char *sensor1, const char *sensor2) { */
/*   // TODO check that the type works for 2 sensors.... */
/*   // e.g. check that it is not "poses" */

/*   Graph *skvg1 = &self->kvg_sensors.get<Graph>(sensor1); */
/*   Graph *skvg2 = &self->kvg_sensors.get<Graph>(sensor2); */
/*   CHECK(self->kvg.getNode(type) != nullptr, STRING("BAM '" << type << "' does not exist.")); */
/*   CHECK(skvg1, STRING("Sensor '" << sensor1 << "' does not exist.")); */
/*   CHECK(skvg2, STRING("Sensor '" << sensor2 << "' does not exist.")); */

/*   uint hid1, sid1, i1; */
/*   uint hid2, sid2, i2; */

/*   hid1 = skvg1->get<double>("hid"); */
/*   sid1 = skvg1->get<double>("sid"); */
/*   i1 = self->kvg.getValue<arr>("hsitoi")->elem(HSI(hid1, sid1)); */

/*   hid2 = skvg2->get<double>("hid"); */
/*   sid2 = skvg2->get<double>("sid"); */
/*   i2 = self->kvg.getValue<arr>("hsitoi")->elem(HSI(hid2, sid2)); */

/*   return self->kvg.getValue<arr>(type)->operator()(i1, i2, {}); */
/* } */

/* arr G4Data::query(const char *type, const char *sensor1, const char *sensor2, uint f) { */
/*   Graph *skvg1 = &self->kvg_sensors.get<Graph>(sensor1); */
/*   Graph *skvg2 = &self->kvg_sensors.get<Graph>(sensor2); */
/*   CHECK(self->kvg.getNode(type) != nullptr, STRING("BAM '" << type << "' does not exist.")); */
/*   CHECK(skvg1, STRING("Sensor '" << sensor1 << "' does not exist.")); */
/*   CHECK(skvg2, STRING("Sensor '" << sensor2 << "' does not exist.")); */

/*   uint hid1, sid1, i1; */
/*   uint hid2, sid2, i2; */

/*   hid1 = skvg1->get<double>("hid"); */
/*   sid1 = skvg1->get<double>("sid"); */
/*   i1 = self->kvg.getValue<arr>("hsitoi")->elem(HSI(hid1, sid1)); */

/*   hid2 = skvg2->get<double>("hid"); */
/*   sid2 = skvg2->get<double>("sid"); */
/*   i2 = self->kvg.getValue<arr>("hsitoi")->elem(HSI(hid2, sid2)); */

/*   return self->kvg.getValue<arr>("bam", type)->operator()(i1, i2,  f, {}); */
/* } */

void G4Rec::computeDPos(const char* sensor) {
  rai::String typeDPos;
  typeDPos << sensor << "_dPos";

  arr bamDPos(nsensors, nframes, numDim("pos"));
  bamDPos.setZero();

  arr posX, quatX, posY;
  posX = query("pos", sensor);
  posY = query("pos");
  quatX = query("quat", sensor);
  rai::Vector pX, pY, p, A;
  rai::Quaternion qX;
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

void G4Rec::computeDQuat(const char* sensor) {
  rai::String typeDQuat;
  typeDQuat << sensor << "_dQuat";

  arr bamDQuat(nsensors, nframes, numDim("quat"));
  bamDQuat.setZero();

  arr quatX, quatY;
  quatX = query("quat", sensor);
  quatY = query("quat");
  rai::Quaternion qX, qY, quat, A;
  for(uint j = 0; j < nsensors; j++) {
    for(uint f = 0; f < nframes; f++) {
      qX.set(quatX[f].p);
      qY.set(quatY[j][f].p);
      if(f == 0)
        A = qX / qY;
      quat = qX / (A * qY);
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
//       window.referToRange(bam[i], wi, wi + wlen - 1);
//       windowMean = sum(window, 0) / (double)wlen;
//       windowMean = ~repmat(windowMean, 1, wlen);
//       bamVar(i, fi) = sumOfSqr(window - windowMean);
//     }
//   }
//   bamVar /= (double)wlen;

//   appendBam(typeVar, bamVar);
// }

void G4Rec::computeVar(const char* type) {
  rai::String typeVar = STRING(type << "Var");

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
      window.referToRange(bam[i], f - wlen + 1, f);
      windowMean = sum(window, 0) / (double)wlen;
      // windowMeanRep = ~repmat(windowMean, 1, wlen);
      // bamVar(i, f_thin) = sumOfSqr(window - windowMeanRep);
      bamVar(i, f_thin) = sumOfSqr(window - ~repmat(windowMean, 1, wlen));
    }
  }
  bamVar /= (double)wlen;

  appendBam(typeVar, bamVar);
}

void G4Rec::computeVar(const StringA& types) {
  for(const rai::String& type: types)
    computeVar(type);
}

G4RawSeq G4Rec::rawseq(const char* sens1, const char* sens2) {
  G4RawSeq seq;
  seq.set(*this, sens1, sens2);
  return seq;
}

G4RawSeqL G4Rec::rawseqlist(const char* obj1, const char* obj2) {
  G4RawSeqL seqlist;
  G4RawSeq* seq;

  const StringA sensors1 = (obj1? g4id.sensorsof(obj1): g4id.sensors());
  const StringA sensors2 = (obj2? g4id.sensorsof(obj2): g4id.sensors());
  for(const rai::String& sens1: sensors1) {
    for(const rai::String& sens2: sensors2) {
      seq = new G4RawSeq();
      seq->set(*this, sens1, sens2);
      seqlist.append(seq);
    }
  }
  return seqlist;
}

G4FeatSeq G4Rec::featseq(const char* sens1, const char* sens2) {
  G4FeatSeq seq;
  seq.set(*this, sens1, sens2);
  return seq;
}

G4FeatSeqL G4Rec::featseqlist(bool with_ann, const char* obj1, const char* obj2) {
  G4FeatSeqL seqlist;
  G4FeatSeq* seq;

  bool checklist[nsensors][nsensors];
  uint i1, i2;
  for(i1 = 0; i1 < nsensors; i1++)
    for(i2 = 0; i2 < nsensors; i2++)
      checklist[i1][i2] = false;

  StringA sensors1 = (obj1? g4id.sensorsof(obj1): g4id.sensors());
  StringA sensors2 = (obj2? g4id.sensorsof(obj2): g4id.sensors());
  for(const rai::String& sens1: sensors1) {
    for(const rai::String& sens2: sensors2) {
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

void G4Rec::write(std::ostream& os) const {
  os << "G4Rec: " << dir << endl;
}

// =============================================================================
// G4Data
//

G4Data::G4Data() { }
G4Data::~G4Data() { }

rai::String& G4Data::base() { return basedir; }

void G4Data::load(const char* recdir, bool interpolate) {
  G4Rec* g4rec = new G4Rec();
  g4rec->load(STRING(basedir << recdir << "/"), interpolate);
  kvg.newNode({recdir}, {}, g4rec);
}

G4Rec& G4Data::rec(const char* recdir) {
  rai::Node* i = kvg.getNode(recdir);
  CHECK(i, STRING("No recording named '" << recdir << "'."));
  return i->get<G4Rec>();
}

#if 0

void G4Data::clear() {
  for(G4ID* g4id: self->kvg.getValuesOfType<G4ID>("g4id"))
    delete g4id;
  for(arr* bam: self->kvg.getValuesOfType<arr>("bam"))
    delete bam;
  self->kvg.clear();
}

void G4Data::save(const char* data_fname) {
  /* cout << " * Saving.." << flush; */
  /* self->kvg >> FILE(data_fname); */
  /* cout << " DONE!" << endl; */
}

#endif

void G4Data::write(std::ostream& os) const {
  os << "G4Data" << endl;
  for(rai::Node* i: kvg) {
    os << " * " << i->key << endl;
  }
}

// =============================================================================
// G4RawSeq
//

G4RawSeq::G4RawSeq() { }
G4RawSeq::G4RawSeq(const G4RawSeq& g4rawseq) {
  // TODO maybe better copy constructor?
  sensor1 = g4rawseq.sensor1;
  sensor2 = g4rawseq.sensor2;
  data1 = g4rawseq.data1;
  data2 = g4rawseq.data2;
  ann = g4rawseq.ann;
  nframes = g4rawseq.nframes;
}

G4RawSeq::~G4RawSeq() { }

void G4RawSeq::set(G4Rec& g4rec, const char* sens1, const char* sens2) {
  G4ID& g4id = g4rec.id();

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

void G4RawSeq::write(std::ostream& os) const {
  os << "G4RawSeq: " << endl;
  os << " * sensors: " << sensor1 << ", " << sensor2 << endl;
  os << " * dims: " << data1.dim() << ", " << data2.dim() << endl;
  os << " * ann: " << (ann.N? "yes": "no") << endl;
}

// =============================================================================
// G4FeatSeq
//

G4FeatSeq::G4FeatSeq() { }
G4FeatSeq::G4FeatSeq(const G4FeatSeq& g4featseq) {
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

void G4FeatSeq::set(G4Rec& g4rec, const char* sens1, const char* sens2) {
  sensor1 = STRING(sens1);
  sensor2 = STRING(sens2);

  g4rec.computeVar("pos");
  g4rec.computeVar("quat");

  rai::String sens1_dPos = STRING(sens1 << "_dPos");
  rai::String sens1_dQuat = STRING(sens1 << "_dQuat");
  rai::String sens1_dPosVar = STRING(sens1_dPos << "Var");
  rai::String sens1_dQuatVar = STRING(sens1_dQuat << "Var");

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
  //params.set("thinning", thinning); mt: NIY!
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

void G4FeatSeq::write(std::ostream& os) const {
  os << "G4FeatSeq: " << endl;
  os << " * sensors: " << sensor1 << ", " << sensor2 << endl;
  os << " * dim: " << data.dim() << endl;
  os << " * ann: " << (ann.N? "yes": "no") << endl;
}
