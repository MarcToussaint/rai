#include "g4data.h"
#include <Core/array.h>
#include <Core/util.h>
#include <Core/geo.h>
#include <Core/keyValueGraph.h>
#include <Core/array_t.h>
#include <Core/registry.h>

// TODO not working
//REGISTER_TYPE(boolA);
//REGISTER_TYPE(uintA);
//REGISTER_TYPE(intA);
//REGISTER_TYPE(StringA);

G4Data::G4Data() { }
G4Data::~G4Data() { }

void G4Data::load(const char *data_fname, const char *meta_fname, const char *poses_fname, bool interpolate) {
  int hid, sid, hsi, hstoiN, hstoiNprev;

  if(data_fname) try {
    cout << " * Loading data from '" << data_fname << "'." << endl;
    // TODO how to avoid the following from printing???
    kvg << FILE(data_fname);
    cout << " * DONE." << endl;
    numS = *kvg.getValue<double>("numS");
    numF = *kvg.getValue<double>("numF");
    numT = kvg.getItems("bam").N;
    names = *kvg.getValue<String>("names");
    MT::Array<KeyValueGraph*> sensors = kvg.getTypedValues<KeyValueGraph>("sensor");
    for(uint i = 0; i < numS; i++)
      names.append(*sensors(i)->getValue<String>("name"));
    //kvg.append("names", new StringA(names));
    return;
  }
  catch(const char *e) {
    cout << " * Data file does not exist, loading from '" << meta_fname << "' instead." << endl;
  }

  kvg << FILE(meta_fname);

  MT::Array<KeyValueGraph*> sensors = kvg.getTypedValues<KeyValueGraph>("sensor");
  numS = sensors.N;

  hstoiN = 0;
  //intA itohs, hstoi;
  arr itohs, hstoi; // TODO arr just because of kvg..
  for(uint i = 0; i < numS; i++) {
    names.append(*sensors(i)->getValue<String>("name"));
    hid = *sensors(i)->getValue<double>("hid");
    sid = *sensors(i)->getValue<double>("sid");
    hsi = 3*hid+sid;

    if(hsi+1 > hstoiN) {
      hstoiNprev = hstoi.N;
      hstoi.resizeCopy(hsi+1);
      hstoi.subRange(hstoiNprev, hsi)() = -1;
      hstoiN = hstoiNprev;
    }
    hstoi(hsi) = i;
    itohs.append(hsi);
  }

  ifstream datafin, tstampfin;
  MT::open(datafin, poses_fname);
  tstampfin.open(STRING(poses_fname << ".times"));
  arr x;

  bool m;
  boolA pm(numS);
  pm.setZero(false);
  uint currfnum;
  double currtstamp;
  arr data, tstamp;
  boolA missing;
  MT::Array<intA> missingno(numS), missingf(numS);
  for(numF = 0; ; numF++) {
    datafin >> x;
    if(tstampfin.good()) {
      tstampfin >> currfnum >> currtstamp;
      tstamp.append(currtstamp);
    }
    if(!x.N || !datafin.good()) break;
    for(uint i = 0; i < numS; i++) {
      hsi = itohs(i);
      if(hsi != -1) {
        data.append(x[hsi]);
        m = length(x[hsi]) == 0;
        missing.append(m);
        if(m && pm(i))
          missingno(i).last()++;
        else if(m && !pm(i)) {
          missingno(i).append(1);
          missingf(i).append(numF);
        }
        pm(i) = m;
      }
    }
  }
  data.reshape(numF, numS, 7);
  missing.reshape(numF, numS);

  // setting quaternions as a continuous path on the 4d sphere
  arr xprev = data[0].sub(0, -1, 3, -1);
  for(uint f = 1; f < data.d0; f++) {
    for(uint i = 0; i < data.d1; i++) {
      x.referToSubRange(data.subDim(f, i)(), 3, -1);
      if(sum(x % xprev[i]) < 0)
        x *= -1.;
      if(!length(xprev[i]) || length(x))
        xprev[i]() = x;
    }
  }

  if(interpolate) { // interpolating missing measures
    for(uint i = 0; i < numS; i++) {
      for(uint j = 0; j < missingno(i).N; j++) {
        uint t = missingf(i).elem(j);
        uint no = missingno(i).elem(j);
        if(t == 0) // set all equal to first
          for(uint tt = 0; tt < no; tt++)
            data[tt][i] = data[no][i];
        else if(t+no < numF) { // interpolate between t-1 and t+missingno(i)
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
            data[numF-tt-1][i] = data[numF-no-1][i];
      }
    }
  }

  arr tdata;
  tensorPermutation(tdata, data, {1, 0, 2});
  arr pos = tdata.sub(0, -1, 0, -1, 0, 2);
  arr quat = tdata.sub(0, -1, 0, -1, 3, -1);
  arr pose; // virtual BAM
  numT = 3;

  appendMeta("numS", (double)numS);
  appendMeta("numF", (double)numF);
  appendMeta("numT", (double)numT);
  //appendMeta("names", names);
  appendMeta("itohs", itohs);
  appendMeta("hstoi", hstoi);
  // TODO Loading these back doesn't work.
  // Probably because they are not arr..
  //appendMeta("missing", missing);
  //appendMeta("missingno", missingno);
  //appendMeta("missingf", missingf);
  appendBam("pos", pos);
  appendBam("quat", quat);
  appendBam("pose", pose);
}

void G4Data::save(const char *data_fname) {
  cout << " * Saving.." << flush;
  kvg >> FILE(data_fname);
  cout << " DONE!" << endl;
}

bool G4Data::isAgent(const String &b) {
  return kvg.getItem("sensor", b)->getValue<KeyValueGraph>()->getValue<bool>("agent") != NULL;
}

bool G4Data::isObject(const String &b) {
  return kvg.getItem("sensor", b)->getValue<KeyValueGraph>()->getValue<bool>("object") != NULL;
}

StringA G4Data::getNames() {
  return names;
  //return *kvg.getItem("meta", "names")->getValue<StringA>();
}

String G4Data::getName(uint i) {
  return names(i);
  //return kvg.getItem("meta", "names")->getValue<StringA>()->elem(i);
}

uint G4Data::getNumTypes() {
  return numT;
}

uint G4Data::getNumFrames() {
  return numF;
}

uint G4Data::getNumSensors() {
  return numS;
}

uint G4Data::getNumDim(const char *type) {
  CHECK(kvg.getItem("bam", type) != NULL, STRING("BAM '" << type << "' does not exist."));
  return kvg.getItem("bam", type)->getValue<arr>()->d2;
}

uint G4Data::getNum(const char *key) {
  return kvg.getItems(key).N;
}

bool G4Data::hasBAM(const char *type) {
  return kvg.getItem("bam", type) != NULL;
}

arr G4Data::query(const char *type) {
  CHECK(kvg.getItem("bam", type) != NULL, STRING("BAM '" << type << "' does not exist."));

  if(0 == strcmp(type, "pose")) {
    arr x, xPos, xQuat;
    xPos.referTo(*kvg.getItem("bam", "pos")->getValue<arr>());
    xQuat.referTo(*kvg.getItem("bam", "quat")->getValue<arr>());
    x.append(xPos);
    x.append(xQuat);
    x.reshape(numS, numF, 7);
    return x;
  }
  return *kvg.getItem("bam", type)->getValue<arr>();
}

arr G4Data::query(const char *type, const char *sensor) {
  CHECK(kvg.getItem("bam", type) != NULL, STRING("BAM '" << type << "' does not exist."));
  CHECK(kvg.getItem("sensor", sensor) != NULL, STRING("Sensor '" << type << "' does not exist."));
  KeyValueGraph *skvg;
  uint hid, sid, hsi, i;
  
  skvg = kvg.getItem("sensor", sensor)->getValue<KeyValueGraph>();
  hid = *skvg->getValue<double>("hid");
  sid = *skvg->getValue<double>("sid");
  hsi = 3 * hid + sid;
  i = kvg.getValue<arr>("hstoi")->elem(hsi); // arr instead of intA
  arr x;
  if(0 == strcmp(type, "pose")) {
    arr xPos, xQuat;
    xPos.referTo(*kvg.getItem("bam", "pos")->getValue<arr>());
    xQuat.referTo(*kvg.getItem("bam", "quat")->getValue<arr>());

    x.append(xPos);
    x.append(xQuat);
    x.reshape(numS, numF, 7);
    return x[i];
  }
  return kvg.getItem("bam", type)->getValue<arr>()->operator[](i);
}

arr G4Data::query(const char *type, const char *sensor, uint f) {
  CHECK(kvg.getItem("bam", type) != NULL, STRING("BAM '" << type << "' does not exist."));
  CHECK(kvg.getItem("sensor", sensor) != NULL, STRING("Sensor '" << type << "' does not exist."));
  KeyValueGraph *skvg;
  uint hid, sid, hsi, i;
  
  skvg = kvg.getValue<KeyValueGraph>(sensor);
  hid = *skvg->getValue<double>("hid");
  sid = *skvg->getValue<double>("sid");
  hsi = 3 * hid + sid;
  i = kvg.getValue<arr>("hstoi")->elem(hsi); // arr instead of intA
  arr x;
  if(0 == strcmp(type, "pose")) {
    arr xPos, xQuat;
    xPos = kvg.getItem("bam", "pos")->getValue<arr>()->subDim(i, f);
    xQuat = kvg.getItem("bam", "quat")->getValue<arr>()->subDim(i, f);

    x.append(xPos);
    x.append(xQuat);
    return x;
  }
  return kvg.getValue<arr>(type)->subDim(i, f);
}

void G4Data::appendBam(const char *name, const arr &data) {
  cout << " * Appending bam: " << name << endl;
  Item *i = kvg.getItem("bam", name);

  if(!i)
    kvg.append("bam", name, new arr(data));
  else {
    cout << " *** bam already exists. Replacing." << endl;
    *i->getValue<arr>() = data;
  }
}

