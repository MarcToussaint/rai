#include "g4data.h"
#include <Core/array.h>
#include <Core/util.h>
#include <Core/geo.h>
#include <Core/keyValueGraph.h>
#include <Core/array_t.h>
#include <Core/registry.h>
#include <map>

struct G4Data::sG4Data {
  KeyValueGraph kvg, kvg_sensors, kvg_sublimbs, kvg_suplimbs, kvg_digitsof;
  StringA sensors, subjects, objects, agents, limbs, digits;
  uint nFrames;

  sG4Data();
  ~sG4Data();
};

// ============================================================================
// sG4Data
//

G4Data::sG4Data::sG4Data(): nFrames(0) { }
G4Data::sG4Data::~sG4Data() { }

G4Data::G4Data():s(new sG4Data()) { }
G4Data::~G4Data() { delete s; }

void G4Data::load(const char *data_fname, const char *meta_fname, const char *poses_fname, bool interpolate) {
  MT::Array<KeyValueGraph*> kvg_agents, kvg_limbs, kvg_digits, kvg_objects;
  arr itohs, hstoi; // FIXME arr just because of kvg
  int hid, sid, hsi, hstoiN, hstoiNprev, i;
  String name_agent, name_limb, name_digit, name_object;

  //clear();
  if(data_fname) try {
    cout << " * Loading data from '" << data_fname << "'." << endl;
    // TODO how to avoid the following from printing???
    s->kvg << FILE(data_fname);
    return;
  }
  catch(const char *e) {
    cout << " * Data file does not exist, loading from '" << meta_fname << "' instead." << endl;
  }

  s->kvg << FILE(meta_fname);

  hstoiN = 0;
  i = 0;

  kvg_agents = s->kvg.getTypedValues<KeyValueGraph>("agent");
  for(auto a: kvg_agents) {
    a->getValue(name_agent, "name");
    s->agents.append(name_agent);
    s->subjects.append(name_agent);

    s->kvg_suplimbs.append(name_agent, new String(""));
    s->kvg_sublimbs.append(name_agent, new StringA());

    s->kvg_digitsof.append(name_agent, new StringA());

    kvg_limbs = a->getTypedValues<KeyValueGraph>("limb");
    for(auto l: kvg_limbs) {
      l->getValue(name_limb, "name");
      s->limbs.append(name_limb);
      s->subjects.append(name_limb);

      s->kvg_suplimbs.append(name_limb, new String(name_agent));
      s->kvg_sublimbs.append(name_limb, new StringA());
      s->kvg_sublimbs.getValue<StringA>(name_agent)->append(name_limb);

      s->kvg_digitsof.append(name_limb, new StringA());

      kvg_digits = l->getTypedValues<KeyValueGraph>("digit");
      for(auto d: kvg_digits) {
        d->getValue(name_digit, "name");
        s->digits.append(name_digit);
        s->subjects.append(name_digit);
        s->sensors.append(name_digit);

        s->kvg_suplimbs.append(name_digit, new String(name_limb));
        s->kvg_sublimbs.append(name_digit, new StringA());
        s->kvg_sublimbs.getValue<StringA>(name_limb)->append(name_digit);

        s->kvg_digitsof.append(name_digit, new StringA());
        s->kvg_digitsof.getValue<StringA>(name_digit)->append(name_digit);
        s->kvg_digitsof.getValue<StringA>(name_limb)->append(name_digit);
        s->kvg_digitsof.getValue<StringA>(name_agent)->append(name_digit);

        s->kvg_sensors.append(name_digit, d);

        hid = *d->getValue<double>("hid");
        sid = *d->getValue<double>("sid");
        hsi = 3*hid + sid;

        if(hsi >= hstoi.N) {
          hstoiN = hstoi.N;
          hstoi.resizeCopy(hsi+1);
          hstoi.subRange(hstoiN, hsi)() = -1;
          //hstoiN = hstoiNprev;
        }
        hstoi(hsi) = i++;
        itohs.append(hsi);
      }
    }
  }

  kvg_objects = s->kvg.getTypedValues<KeyValueGraph>("object");
  for(auto o: kvg_objects) {
    o->getValue(name_object, "name");
    s->objects.append(name_object);
    s->sensors.append(name_object);

    s->kvg_sensors.append(name_object, o);

    hid = *o->getValue<double>("hid");
    sid = *o->getValue<double>("sid");
    hsi = 3*hid + sid;

    if(hsi >= hstoi.N) {
      hstoiN = hstoi.N;
      hstoi.resizeCopy(hsi+1);
      hstoi.subRange(hstoiN, hsi)() = -1;
      //hstoiN = hstoiNprev;
    }
    hstoi(hsi) = i++;
    itohs.append(hsi);
  }

  // OLD STUFF
  //MT::Array<KeyValueGraph*> kvg_sensors = kvg.getTypedValues<KeyValueGraph>("sensor");
  //numS = kvg_sensors.N;
  //numA = 0;
  //numO = 0;

  //hstoiN = 0;
  ////intA itohs, hstoi;
  //arr itohs, hstoi; // TODO arr just because of kvg..
  //for(uint i = 0; i < numS; i++) {
    //hid = *kvg_sensors(i)->getValue<double>("hid");
    //sid = *kvg_sensors(i)->getValue<double>("sid");
    //hsi = 3*hid+sid;

    //nameS.append(*kvg_sensors(i)->getValue<String>("name"));
    //if(kvg_sensors(i)->getValue<bool>("agent")) {
      //nameA.append(nameS.last());
      //numA++;
    //}
    //if(kvg_sensors(i)->getValue<bool>("object")) {
      //nameO.append(nameS.last());
      //numO++;
    //}

    //if(hsi+1 > hstoiN) {
      //hstoiNprev = hstoi.N;
      //hstoi.resizeCopy(hsi+1);
      //hstoi.subRange(hstoiNprev, hsi)() = -1;
      //hstoiN = hstoiNprev;
    //}
    //hstoi(hsi) = i;
    //itohs.append(hsi);
  //}

  ifstream datafin, tstampfin;
  MT::open(datafin, poses_fname);
  tstampfin.open(STRING(poses_fname << ".times"));
  arr x;

  uint nSensors = s->sensors.N;

  bool m;
  boolA pm(nSensors);
  pm.setZero(false);
  uint currfnum;
  double currtstamp;
  arr data, tstamp;
  boolA missing;
  MT::Array<intA> missingno(nSensors), missingf(nSensors);
  for(s->nFrames = 0; ; s->nFrames++) {
    datafin >> x;
    if(tstampfin.good()) {
      tstampfin >> currfnum >> currtstamp;
      tstamp.append(currtstamp);
    }
    if(!x.N || !datafin.good()) break;
    for(uint i = 0; i < nSensors; i++) {
      hsi = itohs(i);
      if(hsi != -1) {
        data.append(x[hsi]);
        m = length(x[hsi]) == 0;
        missing.append(m);
        if(m && pm(i))
          missingno(i).last()++;
        else if(m && !pm(i)) {
          missingno(i).append(1);
          missingf(i).append(s->nFrames);
        }
        pm(i) = m;
      }
    }
  }
  data.reshape(s->nFrames, nSensors, 7);
  missing.reshape(s->nFrames, nSensors);

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
    for(uint i = 0; i < nSensors; i++) {
      for(uint j = 0; j < missingno(i).N; j++) {
        uint t = missingf(i).elem(j);
        uint no = missingno(i).elem(j);
        if(t == 0) // set all equal to first
          for(uint tt = 0; tt < no; tt++)
            data[tt][i] = data[no][i];
        else if(t+no < s->nFrames) { // interpolate between t-1 and t+missingno(i)
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
            data[s->nFrames-tt-1][i] = data[s->nFrames-no-1][i];
      }
    }
  }

  arr tdata;
  tensorPermutation(tdata, data, {1, 0, 2});
  arr pos = tdata.sub(0, -1, 0, -1, 0, 2);
  arr quat = tdata.sub(0, -1, 0, -1, 3, -1);
  arr pose; // virtual BAM

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
  s->kvg >> FILE(data_fname);
  cout << " DONE!" << endl;
}

const StringA& G4Data::sensors() { return s->sensors; }
const StringA& G4Data::subjects() { return s->subjects; }
const StringA& G4Data::objects() { return s->objects; }
const StringA& G4Data::agents() { return s->agents; }
const StringA& G4Data::limbs() { return s->limbs; }
const StringA& G4Data::digits() { return s->digits; }

const StringA& G4Data::digitsof(const String &limb) { return *s->kvg_digitsof.getValue<StringA>(limb); }
const StringA& G4Data::sublimbs(const String &limb) { return *s->kvg_sublimbs.getValue<StringA>(limb); }
const String& G4Data::suplimb(const String &limb) { return *s->kvg_suplimbs.getValue<String>(limb); }

uint G4Data::numFrames() { return s->nFrames; }
uint G4Data::numDim(const char *bam) {
  CHECK(s->kvg.getItem("bam", bam) != NULL, STRING("BAM '" << bam << "' does not exist."));
  return s->kvg.getItem("bam", bam)->getValue<arr>()->d2;
}

bool G4Data::hasBAM(const char *bam) {
  return s->kvg.getItem("bam", bam) != NULL;
}

arr G4Data::query(const char *type) {
  CHECK(s->kvg.getItem("bam", type) != NULL, STRING("BAM '" << type << "' does not exist."));

  if(0 == strcmp(type, "pose")) {
    arr x, xPos, xQuat;
    xPos.referTo(*s->kvg.getItem("bam", "pos")->getValue<arr>());
    xQuat.referTo(*s->kvg.getItem("bam", "quat")->getValue<arr>());
    x.append(xPos);
    x.append(xQuat);
    x.reshape(s->sensors.N, s->nFrames, 7);
    return x;
  }
  return *s->kvg.getItem("bam", type)->getValue<arr>();
}

arr G4Data::query(const char *type, const char *sensor) {
  KeyValueGraph *skvg = s->kvg_sensors.getValue<KeyValueGraph>(sensor);
  CHECK(s->kvg.getItem("bam", type) != NULL, STRING("BAM '" << type << "' does not exist."));
  CHECK(skvg, STRING("Sensor '" << type << "' does not exist."));
  uint hid, sid, hsi, i;
  
  hid = *skvg->getValue<double>("hid");
  sid = *skvg->getValue<double>("sid");
  hsi = 3 * hid + sid;
  i = s->kvg.getValue<arr>("hstoi")->elem(hsi); // arr instead of intA
  arr x;
  if(0 == strcmp(type, "pose")) {
    arr xPos, xQuat;
    xPos.referTo(*s->kvg.getItem("bam", "pos")->getValue<arr>());
    xQuat.referTo(*s->kvg.getItem("bam", "quat")->getValue<arr>());

    x.append(xPos);
    x.append(xQuat);
    x.reshape(s->sensors.N, s->nFrames, 7);
    return x[i];
  }
  return s->kvg.getItem("bam", type)->getValue<arr>()->operator[](i);
}

arr G4Data::query(const char *type, const char *sensor, uint f) {
  KeyValueGraph *skvg = s->kvg_sensors.getValue<KeyValueGraph>(sensor);
  CHECK(s->kvg.getItem("bam", type) != NULL, STRING("BAM '" << type << "' does not exist."));
  CHECK(skvg, STRING("Sensor '" << type << "' does not exist."));
  uint hid, sid, hsi, i;
  
  hid = *skvg->getValue<double>("hid");
  sid = *skvg->getValue<double>("sid");
  hsi = 3 * hid + sid;
  i = s->kvg.getValue<arr>("hstoi")->elem(hsi); // arr instead of intA
  arr x;
  if(0 == strcmp(type, "pose")) {
    arr xPos, xQuat;
    xPos = s->kvg.getItem("bam", "pos")->getValue<arr>()->subDim(i, f);
    xQuat = s->kvg.getItem("bam", "quat")->getValue<arr>()->subDim(i, f);

    x.append(xPos);
    x.append(xQuat);
    return x;
  }
  return s->kvg.getValue<arr>(type)->subDim(i, f);
}

// instantiation
template void G4Data::appendMeta(const char *, const arr&);

template<typename T>
void G4Data::appendMeta(const char *name, const T &data) {
  cout << " * Appending meta: " << name << endl;
  s->kvg.append("meta", name, new T(data));
}


void G4Data::appendBam(const char *name, const arr &data) {
  cout << " * Appending bam: " << name << endl;
  Item *i = s->kvg.getItem("bam", name);

  if(!i)
    s->kvg.append("bam", name, new arr(data));
  else {
    cout << " *** bam already exists. Replacing." << endl;
    *i->getValue<arr>() = data;
  }
}

