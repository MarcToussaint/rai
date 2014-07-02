#include "g4data.h"
#include <Core/array.h>
#include <Core/util.h>
#include <Core/geo.h>
#include <Core/keyValueGraph.h>
#include <Core/registry.h>

struct G4Data::sG4Data {
  G4ID g4id;
  KeyValueGraph kvg;
  uint nframes;

  sG4Data();
  ~sG4Data();
};

G4Data::sG4Data::sG4Data(): nframes(0) { }
G4Data::sG4Data::~sG4Data() { }

G4Data::G4Data():s(new sG4Data()) { }
G4Data::~G4Data() { delete s; }

void G4Data::load(const char *dir, bool interpolate) {
  const char *data_fname = NULL;

  int hsi;

  //clear();
  if(data_fname) try {
    cout << " * Loading data from '" << data_fname << "'." << endl;
    // TODO how to avoid the following from printing???
    s->kvg << FILE(data_fname);
    return;
  }
  catch(const char *e) {
    cout << " * Data file does not exist." << endl;
  }

  s->g4id.load(STRING(dir << "meta.kvg"));

  ifstream datafin, tstampfin;
  MT::open(datafin, STRING(dir << "poses.dat"));
  tstampfin.open(STRING(dir << "poses.dat.times"));
  arr x;

  uint nsensors = s->g4id.sensors().N;

  bool m;
  boolA pm(nsensors);
  pm.setZero(false);
  uint currfnum;
  double currtstamp;
  arr data, tstamp;
  boolA missing;
  MT::Array<intA> missingno(nsensors), missingf(nsensors);
  for(s->nframes = 0; ; s->nframes++) {
    datafin >> x;
    if(tstampfin.good()) {
      tstampfin >> currfnum >> currtstamp;
      tstamp.append(currtstamp);
    }
    if(!x.N || !datafin.good()) break;
    for(uint i = 0; i < nsensors; i++) {
      hsi = s->g4id.itohsi(i);
      if(hsi != -1) {
        data.append(x[hsi]);
        m = length(x[hsi]) == 0;
        missing.append(m);
        if(m && pm(i))
          missingno(i).last()++;
        else if(m && !pm(i)) {
          missingno(i).append(1);
          missingf(i).append(s->nframes);
        }
        pm(i) = m;
      }
    }
  }
  data.reshape(s->nframes, nsensors, 7);
  missing.reshape(s->nframes, nsensors);

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
    for(uint i = 0; i < nsensors; i++) {
      for(uint j = 0; j < missingno(i).N; j++) {
        uint t = missingf(i).elem(j);
        uint no = missingno(i).elem(j);
        if(t == 0) // set all equal to first
          for(uint tt = 0; tt < no; tt++)
            data[tt][i] = data[no][i];
        else if(t+no < s->nframes) { // interpolate between t-1 and t+missingno(i)
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
            data[s->nframes-tt-1][i] = data[s->nframes-no-1][i];
      }
    }
  }

  // setting up default BAMs
  arr tdata;
  tensorPermutation(tdata, data, {1, 0, 2});
  arr pos = tdata.sub(0, -1, 0, -1, 0, 2);
  arr quat = tdata.sub(0, -1, 0, -1, 3, -1);
  arr pose; // virtual BAM

  // setting up meta-data
  appendBam("pos", pos);
  appendBam("quat", quat);
  appendBam("pose", pose);
}

void G4Data::clear() {
  s->nframes = 0;
  s->kvg.clear();
  s->g4id.clear();
}

G4ID &G4Data::id() {
  return s->g4id;
}

void G4Data::save(const char *data_fname) {
  /* cout << " * Saving.." << flush; */
  /* s->kvg >> FILE(data_fname); */
  /* cout << " DONE!" << endl; */
}


uint G4Data::numFrames() { return s->nframes; }
uint G4Data::numDim(const char *bam) {
  CHECK(s->kvg.getItem(bam) != NULL, STRING("BAM '" << bam << "' does not exist."));
  return s->kvg.getValue<arr>(bam)->d2;
}

bool G4Data::hasBam(const char *bam) {
  return s->kvg.getItem(bam) != NULL;
}

arr G4Data::query(const char *type) {
  CHECK(s->kvg.getItem(type) != NULL, STRING("BAM '" << type << "' does not exist."));

  if(0 == strcmp(type, "pose")) {
    arr x, xPos, xQuat;
    xPos.referTo(*s->kvg.getValue<arr>("pos"));
    xQuat.referTo(*s->kvg.getValue<arr>("quat"));
    x.append(xPos);
    x.append(xQuat);
    x.reshape(s->g4id.sensors().N, s->nframes, 7);
    return x;
  }
  return *s->kvg.getValue<arr>(type);
}

arr G4Data::query(const char *type, const char *sensor) {
  CHECK(s->kvg.getItem(type) != NULL, STRING("BAM '" << type << "' does not exist."));

  int i = s->g4id.i(sensor);
  CHECK(i >= 0, STRING("Sensor '" << sensor << "' does not exist."));

  if(0 == strcmp(type, "pose")) {
    arr xPos, xQuat;
    xPos.referTo(*s->kvg.getValue<arr>("pos"));
    xQuat.referTo(*s->kvg.getValue<arr>("quat"));

    arr x;
    /* x.append(xPos); */
    /* x.append(xQuat); */
    /* x.reshape(s->g4id.sensors().N, s->nframes, 7); */
    /* return x[i]; */
    x.append(xPos[i]);
    x.append(xQuat[i]);
    x.reshape(s->nframes, 7);
    return x;
  }
  return s->kvg.getValue<arr>(type)->operator[](i);
}

arr G4Data::query(const char *type, const char *sensor, uint f) {
  CHECK(s->kvg.getItem(type) != NULL, STRING("BAM '" << type << "' does not exist."));

  // TODO finish
  int i = s->g4id.i(sensor);
  CHECK(i >= 0, STRING("Sensor '" << sensor << "' does not exist."));

  if(0 == strcmp(type, "pose")) {
    arr x;
    x.append(s->kvg.getValue<arr>("pos")->subDim(i, f));
    x.append(s->kvg.getValue<arr>("quat")->subDim(i, f));
    return x;
  }
  return s->kvg.getValue<arr>(type)->subDim(i, f);
}

/* arr G4Data::query(const char *type, const char *sensor1, const char *sensor2) { */
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

// instantiation
/* template void G4Data::appendMeta(const char *, const arr&); */

/* template<typename T> */
/* void G4Data::appendMeta(const char *name, const T &data) { */
/*   /1* cout << " * Appending meta: " << name << endl; *1/ */
/*   s->kvg.append("meta", name, new T(data)); */
/* } */


void G4Data::appendBam(const char *name, const arr &data) {
  /* cout << " * Appending bam: " << name << endl; */
  Item *i = s->kvg.getItem(name);

  if(!i)
    s->kvg.append("bam", name, new arr(data));
  else {
    /* cout << " *** bam already exists. Replacing." << endl; */
    *i->getValue<arr>() = data;
  }
}

void G4Data::computeVar(const StringA &types, uint wlen, bool force) {
  for(const String &type: types)
    computeVar(type, wlen, force);
}

void G4Data::computeVar(const String &type, uint wlen, bool force) {
  String typeVar = STRING(type << "Var");
  /* cout << " * computing " << typeVar << endl; */
  if(!force && hasBam(typeVar)) {
    /* cout << " * " << typeVar << " already computed (force = 0). Skipping." << endl; */
    return;
  }

  arr x, y, win, m;

  x = query(type);
  y.resize(x.d0, x.d1);
  y.setZero();

  uint ff = wlen / 2;
  uint ft = numFrames() - ff;
  for(uint i = 0; i < x.d0; i++) {
    for(uint fi = ff; fi < ft; fi++) {
      uint wi = fi - ff;
      win.referToSubRange(x[i], wi, wi + wlen - 1);
      m = sum(win, 0) / (double)wlen;
      m = ~repmat(m, 1, wlen);
      y(i, fi) = sumOfSqr(win - m);
    }
  }
  y /= (double)wlen;

  appendBam(typeVar, y);
}

void G4Data::computeDPos(const String &b, bool force) {
  String typeDPos;
  typeDPos << b << "_dPos";
  /* cout << " * computing " << typeDPos << endl; */
  if(!force && hasBam(typeDPos)) {
    /* cout << " * " << typeDPos << " already computed (force = 0). Skipping." << endl; */
    return;
  }
  uint nsensors = s->g4id.sensors().N;
  uint nframes = numFrames();
  uint ndims = numDim("pos");

  // TODO FIXME some better format?

  arr y(nsensors, nframes, ndims);
  y.setZero();

  arr posX, quatX, posY;
  posX = query("pos", b);
  posY = query("pos");
  quatX = query("quat", b);
  ors::Vector v1, v2, v, A;
  ors::Quaternion q1;
  for(uint j = 0; j < nsensors; j++) {
    for(uint f = 0; f < nframes; f++) {
      v1.set(posX[f].p);
      v2.set(posY[j][f].p);
      q1.set(quatX[f].p);
      if(f == 0)
        A = q1 * (v2 - v1);
      v = q1 * (v2 - v1) - A;
      y[j][f]() = { v.x, v.y, v.z };
    }
  }

  appendBam(typeDPos, y);
}

void G4Data::computeDQuat(const String &b, bool force) {
  String typeDQuat;
  typeDQuat << b << "_dQuat";
  /* cout << " * computing " << typeDQuat << endl; */
  if(!force && hasBam(typeDQuat)) {
    /* cout << " * " << typeDQuat << " already computed (force = 0). Skipping." << endl; */
    return;
  }
  uint nsensors = s->g4id.sensors().N;
  uint nframes = numFrames();
  uint ndims = numDim("quat");

  arr y(nsensors, nframes, ndims);
  y.setZero();

  arr quatX, quatY;
  quatX = query("quat", b);
  quatY = query("quat");
  ors::Quaternion q1, q2, q, A;
  for(uint j = 0; j < nsensors; j++) {
    for(uint f = 0; f < nframes; f++) {
      q1.set(quatX[f].p);
      q2.set(quatY[j][f].p);
      if(f == 0)
        A = q1 / q2;
      q = q1 / ( A * q2 );
      y[j][f]() = { q.w, q.x, q.y, q.z };
    }
  }

  appendBam(typeDQuat, y);
}

void G4Data::write(std::ostream &os) const {
  os << "G4Data" << endl;
}
